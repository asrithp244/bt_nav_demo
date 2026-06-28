#include "bt_nav_eval/scenario_runner.hpp"

#include <thread>
#include <chrono>
#include <cmath>

namespace bt_nav_eval
{

// ─────────────────────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────────────────────

ScenarioRunner::ScenarioRunner(
  const std::vector<ScenarioDef> & scenarios,
  const std::string & nav_action_server)
: rclcpp::Node("scenario_runner"),
  scenarios_(scenarios)
{
  // Declare parameters (overridable from launch file)
  declare_parameter("nav_action_server", nav_action_server);
  declare_parameter("inter_scenario_delay_s", 2.0);

  const std::string server = get_parameter("nav_action_server").as_string();

  // Async NavigateToPose action client
  nav_client_ = rclcpp_action::create_client<NavAction>(this, server);

  // Metrics and obstacle management share this node
  metrics_collector_ = std::make_unique<MetricsCollector>(this);
  obstacle_manager_  = std::make_unique<ObstacleManager>(this);

  RCLCPP_INFO(get_logger(), "[ScenarioRunner] Ready — %zu scenarios loaded",
              scenarios_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Top-level orchestration
// ─────────────────────────────────────────────────────────────────────────────

void ScenarioRunner::run_all_scenarios()
{
  const double delay_s =
    get_parameter("inter_scenario_delay_s").as_double();

  for (std::size_t i = 0; i < scenarios_.size(); ++i) {
    const auto & s = scenarios_[i];
    RCLCPP_INFO(get_logger(),
                "\n══════════════════════════════════════════\n"
                " Scenario [%zu/%zu]: %s\n"
                " %s\n"
                "══════════════════════════════════════════",
                i + 1, scenarios_.size(),
                s.id.c_str(), s.description.c_str());

    run_scenario(s);

    // Brief pause between scenarios so Nav2 / costmap can settle
    if (i + 1 < scenarios_.size()) {
      RCLCPP_INFO(get_logger(),
                  "[ScenarioRunner] Waiting %.1f s before next scenario...", delay_s);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(delay_s)));
    }
  }

  RCLCPP_INFO(get_logger(), "[ScenarioRunner] All scenarios complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Single scenario execution (SETUP → RUNNING → TEARDOWN)
// ─────────────────────────────────────────────────────────────────────────────

void ScenarioRunner::run_scenario(const ScenarioDef & scenario)
{
  // ── SETUP ────────────────────────────────────────────────────────────────
  state_          = State::SETUP;
  current_result_ = ScenarioResult::FAILURE;
  latest_recovery_count_ = 0;
  active_goal_handle_.reset();
  metrics_collector_->reset();

  bool spawned = obstacle_manager_->spawn_obstacles(scenario);
  if (!spawned) {
    RCLCPP_WARN(get_logger(),
                "[ScenarioRunner] Obstacle spawn failed — continuing anyway");
  }

  // Wait for Nav2 action server
  RCLCPP_INFO(get_logger(), "[ScenarioRunner] Waiting for NavigateToPose server...");
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(),
                 "[ScenarioRunner] NavigateToPose server not available — "
                 "marking scenario as FAILURE");
    obstacle_manager_->delete_obstacles(scenario);
    all_metrics_.push_back(
      metrics_collector_->finalize(scenario, ScenarioResult::FAILURE, 0.0, 0));
    return;
  }

  // ── RUNNING ──────────────────────────────────────────────────────────────
  state_ = State::RUNNING;
  goal_start_time_ = now();

  send_nav_goal(scenario);
  start_timeout_watchdog(scenario.timeout_sec);

  // Spin this node until the scenario finishes (result/timeout callback sets state)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(shared_from_this());

  while (rclcpp::ok() && state_ == State::RUNNING) {
    exec.spin_some(std::chrono::milliseconds(20));
  }

  exec.remove_node(shared_from_this());
  cancel_watchdog();

  // ── TEARDOWN ─────────────────────────────────────────────────────────────
  state_ = State::TEARDOWN;

  double elapsed = (now() - goal_start_time_).seconds();

  RCLCPP_INFO(get_logger(),
              "[ScenarioRunner] Scenario %s → %s (%.1f s)",
              scenario.id.c_str(),
              result_to_str(current_result_),
              elapsed);

  obstacle_manager_->delete_obstacles(scenario);

  auto metrics = metrics_collector_->finalize(
    scenario, current_result_, elapsed, latest_recovery_count_);

  // Console summary for the demo video
  RCLCPP_INFO(get_logger(),
              "  path_length=%.2f m  efficiency=%.2f  min_clear=%.2f m  "
              "recoveries=%d  mean_vel=%.2f m/s",
              metrics.path_length_m,
              metrics.path_efficiency,
              metrics.min_clearance_m,
              metrics.recovery_count,
              metrics.mean_velocity_ms);

  all_metrics_.push_back(metrics);
  state_ = State::IDLE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Goal sending (mirrors bt_nav_demo NavigateToPoseAction::onStart)
// ─────────────────────────────────────────────────────────────────────────────

void ScenarioRunner::send_nav_goal(const ScenarioDef & scenario)
{
  const auto & gp = scenario.goal_pose;

  NavAction::Goal goal{};
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp    = now();
  goal.pose.pose.position.x = gp.x;
  goal.pose.pose.position.y = gp.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, gp.yaw);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  RCLCPP_INFO(get_logger(),
              "[ScenarioRunner] Sending goal → (%.2f, %.2f, yaw=%.2f)",
              gp.x, gp.y, gp.yaw);

  auto options = rclcpp_action::Client<NavAction>::SendGoalOptions{};

  options.goal_response_callback =
    [this](GoalHandle::SharedPtr handle) { on_goal_response(handle); };

  options.feedback_callback =
    [this](GoalHandle::SharedPtr handle,
           const std::shared_ptr<const NavAction::Feedback> feedback) {
      on_feedback(handle, feedback);
    };

  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) { on_result(result); };

  nav_client_->async_send_goal(goal, options);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timeout watchdog (safety monitor pattern)
// ─────────────────────────────────────────────────────────────────────────────

void ScenarioRunner::start_timeout_watchdog(double timeout_sec)
{
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  timeout_timer_ = create_wall_timer(duration, [this]() { handle_timeout(); });
}

void ScenarioRunner::cancel_watchdog()
{
  if (timeout_timer_) {
    timeout_timer_->cancel();
    timeout_timer_.reset();
  }
}

void ScenarioRunner::handle_timeout()
{
  if (state_ != State::RUNNING) {
    return;
  }

  RCLCPP_WARN(get_logger(), "[ScenarioRunner] TIMEOUT — cancelling active goal");

  // Cancel the navigation goal
  if (active_goal_handle_) {
    nav_client_->async_cancel_goal(active_goal_handle_);
  }

  current_result_ = ScenarioResult::TIMEOUT;
  state_          = State::TEARDOWN;  // breaks the spin loop
}

// ─────────────────────────────────────────────────────────────────────────────
//  Action client callbacks
// ─────────────────────────────────────────────────────────────────────────────

void ScenarioRunner::on_goal_response(GoalHandle::SharedPtr handle)
{
  if (!handle) {
    RCLCPP_ERROR(get_logger(), "[ScenarioRunner] Goal rejected by server");
    current_result_ = ScenarioResult::FAILURE;
    state_          = State::TEARDOWN;
    return;
  }

  active_goal_handle_ = handle;
  RCLCPP_INFO(get_logger(), "[ScenarioRunner] Goal accepted by Nav2");
}

void ScenarioRunner::on_feedback(
  GoalHandle::SharedPtr /*handle*/,
  const std::shared_ptr<const NavAction::Feedback> feedback)
{
  // Track recovery count from Nav2 feedback
  latest_recovery_count_ = feedback->number_of_recoveries;
  metrics_collector_->update_recovery_count(feedback->number_of_recoveries);

  RCLCPP_DEBUG(get_logger(),
               "[ScenarioRunner] dist_remaining=%.2f  recoveries=%d",
               feedback->distance_remaining,
               feedback->number_of_recoveries);
}

void ScenarioRunner::on_result(const GoalHandle::WrappedResult & result)
{
  if (state_ != State::RUNNING) {
    return;  // already handled by timeout
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "[ScenarioRunner] NavigateToPose → SUCCEEDED");
      current_result_ = ScenarioResult::SUCCESS;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "[ScenarioRunner] NavigateToPose → ABORTED");
      current_result_ = ScenarioResult::FAILURE;
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "[ScenarioRunner] NavigateToPose → CANCELED");
      current_result_ = ScenarioResult::FAILURE;
      break;

    default:
      RCLCPP_ERROR(get_logger(), "[ScenarioRunner] NavigateToPose → UNKNOWN result");
      current_result_ = ScenarioResult::FAILURE;
      break;
  }

  state_ = State::TEARDOWN;  // breaks the spin loop in run_scenario()
}

}  // namespace bt_nav_eval
