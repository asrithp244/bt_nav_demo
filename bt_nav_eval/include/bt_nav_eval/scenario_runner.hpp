#pragma once

#include <memory>
#include <vector>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "bt_nav_eval/scenario_types.hpp"
#include "bt_nav_eval/metrics_collector.hpp"
#include "bt_nav_eval/obstacle_manager.hpp"

namespace bt_nav_eval
{

/**
 * @brief Orchestrates the closed-loop evaluation pipeline.
 *
 * For each ScenarioDef:
 *   SETUP     → spawn Gazebo obstacles
 *   RUNNING   → send NavigateToPose goal, collect metrics, watch timeout
 *   TEARDOWN  → delete obstacles, finalize metrics
 *
 * Uses an async action client (same pattern as bt_nav_demo's
 * NavigateToPoseAction) and a safety watchdog timer — no blocking waits
 * inside the hot loop.
 */
class ScenarioRunner : public rclcpp::Node
{
public:
  using NavAction  = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  /// State machine states for a single scenario run.
  enum class State
  {
    IDLE,
    SETUP,
    RUNNING,
    TEARDOWN,
    DONE
  };

  explicit ScenarioRunner(
    const std::vector<ScenarioDef> & scenarios,
    const std::string & nav_action_server = "/navigate_to_pose");

  /**
   * @brief Run all scenarios sequentially and block until complete.
   *        Call from main after constructing the node.
   */
  void run_all_scenarios();

  /// Access results after run_all_scenarios() returns.
  const std::vector<ScenarioMetrics> & get_metrics() const { return all_metrics_; }

private:
  // ── Single-scenario execution ─────────────────────────────────────────────
  void run_scenario(const ScenarioDef & scenario);
  void send_nav_goal(const ScenarioDef & scenario);
  void start_timeout_watchdog(double timeout_sec);
  void cancel_watchdog();

  // ── Action client callbacks ───────────────────────────────────────────────
  void on_goal_response(GoalHandle::SharedPtr handle);
  void on_feedback(
    GoalHandle::SharedPtr handle,
    const std::shared_ptr<const NavAction::Feedback> feedback);
  void on_result(const GoalHandle::WrappedResult & result);

  // ── Timeout handler (safety monitor pattern) ──────────────────────────────
  void handle_timeout();

  // ── Members ───────────────────────────────────────────────────────────────
  std::vector<ScenarioDef>    scenarios_;
  std::vector<ScenarioMetrics> all_metrics_;

  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;

  std::unique_ptr<MetricsCollector> metrics_collector_;
  std::unique_ptr<ObstacleManager>  obstacle_manager_;

  // Per-scenario state
  std::atomic<State> state_{State::IDLE};
  ScenarioResult     current_result_{ScenarioResult::FAILURE};
  rclcpp::Time       goal_start_time_;
  int                latest_recovery_count_{0};

  // Active goal handle (needed for cancellation on timeout)
  GoalHandle::SharedPtr active_goal_handle_;

  // Timeout watchdog timer
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace bt_nav_eval
