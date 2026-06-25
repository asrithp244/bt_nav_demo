#include "bt_nav_demo/navigate_to_pose_action.hpp"

#include <cmath>
#include "tf2/LinearMath/Quaternion.h"

namespace bt_nav_demo
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  std::string action_server = "/navigate_to_pose";
  getInput("nav_action_server", action_server);

  action_client_ = rclcpp_action::create_client<Nav2Goal>(node_, action_server);
}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<double>("goal_x", 0.0, "Target x in map frame"),
    BT::InputPort<double>("goal_y", 0.0, "Target y in map frame"),
    BT::InputPort<double>("goal_yaw", 0.0, "Target heading in radians"),
    BT::InputPort<std::string>("nav_action_server", "/navigate_to_pose",
                               "Nav2 action server name"),
  };
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[NavigateToPose] Action server not available after 5 s");
    return BT::NodeStatus::FAILURE;
  }

  double x{0.0}, y{0.0}, yaw{0.0};
  getInput("goal_x", x);
  getInput("goal_y", y);
  getInput("goal_yaw", yaw);

  auto goal = Nav2Goal::Goal{};
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  RCLCPP_INFO(node_->get_logger(),
              "[NavigateToPose] Sending goal (%.2f, %.2f, yaw=%.2f)", x, y, yaw);

  goal_future_ = action_client_->async_send_goal(goal);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;  // still waiting for goal acceptance
  }

  if (!goal_handle_) {
    goal_handle_ = goal_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "[NavigateToPose] Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = goal_handle_->get_status();

  switch (status) {
    case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "[NavigateToPose] Goal reached!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(node_->get_logger(), "[NavigateToPose] Goal aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(node_->get_logger(), "[NavigateToPose] Goal cancelled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void NavigateToPoseAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[NavigateToPose] Cancelling active goal");
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  goal_sent_ = false;
}

}  // namespace bt_nav_demo
