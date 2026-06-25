#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace bt_nav_demo
{

/**
 * @brief Async BT action node that sends a NavigateToPose goal to Nav2.
 *
 * Input ports:
 *   goal_x            (double)  — target x in map frame
 *   goal_y            (double)  — target y in map frame
 *   goal_yaw          (double)  — target heading in radians
 *   nav_action_server (string)  — action server name (default: /navigate_to_pose)
 *
 * Returns:
 *   SUCCESS  — Nav2 reported goal reached
 *   FAILURE  — Nav2 reported abort/cancellation or timed out
 *   RUNNING  — goal still in progress
 */
class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPoseAction(const std::string & name, const BT::NodeConfig & config,
                       rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_sent_{false};
};

}  // namespace bt_nav_demo
