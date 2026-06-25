#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_nav_demo
{

/**
 * @brief Logs mission success to the ROS 2 console and always returns SUCCESS.
 *
 * Input ports:
 *   message (string) — message to log
 */
class ReportSuccess : public BT::SyncActionNode
{
public:
  ReportSuccess(const std::string & name, const BT::NodeConfig & config,
                rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace bt_nav_demo
