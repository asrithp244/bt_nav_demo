#pragma once

#include <string>
#include <atomic>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace bt_nav_demo
{

/**
 * @brief Condition node that checks robot battery percentage.
 *
 * Subscribes to a BatteryState topic (default: /battery_state).
 * Returns SUCCESS if percentage >= min_percent, FAILURE otherwise.
 *
 * Input ports:
 *   battery_topic  (string) — topic name
 *   min_percent    (double) — minimum acceptable battery percentage (0–100)
 */
class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string & name, const BT::NodeConfig & config,
               rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
  std::atomic<float> battery_percent_{100.0f};
};

}  // namespace bt_nav_demo
