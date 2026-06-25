#include "bt_nav_demo/check_battery.hpp"

namespace bt_nav_demo
{

CheckBattery::CheckBattery(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config), node_(node)
{
  std::string topic = "/battery_state";
  getInput("battery_topic", topic);

  sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      // percentage is 0.0–1.0 in the ROS message; convert to 0–100
      battery_percent_.store(msg->percentage * 100.0f);
    });
}

BT::PortsList CheckBattery::providedPorts()
{
  return {
    BT::InputPort<std::string>("battery_topic", "/battery_state", "BatteryState topic"),
    BT::InputPort<double>("min_percent", 20.0, "Minimum battery percentage required"),
  };
}

BT::NodeStatus CheckBattery::tick()
{
  double min_pct{20.0};
  getInput("min_percent", min_pct);

  const float current = battery_percent_.load();

  if (current < static_cast<float>(min_pct)) {
    RCLCPP_WARN(node_->get_logger(),
                "[CheckBattery] Battery %.1f%% is below minimum %.1f%% — aborting mission",
                current, min_pct);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CheckBattery] Battery %.1f%% — OK", current);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav_demo
