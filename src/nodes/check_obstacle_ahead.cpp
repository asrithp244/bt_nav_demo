#include "bt_nav_demo/check_obstacle_ahead.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bt_nav_demo
{

CheckObstacleAhead::CheckObstacleAhead(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config), node_(node)
{
  std::string topic = "/scan";
  getInput("scan_topic", topic);

  sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // Examine only the ±30° (π/6 rad) forward arc
      const float half_arc = static_cast<float>(M_PI / 6.0);
      float min_r = std::numeric_limits<float>::max();

      for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
        if (std::fabs(angle) > half_arc) continue;

        const float r = msg->ranges[i];
        if (r >= msg->range_min && r <= msg->range_max) {
          min_r = std::min(min_r, r);
        }
      }

      min_range_ahead_.store(min_r);
    });
}

BT::PortsList CheckObstacleAhead::providedPorts()
{
  return {
    BT::InputPort<std::string>("scan_topic", "/scan", "LaserScan topic"),
    BT::InputPort<double>("obstacle_distance_m", 0.5,
                          "Distance threshold for obstacle detection (metres)"),
  };
}

BT::NodeStatus CheckObstacleAhead::tick()
{
  double threshold{0.5};
  getInput("obstacle_distance_m", threshold);

  const float nearest = min_range_ahead_.load();

  if (nearest <= static_cast<float>(threshold)) {
    RCLCPP_WARN(node_->get_logger(),
                "[CheckObstacleAhead] Obstacle at %.2f m (threshold %.2f m)",
                nearest, threshold);
    return BT::NodeStatus::SUCCESS;  // SUCCESS = obstacle IS present
  }

  return BT::NodeStatus::FAILURE;  // FAILURE = path is clear
}

}  // namespace bt_nav_demo
