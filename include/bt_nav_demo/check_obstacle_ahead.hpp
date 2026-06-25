#pragma once

#include <string>
#include <atomic>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace bt_nav_demo
{

/**
 * @brief Condition node that detects obstacles in the forward arc.
 *
 * Returns SUCCESS (obstacle present) if any range reading in the ±30°
 * forward arc is below obstacle_distance_m. FAILURE if path is clear.
 *
 * Designed to be used in a recovery Sequence after NavigateToPose fails:
 *   CheckObstacleAhead → SetAlternateGoal → NavigateToPose(alternate)
 *
 * Input ports:
 *   scan_topic          (string) — LaserScan topic name
 *   obstacle_distance_m (double) — threshold distance in metres
 */
class CheckObstacleAhead : public BT::ConditionNode
{
public:
  CheckObstacleAhead(const std::string & name, const BT::NodeConfig & config,
                     rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::atomic<float> min_range_ahead_{999.0f};
};

}  // namespace bt_nav_demo
