#pragma once

#include <memory>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "bt_nav_eval/scenario_types.hpp"

namespace bt_nav_eval
{

/**
 * @brief Subscribes to /odom, /scan, and /cmd_vel while a scenario is active,
 *        accumulating raw measurements.  Call reset() before each scenario and
 *        finalize() after to produce a populated ScenarioMetrics.
 *
 * Metrics collected:
 *   - path_length_m       : integrated odometry displacement
 *   - min_clearance_m     : minimum finite LaserScan range
 *   - mean/std_velocity_ms: statistics of cmd_vel linear.x
 *   - recovery_count      : injected externally from action feedback
 */
class MetricsCollector
{
public:
  /**
   * @param node  Owning node — subscriptions are created on this node's
   *              callback group / executor.
   */
  explicit MetricsCollector(rclcpp::Node * node);

  /// Clear all accumulators — call before each scenario.
  void reset();

  /**
   * @brief Compute and return a ScenarioMetrics struct.
   *
   * @param scenario_def   The scenario that was just run (for ids/distances).
   * @param result         SUCCESS / FAILURE / TIMEOUT.
   * @param elapsed_s      Wall-clock seconds from goal send to result.
   * @param recovery_count Recovery activations (from action feedback).
   */
  ScenarioMetrics finalize(
    const ScenarioDef &  scenario_def,
    ScenarioResult       result,
    double               elapsed_s,
    int                  recovery_count) const;

  // Called externally by ScenarioRunner from action feedback.
  void update_recovery_count(int count);

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node * node_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_sub_;

  // Odometry accumulators
  bool   has_prev_pose_{false};
  double prev_x_{0.0}, prev_y_{0.0};
  double path_length_{0.0};

  // LaserScan accumulator
  double min_clearance_{std::numeric_limits<double>::max()};

  // cmd_vel accumulator
  std::vector<double> velocities_;

  // Recovery count (injected from action feedback)
  int recovery_count_{0};
};

}  // namespace bt_nav_eval
