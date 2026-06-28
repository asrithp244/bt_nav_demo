#include "bt_nav_eval/metrics_collector.hpp"

#include <algorithm>
#include <numeric>

namespace bt_nav_eval
{

MetricsCollector::MetricsCollector(rclcpp::Node * node)
: node_(node)
{
  // Use best-effort QoS for sensor data (matches typical publisher QoS)
  auto qos = rclcpp::SensorDataQoS();

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", qos,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { on_scan(msg); });

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });
}

void MetricsCollector::reset()
{
  has_prev_pose_  = false;
  prev_x_         = 0.0;
  prev_y_         = 0.0;
  path_length_    = 0.0;
  min_clearance_  = std::numeric_limits<double>::max();
  velocities_.clear();
  recovery_count_ = 0;
}

void MetricsCollector::update_recovery_count(int count)
{
  recovery_count_ = count;
}

ScenarioMetrics MetricsCollector::finalize(
  const ScenarioDef & def,
  ScenarioResult result,
  double elapsed_s,
  int recovery_count) const
{
  ScenarioMetrics m;
  m.id          = def.id;
  m.description = def.description;
  m.result      = result;
  m.time_to_goal_s = elapsed_s;
  m.path_length_m  = path_length_;
  m.recovery_count = recovery_count;
  m.replanning_count = recovery_count;  // proxy: each recovery triggers replanning

  // Euclidean start → goal distance
  double dx = def.goal_pose.x - def.start_pose.x;
  double dy = def.goal_pose.y - def.start_pose.y;
  m.euclidean_dist_m = std::sqrt(dx * dx + dy * dy);

  // Path efficiency: ideal = 1.0, >1 = detour taken
  if (m.euclidean_dist_m > 1e-3 && m.path_length_m > 1e-3) {
    m.path_efficiency = m.euclidean_dist_m / m.path_length_m;
    m.path_efficiency = std::min(m.path_efficiency, 1.0);  // cap at 1.0
  } else {
    m.path_efficiency = (result == ScenarioResult::SUCCESS) ? 1.0 : 0.0;
  }

  // Min clearance (reset value means no LaserScan received)
  if (min_clearance_ < std::numeric_limits<double>::max()) {
    m.min_clearance_m = min_clearance_;
  } else {
    m.min_clearance_m = -1.0;  // sentinel: no scan data
  }

  // Velocity statistics
  if (!velocities_.empty()) {
    double sum  = std::accumulate(velocities_.begin(), velocities_.end(), 0.0);
    double mean = sum / static_cast<double>(velocities_.size());
    double sq_sum = 0.0;
    for (double v : velocities_) {
      sq_sum += (v - mean) * (v - mean);
    }
    m.mean_velocity_ms = mean;
    m.std_velocity_ms  = std::sqrt(sq_sum / static_cast<double>(velocities_.size()));
  }

  return m;
}

// ─── Private topic callbacks ──────────────────────────────────────────────────

void MetricsCollector::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if (has_prev_pose_) {
    double dx = x - prev_x_;
    double dy = y - prev_y_;
    path_length_ += std::sqrt(dx * dx + dy * dy);
  }

  prev_x_       = x;
  prev_y_       = y;
  has_prev_pose_ = true;
}

void MetricsCollector::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  for (float r : msg->ranges) {
    if (std::isfinite(r) && r > 0.05f) {
      min_clearance_ = std::min(min_clearance_, static_cast<double>(r));
    }
  }
}

void MetricsCollector::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  velocities_.push_back(msg->linear.x);
}

}  // namespace bt_nav_eval
