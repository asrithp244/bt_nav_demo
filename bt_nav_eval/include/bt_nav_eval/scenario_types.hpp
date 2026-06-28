#pragma once

#include <string>
#include <vector>

namespace bt_nav_eval
{

// ─────────────────────────────────────────────────────────────────────────────
//  Scenario definition (parsed from scenarios.yaml)
// ─────────────────────────────────────────────────────────────────────────────

struct Pose2D
{
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;
};

struct ObstacleDef
{
  std::string type = "static_box";  // only static_box supported in v1
  double pose_x    = 0.0;
  double pose_y    = 0.0;
  double size_w    = 0.5;           // width  (x-axis)
  double size_h    = 0.5;           // height (y-axis)
};

struct ScenarioDef
{
  std::string             id;
  std::string             description;
  Pose2D                  start_pose;
  Pose2D                  goal_pose;
  std::vector<ObstacleDef> obstacles;
  double                  timeout_sec           = 30.0;
  double                  success_threshold_m   = 0.25;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Per-scenario metrics (produced by MetricsCollector + scenario result)
// ─────────────────────────────────────────────────────────────────────────────

enum class ScenarioResult
{
  SUCCESS,
  FAILURE,
  TIMEOUT
};

inline const char * result_to_str(ScenarioResult r)
{
  switch (r) {
    case ScenarioResult::SUCCESS: return "SUCCESS";
    case ScenarioResult::FAILURE: return "FAILURE";
    case ScenarioResult::TIMEOUT: return "TIMEOUT";
  }
  return "UNKNOWN";
}

struct ScenarioMetrics
{
  std::string    id;
  std::string    description;
  ScenarioResult result            = ScenarioResult::FAILURE;
  double         time_to_goal_s    = 0.0;   // wall-clock seconds
  double         path_length_m     = 0.0;   // integrated odometry
  double         path_efficiency   = 0.0;   // path_length / euclidean distance
  double         euclidean_dist_m  = 0.0;   // straight-line start→goal
  int            replanning_count  = 0;      // Nav2 recovery events (proxy)
  double         min_clearance_m   = 999.0;  // minimum LaserScan range during run
  double         mean_velocity_ms  = 0.0;   // mean cmd_vel linear.x
  double         std_velocity_ms   = 0.0;   // std  cmd_vel linear.x
  int            recovery_count    = 0;      // from NavigateToPose feedback
};

}  // namespace bt_nav_eval
