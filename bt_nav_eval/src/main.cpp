#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

#include "bt_nav_eval/scenario_types.hpp"
#include "bt_nav_eval/scenario_runner.hpp"
#include "bt_nav_eval/report_generator.hpp"

namespace bt_nav_eval
{

// ─────────────────────────────────────────────────────────────────────────────
//  YAML parser — converts scenarios.yaml → vector<ScenarioDef>
// ─────────────────────────────────────────────────────────────────────────────

static std::vector<ScenarioDef> parse_scenarios(const std::string & yaml_path)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Failed to parse " + yaml_path + ": " + e.what());
  }

  const auto & scenario_list = config["scenarios"];
  if (!scenario_list || !scenario_list.IsSequence()) {
    throw std::runtime_error("scenarios.yaml must have a top-level 'scenarios' list");
  }

  std::vector<ScenarioDef> scenarios;
  scenarios.reserve(scenario_list.size());

  for (const auto & node : scenario_list) {
    ScenarioDef s;

    s.id          = node["id"].as<std::string>();
    s.description = node["description"].as<std::string>("");

    // Start pose (optional — defaults to origin)
    if (node["start_pose"]) {
      const auto & sp = node["start_pose"];
      s.start_pose.x   = sp["x"].as<double>(0.0);
      s.start_pose.y   = sp["y"].as<double>(0.0);
      s.start_pose.yaw = sp["yaw"].as<double>(0.0);
    }

    // Goal pose (required)
    if (!node["goal_pose"]) {
      throw std::runtime_error("Scenario '" + s.id + "' missing 'goal_pose'");
    }
    const auto & gp = node["goal_pose"];
    s.goal_pose.x   = gp["x"].as<double>();
    s.goal_pose.y   = gp["y"].as<double>();
    s.goal_pose.yaw = gp["yaw"].as<double>(0.0);

    // Obstacles (optional)
    if (node["obstacles"] && node["obstacles"].IsSequence()) {
      for (const auto & obs_node : node["obstacles"]) {
        ObstacleDef obs;
        obs.type    = obs_node["type"].as<std::string>("static_box");
        obs.pose_x  = obs_node["pose"]["x"].as<double>(0.0);
        obs.pose_y  = obs_node["pose"]["y"].as<double>(0.0);
        obs.size_w  = obs_node["size"]["w"].as<double>(0.5);
        obs.size_h  = obs_node["size"]["h"].as<double>(0.5);
        s.obstacles.push_back(obs);
      }
    }

    s.timeout_sec         = node["timeout_sec"].as<double>(30.0);
    s.success_threshold_m = node["success_threshold_m"].as<double>(0.25);

    scenarios.push_back(std::move(s));
  }

  return scenarios;
}

}  // namespace bt_nav_eval

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ── Resolve scenarios.yaml path ──────────────────────────────────────────
  // Priority: 1) CLI arg  2) ROS param  3) default (package share)
  std::string scenarios_path;
  std::string output_dir;

  // Temporary node just for parameter access
  {
    auto param_node = std::make_shared<rclcpp::Node>("bt_nav_eval_params");
    param_node->declare_parameter("scenarios_file", "");
    param_node->declare_parameter("output_dir", "/tmp/bt_nav_eval_results");

    param_node->get_parameter("scenarios_file", scenarios_path);
    param_node->get_parameter("output_dir", output_dir);
  }

  if (scenarios_path.empty()) {
    try {
      const std::string pkg_dir =
        ament_index_cpp::get_package_share_directory("bt_nav_eval");
      scenarios_path = pkg_dir + "/config/scenarios.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("main"),
                   "Cannot find package 'bt_nav_eval': %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Loading scenarios from: %s", scenarios_path.c_str());
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Output directory:       %s", output_dir.c_str());

  // ── Parse scenarios ───────────────────────────────────────────────────────
  std::vector<bt_nav_eval::ScenarioDef> scenarios;
  try {
    scenarios = bt_nav_eval::parse_scenarios(scenarios_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Scenario parse error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Loaded %zu scenario(s)", scenarios.size());

  // ── Create output directory ───────────────────────────────────────────────
  std::filesystem::create_directories(output_dir);

  // ── Run evaluation ────────────────────────────────────────────────────────
  auto runner = std::make_shared<bt_nav_eval::ScenarioRunner>(scenarios);

  try {
    runner->run_all_scenarios();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "ScenarioRunner error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // ── Generate report ───────────────────────────────────────────────────────
  const auto & metrics = runner->get_metrics();

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Writing report to: %s", output_dir.c_str());

  try {
    bt_nav_eval::ReportGenerator gen;
    gen.generate(metrics, output_dir);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Report generation error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "✓ metrics.json   → %s/metrics.json", output_dir.c_str());
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "✓ report         → %s/evaluation_report.md", output_dir.c_str());

  rclcpp::shutdown();
  return 0;
}
