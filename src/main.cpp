#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "bt_nav_demo/navigate_to_pose_action.hpp"
#include "bt_nav_demo/check_battery.hpp"
#include "bt_nav_demo/check_obstacle_ahead.hpp"
#include "bt_nav_demo/set_alternate_goal.hpp"
#include "bt_nav_demo/report_success.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bt_nav_demo_node");

  // ── Declare parameters ──────────────────────────────────────────────────
  node->declare_parameter("bt_xml", "");  // override tree file at runtime

  // ── Register all custom BT nodes ────────────────────────────────────────
  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<bt_nav_demo::NavigateToPoseAction>(
    "NavigateToPose",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<bt_nav_demo::NavigateToPoseAction>(name, config, node);
    });

  factory.registerBuilder<bt_nav_demo::CheckBattery>(
    "CheckBattery",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<bt_nav_demo::CheckBattery>(name, config, node);
    });

  factory.registerBuilder<bt_nav_demo::CheckObstacleAhead>(
    "CheckObstacleAhead",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<bt_nav_demo::CheckObstacleAhead>(name, config, node);
    });

  factory.registerNodeType<bt_nav_demo::SetAlternateGoal>("SetAlternateGoal");

  factory.registerBuilder<bt_nav_demo::ReportSuccess>(
    "ReportSuccess",
    [node](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<bt_nav_demo::ReportSuccess>(name, config, node);
    });

  // ── Load behavior tree XML ───────────────────────────────────────────────
  std::string bt_xml;
  node->get_parameter("bt_xml", bt_xml);

  if (bt_xml.empty()) {
    const std::string pkg_dir =
      ament_index_cpp::get_package_share_directory("bt_nav_demo");
    bt_xml = pkg_dir + "/config/nav_mission.xml";
  }

  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", bt_xml.c_str());

  auto tree = factory.createTreeFromFile(bt_xml);

  // ── Console logger: prints BT status changes to stdout ──────────────────
  BT::StdCoutLogger logger(tree);

  // ── Tick the tree at 10 Hz until the mission completes or fails ─────────
  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Mission SUCCESS");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Mission FAILED (status=%d)",
                 static_cast<int>(status));
  }

  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
