#include "bt_nav_demo/report_success.hpp"

namespace bt_nav_demo
{

ReportSuccess::ReportSuccess(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config), node_(node)
{
}

BT::PortsList ReportSuccess::providedPorts()
{
  return {
    BT::InputPort<std::string>("message", "Mission complete.", "Message to log"),
  };
}

BT::NodeStatus ReportSuccess::tick()
{
  std::string msg{"Mission complete."};
  getInput("message", msg);
  RCLCPP_INFO(node_->get_logger(), "[ReportSuccess] %s", msg.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav_demo
