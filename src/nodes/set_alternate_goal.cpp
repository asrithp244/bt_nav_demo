#include "bt_nav_demo/set_alternate_goal.hpp"

namespace bt_nav_demo
{

SetAlternateGoal::SetAlternateGoal(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetAlternateGoal::providedPorts()
{
  // Both input AND output on the same key so downstream NavigateToPose
  // can read {alternate_x} / {alternate_y} from the blackboard.
  return {
    BT::BidirectionalPort<double>("alternate_x", 1.0, "Alternate goal x"),
    BT::BidirectionalPort<double>("alternate_y", 3.0, "Alternate goal y"),
  };
}

BT::NodeStatus SetAlternateGoal::tick()
{
  double ax{1.0}, ay{3.0};
  getInput("alternate_x", ax);
  getInput("alternate_y", ay);

  setOutput("alternate_x", ax);
  setOutput("alternate_y", ay);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nav_demo
