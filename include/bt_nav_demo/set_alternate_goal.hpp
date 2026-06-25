#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace bt_nav_demo
{

/**
 * @brief Writes alternate goal coordinates to the BT blackboard.
 *
 * Input ports:
 *   alternate_x (double) — alternate goal x
 *   alternate_y (double) — alternate goal y
 *
 * Output ports:
 *   alternate_x (double) — written to blackboard for downstream NavigateToPose
 *   alternate_y (double) — written to blackboard for downstream NavigateToPose
 *
 * Always returns SUCCESS.
 */
class SetAlternateGoal : public BT::SyncActionNode
{
public:
  SetAlternateGoal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace bt_nav_demo
