#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "bt_nav_eval/scenario_types.hpp"

namespace bt_nav_eval
{

/**
 * @brief Manages Gazebo obstacle lifecycle for each scenario.
 *
 * Spawns SDF box models before a scenario run and deletes them after,
 * giving each scenario a clean world state.
 *
 * Naming convention: obstacle entity name = "<scenario_id>_obs_<index>"
 * (e.g. "s001_straight_corridor_obs_0")
 */
class ObstacleManager
{
public:
  explicit ObstacleManager(rclcpp::Node * node);

  /**
   * @brief Spawn all obstacles defined in the scenario into Gazebo.
   *        Blocks until all SpawnEntity service calls return.
   * @return true if all obstacles spawned successfully.
   */
  bool spawn_obstacles(const ScenarioDef & scenario);

  /**
   * @brief Delete all obstacles spawned for the scenario.
   *        Blocks until all DeleteEntity service calls return.
   */
  void delete_obstacles(const ScenarioDef & scenario);

private:
  /// Build an SDF XML string for a static box obstacle.
  static std::string make_sdf(const std::string & model_name, const ObstacleDef & obs);

  /// Entity name for a given scenario + obstacle index.
  static std::string entity_name(const std::string & scenario_id, std::size_t idx);

  rclcpp::Node * node_;

  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr  spawn_client_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

  // Track spawned entity names for cleanup
  std::vector<std::string> spawned_entities_;
};

}  // namespace bt_nav_eval
