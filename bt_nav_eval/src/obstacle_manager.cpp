#include "bt_nav_eval/obstacle_manager.hpp"

#include <sstream>
#include <iomanip>
#include <chrono>

namespace bt_nav_eval
{

ObstacleManager::ObstacleManager(rclcpp::Node * node)
: node_(node)
{
  spawn_client_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>(
    "/spawn_entity");

  delete_client_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>(
    "/delete_entity");
}

bool ObstacleManager::spawn_obstacles(const ScenarioDef & scenario)
{
  spawned_entities_.clear();

  if (scenario.obstacles.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[ObstacleManager] No obstacles for scenario %s",
                scenario.id.c_str());
    return true;
  }

  // Wait for the spawn service to be available
  if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ObstacleManager] /spawn_entity service not available — "
                 "is Gazebo running?");
    return false;
  }

  bool all_ok = true;

  for (std::size_t i = 0; i < scenario.obstacles.size(); ++i) {
    const auto & obs  = scenario.obstacles[i];
    const auto   name = entity_name(scenario.id, i);
    const auto   sdf  = make_sdf(name, obs);

    auto request       = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name      = name;
    request->xml       = sdf;
    request->reference_frame = "world";

    // Set initial pose
    request->initial_pose.position.x = obs.pose_x;
    request->initial_pose.position.y = obs.pose_y;
    request->initial_pose.position.z = 0.25;  // half box height (0.5 m tall)
    request->initial_pose.orientation.w = 1.0;

    auto future = spawn_client_->async_send_request(request);

    // Spin-wait for the service response (max 5 s)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
        break;
      }
      rclcpp::spin_some(node_->get_node_base_interface());
    }

    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[ObstacleManager] Timed out waiting for spawn response: %s",
                   name.c_str());
      all_ok = false;
      continue;
    }

    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "[ObstacleManager] Spawned: %s", name.c_str());
      spawned_entities_.push_back(name);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "[ObstacleManager] Failed to spawn %s: %s",
                  name.c_str(), response->status_message.c_str());
      all_ok = false;
    }
  }

  return all_ok;
}

void ObstacleManager::delete_obstacles(const ScenarioDef & /*scenario*/)
{
  if (spawned_entities_.empty()) {
    return;
  }

  if (!delete_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(),
                "[ObstacleManager] /delete_entity service not available — "
                "skipping cleanup");
    return;
  }

  for (const auto & name : spawned_entities_) {
    auto request  = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = name;

    auto future = delete_client_->async_send_request(request);

    // Brief spin-wait
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
        break;
      }
      rclcpp::spin_some(node_->get_node_base_interface());
    }

    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "[ObstacleManager] Deleted: %s", name.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[ObstacleManager] Could not delete %s: %s",
                    name.c_str(), response->status_message.c_str());
      }
    }
  }

  spawned_entities_.clear();
}

// ─── Private helpers ──────────────────────────────────────────────────────────

std::string ObstacleManager::entity_name(const std::string & scenario_id, std::size_t idx)
{
  return scenario_id + "_obs_" + std::to_string(idx);
}

std::string ObstacleManager::make_sdf(
  const std::string & model_name, const ObstacleDef & obs)
{
  const double box_height = 0.5;  // 50 cm tall — visible but not huge

  std::ostringstream ss;
  ss << "<?xml version='1.0'?>\n"
     << "<sdf version='1.6'>\n"
     << "  <model name='" << model_name << "'>\n"
     << "    <static>true</static>\n"
     << "    <link name='link'>\n"
     << "      <collision name='collision'>\n"
     << "        <geometry>\n"
     << "          <box><size>"
     << std::fixed << std::setprecision(3)
     << obs.size_w << " " << obs.size_h << " " << box_height
     << "</size></box>\n"
     << "        </geometry>\n"
     << "      </collision>\n"
     << "      <visual name='visual'>\n"
     << "        <geometry>\n"
     << "          <box><size>"
     << obs.size_w << " " << obs.size_h << " " << box_height
     << "</size></box>\n"
     << "        </geometry>\n"
     << "        <material>\n"
     << "          <script>\n"
     << "            <uri>file://media/materials/scripts/gazebo.material</uri>\n"
     << "            <name>Gazebo/Red</name>\n"
     << "          </script>\n"
     << "        </material>\n"
     << "      </visual>\n"
     << "    </link>\n"
     << "  </model>\n"
     << "</sdf>\n";

  return ss.str();
}

}  // namespace bt_nav_eval
