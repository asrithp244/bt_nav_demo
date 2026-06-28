cmake_minimum_required(VERSION 3.8)
project(bt_nav_eval)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_action     REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(nav2_msgs         REQUIRED)
find_package(gazebo_msgs       REQUIRED)
find_package(tf2               REQUIRED)
find_package(ament_index_cpp   REQUIRED)
find_package(yaml_cpp_vendor   REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(scenario_runner_node
  src/main.cpp
  src/scenario_runner.cpp
  src/metrics_collector.cpp
  src/obstacle_manager.cpp
  src/report_generator.cpp
)

target_include_directories(scenario_runner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(scenario_runner_node PUBLIC cxx_std_17)

ament_target_dependencies(scenario_runner_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  gazebo_msgs
  tf2
  ament_index_cpp
  yaml_cpp_vendor
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS scenario_runner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bt_nav_eval_tests test/test_scenario_parser.py)
endif()

ament_package()
