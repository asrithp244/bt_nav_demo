cmake_minimum_required(VERSION 3.8)
project(bt_nav_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(bt_nav_demo_node
  src/main.cpp
  src/nodes/navigate_to_pose_action.cpp
  src/nodes/check_battery.cpp
  src/nodes/check_obstacle_ahead.cpp
  src/nodes/set_alternate_goal.cpp
  src/nodes/report_success.cpp
)

target_include_directories(bt_nav_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(bt_nav_demo_node PUBLIC cxx_std_17)

ament_target_dependencies(bt_nav_demo_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS bt_nav_demo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(bt_nav_demo_tests test/test_bt_nodes.py)
endif()

ament_package()
