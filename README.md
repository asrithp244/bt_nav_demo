# bt_nav_demo — Autonomous Navigation with BehaviorTree.CPP + ROS 2

A ROS 2 package that uses [BehaviorTree.CPP v4](https://www.behaviortree.dev/) to implement a **mission-level autonomous task**: navigate a robot to a target, check battery and obstacle state in real time, and automatically re-route around obstacles when the primary path fails.

> **Skills demonstrated:** `BehaviorTree.CPP` · `ROS 2 Humble` · `Nav2 action client` · `rclcpp_action` · `LiDAR processing` · `C++17` · `async state machine`

---

## What it does

```
[Battery OK?] ──fail──▶ ABORT
      │ ok
      ▼
[Navigate to primary goal (3.0, 1.5)]
      │
      ├── SUCCESS ──▶ [Report mission complete]
      │
      └── FAILURE ──▶ [Obstacle ahead?]
                             │ yes
                             ▼
                    [Set alternate goal (1.0, 3.0)]
                             │
                             ▼
                    [Navigate to alternate goal]
                             │
                             └── SUCCESS ──▶ [Report mission complete]
```

## Demo

![BT execution output](image.jpeg)

> Expected output (no robot connected): CheckBattery passes, Nav2 server unavailable triggers reroute, CheckObstacleAhead fails (no LiDAR) — correct BT failure-handling behavior. Connect Nav2 or run in Gazebo for full mission success.

---


The entire mission logic lives in a single XML file (`config/nav_mission.xml`) — swap it at runtime with `bt_xml:=...` to change mission behaviour without recompiling.

---

## Package structure

```
bt_nav_demo/
├── config/
│   └── nav_mission.xml          # BT mission tree (the heart of the demo)
├── include/bt_nav_demo/
│   ├── navigate_to_pose_action.hpp
│   ├── check_battery.hpp
│   ├── check_obstacle_ahead.hpp
│   ├── set_alternate_goal.hpp
│   └── report_success.hpp
├── src/
│   ├── main.cpp                 # Node entry point, factory, tick loop
│   └── nodes/
│       ├── navigate_to_pose_action.cpp   # Async Nav2 action wrapper
│       ├── check_battery.cpp             # /battery_state condition
│       ├── check_obstacle_ahead.cpp      # /scan forward-arc condition
│       ├── set_alternate_goal.cpp        # Blackboard goal writer
│       └── report_success.cpp            # Mission success logger
├── launch/
│   └── bt_nav_demo.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Custom BT nodes

| Node | Type | What it does |
|---|---|---|
| `CheckBattery` | Condition | Reads `/battery_state`, fails if below threshold |
| `NavigateToPose` | Async Action | Sends goal to Nav2, returns RUNNING/SUCCESS/FAILURE |
| `CheckObstacleAhead` | Condition | Reads `/scan`, succeeds if obstacle < 0.5 m ahead |
| `SetAlternateGoal` | Sync Action | Writes alternate x/y to BT blackboard |
| `ReportSuccess` | Sync Action | Logs mission complete message |

---

## Build

```bash
# Prerequisites: ROS 2 Humble, Nav2, BehaviorTree.CPP v4
sudo apt install ros-humble-behaviortree-cpp ros-humble-nav2-msgs

cd ~/ros2_ws
cp -r bt_nav_demo src/
colcon build --packages-select bt_nav_demo
source install/setup.bash
```

---

## Run

### With a real robot or Nav2 in Gazebo

```bash
# Terminal 1 — start Nav2 (TurtleBot4 example)
ros2 launch turtlebot4_navigation nav2_bringup.launch.py

# Terminal 2 — run the BT mission
ros2 launch bt_nav_demo bt_nav_demo.launch.py
```

### Override the mission tree at runtime

```bash
ros2 launch bt_nav_demo bt_nav_demo.launch.py \
    bt_xml:=/path/to/my_custom_mission.xml
```

---

## Modify the mission

Edit `config/nav_mission.xml` to change goal coordinates, add new waypoints, or swap in different conditions — no recompile needed. The tree is loaded fresh on each run.

Example: add a second waypoint after the primary goal:

```xml
<Sequence name="multi_waypoint">
  <CheckBattery min_percent="20.0" />
  <NavigateToPose goal_x="3.0" goal_y="1.5" goal_yaw="0.0" />
  <NavigateToPose goal_x="5.0" goal_y="2.0" goal_yaw="1.57" />
  <ReportSuccess message="Two-waypoint mission complete!" />
</Sequence>
```

---

## Tests

```bash
colcon test --packages-select bt_nav_demo
colcon test-result --verbose
```

For Nav2 integration tests:

```bash
FULL_NAV2=1 pytest test/test_bt_nodes.py -v
```
