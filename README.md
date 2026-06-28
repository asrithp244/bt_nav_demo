# bt_nav_demo — Autonomous Navigation with BehaviorTree.CPP + ROS 2

A ROS 2 package that uses [BehaviorTree.CPP v4](https://www.behaviortree.dev/) to implement a mission-level autonomous task: navigate a robot to a target, check battery and obstacle state in real time, and automatically re-route around obstacles when the primary path fails.

This repo also includes **bt_nav_eval**, a closed-loop scenario evaluation harness built on top of the same Nav2 stack. It runs a TurtleBot3 through 10 procedurally-defined obstacle scenarios in Gazebo, collects per-scenario metrics (path efficiency, clearance, recovery count, velocity), and writes a structured JSON + Markdown report.

> **Skills demonstrated:** `BehaviorTree.CPP` · `ROS 2 Humble` · `Nav2` · `Gazebo` · `rclcpp_action` · `AMCL` · `DWB planner` · `LiDAR processing` · `C++17` · `async state machines` · `closed-loop evaluation`

---

## Demo

**bt_nav_eval full run (Gazebo + RViz side by side):** https://www.youtube.com/watch?v=lsS41a8zwlo

![BT execution output](image.jpeg)

---

## What bt_nav_demo does

```
[Battery OK?] --fail--> ABORT
      | ok
      v
[Navigate to primary goal (3.0, 1.5)]
      |
      +-- SUCCESS --> [Report mission complete]
      |
      +-- FAILURE --> [Obstacle ahead?]
                             | yes
                             v
                    [Set alternate goal (1.0, 3.0)]
                             |
                             v
                    [Navigate to alternate goal]
                             |
                             +-- SUCCESS --> [Report mission complete]
```

The entire mission logic lives in a single XML file (`config/nav_mission.xml`) — swap it at runtime with `bt_xml:=...` to change mission behaviour without recompiling.

---

## What bt_nav_eval does

bt_nav_eval is a standalone ROS 2 node that acts as a test harness for Nav2 agents. For each scenario it:

1. Spawns SDF box obstacles into Gazebo via the `/spawn_entity` service
2. Sends a `NavigateToPose` action goal and monitors feedback in real time
3. Collects odometry-integrated path length, LiDAR minimum clearance, cmd_vel statistics, and recovery count
4. Tears down obstacles and moves to the next scenario
5. Writes `metrics.json` and `evaluation_report.md` to a configurable output directory

**Scenarios run:**

| ID | Description | Result |
|---|---|---|
| s001 | Straight corridor, one obstacle | FAILURE |
| s002 | Narrow 0.6 m gap between walls | SUCCESS |
| s003 | U-shaped dead end, forces recovery | SUCCESS |
| s004 | T-junction, goal around a corner | SUCCESS |
| s005 | Five scattered obstacles | TIMEOUT |
| s006 | Diagonal run with mid-course obstacles | SUCCESS |
| s007 | Goal 0.35 m from a wall | SUCCESS |
| s008 | Diagonal obstacle field | TIMEOUT |
| s009 | Goal completely enclosed (graceful failure test) | TIMEOUT |
| s010 | Clear baseline, no obstacles | TIMEOUT |

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

## Running bt_nav_eval

```bash
# Terminal 1 - Gazebo with TurtleBot3 world
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 - Nav2 navigation stack
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# Set 2D Pose Estimate in RViz, then:

# Terminal 3 - run the evaluation harness
source ~/ros2_ws/install/setup.bash
ros2 launch bt_nav_eval bt_nav_eval.launch.py output_dir:=$HOME/eval_results
```

Results are written to `~/eval_results/metrics.json` and `~/eval_results/evaluation_report.md`.

---

## Tests

```bash
colcon test --packages-select bt_nav_eval
colcon test-result --verbose
```
