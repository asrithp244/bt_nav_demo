# bt_nav_eval — Closed-Loop Evaluation Pipeline for Nav2 Agents

> **Automated test harness for navigation agents** — runs a battery of
> procedurally-defined obstacle scenarios, collects per-scenario performance
> metrics, and generates a structured evaluation report.

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)](https://en.cppreference.com/w/cpp/17)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## What It Is

`bt_nav_eval` is a **scenario-based agent performance benchmarking** framework
built on top of [bt_nav_demo](https://github.com/asrithp244/bt_nav_demo).
It provides **multi-scenario regression testing** for any Nav2-compatible
navigation agent by:

1. Spawning procedurally-defined obstacle worlds in Gazebo
2. Sending `NavigateToPose` goals and monitoring the agent's response
3. Collecting eight distinct performance metrics per scenario
4. Writing a machine-readable `metrics.json` + human-readable `evaluation_report.md`

This mirrors the **closed-loop evaluation pipelines** used in production
autonomy tooling (e.g. Applied Intuition's simulation-based agent benchmarking).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          scenario_runner_node                           │
│                                                                         │
│  ┌─────────────────┐   reads   ┌──────────────────┐                    │
│  │  scenarios.yaml │ ────────► │   ScenarioRunner  │ (state machine)   │
│  └─────────────────┘           │                  │                    │
│                                │  SETUP           │ ◄── ObstacleManager│
│                                │    │             │     /spawn_entity  │
│                                │    ▼             │     /delete_entity │
│                                │  RUNNING         │                    │
│                                │    │             │ ◄── MetricsCollector
│                                │    │             │     /odom          │
│                                │    │ NavigateTo  │     /scan          │
│                                │    │ Pose action │     /cmd_vel       │
│                                │    ▼             │                    │
│                                │  TEARDOWN        │                    │
│                                └──────────────────┘                    │
│                                         │                              │
│                                         ▼                              │
│                              ┌──────────────────────┐                  │
│                              │   ReportGenerator    │                  │
│                              │                      │                  │
│                              │  metrics.json        │                  │
│                              │  evaluation_report.md│                  │
│                              └──────────────────────┘                  │
└─────────────────────────────────────────────────────────────────────────┘
```

**State machine per scenario:**
```
IDLE → SETUP (spawn obstacles) → RUNNING (goal active + metrics) → TEARDOWN → IDLE
                                      │
                              timeout watchdog
                              (safety monitor pattern)
```

---

## Metrics Collected Per Scenario

| Metric | Source | Why It Matters |
|---|---|---|
| **Success / Failure / Timeout** | NavigateToPose result code | Pass/fail gate |
| **Time to goal (s)** | Wall clock, goal send → arrival | Planning latency |
| **Path length (m)** | Integrated `/odom` displacement | Actual distance travelled |
| **Path efficiency** | Euclidean dist / path length | Detour penalty (1.0 = optimal) |
| **Min clearance (m)** | Minimum `/scan` range during run | Safety margin |
| **Velocity profile** | Mean + std of `/cmd_vel` linear.x | Smoothness / aggression |
| **Recovery count** | `NavigateToPose` feedback field | Replanning frequency |
| **Replanning count** | Proxy via recovery activations | Agent robustness |

Path efficiency and min clearance are the two signals that differentiate a
"barely succeeded" run from a well-executed one — the same distinction that
matters in production AV evaluation.

---

## Scenario Suite (10 scenarios)

| ID | Description | Key Test |
|---|---|---|
| `s001_straight_corridor` | Single obstacle, straight path | Basic avoidance |
| `s002_narrow_gap` | Two walls, 0.6 m gap | DWB/MPPI squeeze |
| `s003_u_shaped_dead_end` | U-shaped pocket | Recovery behaviour |
| `s004_t_junction` | Wall across path, goal around corner | Global replanning |
| `s005_multiple_scattered` | Five obstacles | Global planner stress |
| `s006_long_path_mid_obstacles` | 10 m run, two mid-course blocks | Sustained navigation |
| `s007_wall_adjacent_goal` | Goal 0.4 m from wall | Final approach precision |
| `s008_diagonal_obstacle_field` | Obstacles at 45° | Angled approach |
| `s009_goal_unreachable` | Fully enclosed goal | Graceful failure |
| `s010_clear_baseline` | No obstacles | Efficiency ceiling |

All scenarios are defined in `config/scenarios.yaml` — add or modify without
recompiling.

---

## Package Structure

```
bt_nav_eval/
├── config/
│   └── scenarios.yaml              # All 10 scenario definitions
├── include/bt_nav_eval/
│   ├── scenario_types.hpp          # ScenarioDef, ScenarioMetrics structs
│   ├── metrics_collector.hpp       # /odom + /scan + /cmd_vel subscriber
│   ├── obstacle_manager.hpp        # Gazebo SpawnEntity/DeleteEntity wrapper
│   ├── scenario_runner.hpp         # State machine + async action client
│   └── report_generator.hpp        # JSON + Markdown writer
├── src/
│   ├── main.cpp                    # Entry point, YAML parser, report trigger
│   ├── scenario_runner.cpp
│   ├── metrics_collector.cpp
│   ├── obstacle_manager.cpp
│   └── report_generator.cpp
├── launch/
│   └── bt_nav_eval.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Build

```bash
# Prerequisites: ROS 2 Humble, Nav2, Gazebo, gazebo_ros_pkgs
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo

# Clone alongside bt_nav_demo
cd ~/ros2_ws/src
git clone https://github.com/asrithp244/bt_nav_eval.git

cd ~/ros2_ws
colcon build --packages-select bt_nav_eval
source install/setup.bash
```

---

## Run

### Full evaluation (recommended)

```bash
# Terminal 1 — Gazebo + TurtleBot3 world
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — Nav2 stack
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
  map:=$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/map/map.yaml

# Terminal 3 — evaluation harness
ros2 launch bt_nav_eval bt_nav_eval.launch.py \
  output_dir:=$HOME/eval_results
```

Results appear in `~/eval_results/`:

```
~/eval_results/
├── metrics.json           ← machine-readable, one object per scenario
└── evaluation_report.md   ← pass/fail table + aggregate stats
```

### Run a subset of scenarios

Edit `config/scenarios.yaml` to comment out scenarios, or pass a custom file:

```bash
ros2 launch bt_nav_eval bt_nav_eval.launch.py \
  scenarios_file:=/path/to/my_scenarios.yaml \
  output_dir:=/tmp/my_results
```

### Override Nav2 action server name

```bash
ros2 launch bt_nav_eval bt_nav_eval.launch.py \
  nav_action_server:=/robot1/navigate_to_pose
```

---

## Sample Output

**`metrics.json`**
```json
[
  {
    "id": "s001_straight_corridor",
    "description": "Straight corridor with one central static obstacle",
    "result": "SUCCESS",
    "time_to_goal_s": 11.40,
    "path_length_m": 5.31,
    "euclidean_dist_m": 5.00,
    "path_efficiency": 0.942,
    "replanning_count": 0,
    "min_clearance_m": 0.38,
    "mean_velocity_ms": 0.21,
    "std_velocity_ms": 0.05,
    "recovery_count": 0
  }
]
```

**`evaluation_report.md` excerpt**

| ID | Result | Time (s) | Path (m) | Efficiency | Min Clear (m) | Recoveries |
|---|:---:|---:|---:|---:|---:|---:|
| `s001_straight_corridor` | ✅ SUCCESS | 11.40 | 5.31 | 0.942 | 0.38 | 0 |
| `s002_narrow_gap` | ✅ SUCCESS | 18.20 | 4.43 | 0.904 | 0.31 | 1 |
| `s009_goal_unreachable` | ❌ FAILURE | 28.10 | 3.20 | 0.000 | 0.29 | 3 |

---

## Design Notes

**Async action client** — mirrors `bt_nav_demo`'s `NavigateToPoseAction`
pattern. The runner sends a `NavigateToPose` goal with response/feedback/result
callbacks, then spins a `SingleThreadedExecutor` in a non-blocking loop until
the scenario resolves.

**Gazebo obstacle injection** — `ObstacleManager` calls `/spawn_entity` /
`/delete_entity` (Gazebo ROS services) with procedurally-generated SDF box
models. Each scenario gets a clean world state without manual world file
editing.

**Safety watchdog** — an `rclcpp::TimerBase` independently monitors each
scenario's timeout. On expiry it cancels the active `NavigateToPose` goal and
logs `TIMEOUT` — same safety monitor pattern used in production navigation stacks.

**Report generation** — `ReportGenerator` writes both `metrics.json` (CI
ingestible) and `evaluation_report.md` (human-readable) after all scenarios
complete.

---

## Planned Extensions (v2)

- Dynamic obstacle support via Gazebo actor plugins
- Parallel scenario execution across multiple robot namespaces
- CI integration: fail the build if success rate drops below threshold
- Metric trend plots (matplotlib / gnuplot) per regression run

---

## Related

- [bt_nav_demo](https://github.com/asrithp244/bt_nav_demo) — BehaviorTree.CPP
  v4 mission controller that this harness evaluates

---

*Skills demonstrated:* `closed-loop evaluation pipeline` ·
`scenario-based agent performance benchmarking` ·
`automated test harness for navigation agents` ·
`multi-scenario regression testing` ·
`structured metrics collection and report generation` ·
`ROS 2 Humble` · `Nav2` · `Gazebo` · `rclcpp_action` · `C++17` · `async state machine`
