# cerveza_tb_planning

A ROS 2 Python package that provides **occupancy grid mapping** and **RRT\* motion planning** for a TurtleBot robot. The package builds a live 2D map from LiDAR data and plans collision-free paths to user-defined goals, with a proportional controller that drives the robot along the planned path.

---

## Package Overview

```
cerveza_tb_planning/
├── cerveza_tb_planning/
│   ├── grid_map.py           # GridMap data structure (log-odds, Bresenham ray-casting, inflation)
│   ├── grid_mapping.py       # ROS 2 node: builds OccupancyGrid from LaserScan + Odometry
│   ├── rrt_star.py           # Pure RRT* path planner (no ROS dependency)
│   ├── rrt_motion_planning.py# ROS 2 node: receives goals, calls RRT*, publishes path
│   └── control_tb.py         # ROS 2 node: proportional controller, drives robot to waypoints
├── launch/
│   ├── turtlebot_planning.launch.py      # Real robot launch
│   └── turtlebot_planning_sim.launch.py  # Simulation launch (+ joy teleop)
└── config/
    └── tb_map_rviz_conf.rviz             # Pre-configured RViz layout
```

---

## Nodes

### `grid_mapping`

Builds a probabilistic occupancy grid from LiDAR scans using **log-odds Bayesian updates** and Bresenham ray-casting.

| Item | Value |
|------|-------|
| Executable | `grid_mapping` |
| Subscribed topics | `/turtlebot/odom` (`nav_msgs/Odometry`), `/turtlebot/scan` (`sensor_msgs/LaserScan`) |
| Published topics | `/map` (`nav_msgs/OccupancyGrid`), `/inflated_map` (`nav_msgs/OccupancyGrid`) |
| TF broadcast | `world_enu` → `map` |
| Parameters | `is_sim` (bool, default: `true`) — adjusts LiDAR orientation offset for real vs. simulated hardware |

The inflated map (obstacle radius ≈ 0.28 m) is fed directly to the planner to ensure safe clearance.

---

### `rrt_motion_planning`

Receives a goal from RViz and computes a collision-free path using **RRT\*** with path smoothing. Replans automatically if the path becomes invalid.

| Item | Value |
|------|-------|
| Executable | `rrt_motion_planning` |
| Subscribed topics | `odom` (`nav_msgs/Odometry`), `/goal_pose` (`geometry_msgs/PoseStamped`), `/inflated_map` (`nav_msgs/OccupancyGrid`) |
| Published topics | `/planned_path` (`nav_msgs/Path`), `/path_goal_pose` (`geometry_msgs/PoseStamped`), `/debug_map` (`nav_msgs/OccupancyGrid`) |

Key planner parameters (set in code):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `delta_q` | 0.2 m | RRT step size |
| `max_iter` | 2000 | Max tree iterations per plan |
| `goal_bias` | 0.2 | Probability of sampling goal directly |
| `radius` | 0.5 m | Rewire neighbourhood radius |
| `replan_period` | 10 s | Replanning interval when path is invalidated |
| `n_trial` | 3 | Planning attempts before giving up |

---

### `control_tb`

Proportional controller that drives the TurtleBot through the waypoints published by the planner.

| Item | Value |
|------|-------|
| Executable | `control_tb` |
| Subscribed topics | `odom` (`nav_msgs/Odometry`), `/path_goal_pose` (`geometry_msgs/PoseStamped`) |
| Published topics | `cmd_vel` (`geometry_msgs/Twist`) |
| Service | `/goto` (`cerveza_tb_interfaces/GoTo`) — sends a goal directly via a service call |

Control gains: `k_v = 0.2` (linear), `k_w = 0.5` (angular). Goal tolerance: 0.1 m.

---

## Dependencies

- **ROS 2** (tested with Humble)
- [`cerveza_tb_interfaces`](../cerveza_tb_interfaces) — custom service definition (`GoTo`)
- `cerveza_tb_localization` — provides the joy teleop launch file (simulation only)
- Python packages: `numpy`, `scipy`, `bresenham`, `tf_transformations`

---

## Usage

### Real Robot

```bash
ros2 launch cerveza_tb_planning turtlebot_planning.launch.py
```

Launches `control_tb`, `grid_mapping` (real-robot LiDAR offset), `rrt_motion_planning`, a static TF publisher (`world_enu` → `odom`), and RViz.

### Simulation

```bash
ros2 launch cerveza_tb_planning turtlebot_planning_sim.launch.py
```

Same as above but with `is_sim:=true` for the grid mapping node, plus the joy teleop launch for manual control.

### Setting a Goal

In RViz, use the **2D Goal Pose** tool to publish a target pose to `/goal_pose`. The planner will compute a path and the controller will drive the robot to the goal.

---

## Architecture

```
LaserScan + Odometry
        │
        ▼
  grid_mapping ──► /map
        │
        └──► /inflated_map ──► rrt_motion_planning ◄── /goal_pose (RViz)
                                       │
                                       └──► /path_goal_pose ──► control_tb ──► cmd_vel
```

---
