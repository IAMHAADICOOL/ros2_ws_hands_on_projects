# Complete Autonomous Navigation System

## Overview

This package implements a **complete autonomous navigation pipeline** for TurtleBot using:
- **Occupancy Grid Mapping** - Converts sensor data to spatial representation
- **RRT* Path Planning** - Computes collision-free optimal paths  
- **Waypoint Control** - Executes paths via proportional control

## Quick Start (3 Steps)

### Step 1: Build
```bash
cd ~/ROS2_Crash_Course/ros2_ws
colcon build --packages-select lab1_1
source install/setup.bash
```

### Step 2: Validate Setup
```bash
python3 src/lab1_1/validate_system.py
```

### Step 3: Run Everything
**Terminal 1 (Bag Playback):**
```bash
ros2 bag play your_bag.mcap --clock
```

**Terminal 2 (All Nodes):**
```bash
ros2 launch lab1_1 navigation_pipeline.launch.py
```

**Terminal 3 (Visualization):**
```bash
rviz2
```

Then in RViz:
- Set Fixed Frame to "odom"
- Add displays: Map, Path, MarkerArray (see QUICK_REFERENCE.md)
- Click "Publish Point" tool and click goals on the map
- Watch robot execute autonomously!

## System Components

### 1. **map_saver.py** - Save Occupancy Maps
Saves occupancy grids in standard ROS format (PGM + YAML)
```bash
ros2 service call /save_map std_srvs/srv/Empty
# Saved to: ~/maps/map_YYYYMMDD_HHMMSS.yaml
```

### 2. **path_planner_rrt_star.py** - RRT* Path Planning  
Plans collision-free paths with the following features:
- **Algorithm**: RRT* (Rapidly-exploring Random Tree*)
- **Collision Checking**: Circular robot footprint  
- **Parameters**: 2000 iterations, 0.3m step size (balanced)
- **Output**: `/plan` topic with waypoint list

### 3. **node_waypoint_controller.py** - Execution Control
Executes waypoint sequences sequentially:
- **Control**: Proportional controller (10 Hz)
- **Outputs**: `/turtlebot/cmd_vel` for robot motion
- **Feedback**: `/controller_status` for progress updates

## Documentation

| Document | Purpose |
|----------|---------|
| [FILE_INDEX.md](FILE_INDEX.md) | Complete file list and descriptions |
| [QUICK_REFERENCE.md](QUICK_REFERENCE.md) | Copy-paste commands and quick tips |
| [NAVIGATION_GUIDE.py](NAVIGATION_GUIDE.py) | Comprehensive system guide |
| [ARCHITECTURE.md](ARCHITECTURE.md) | System design and data flow |
| [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) | Implementation details |

## System Data Flow

```
Bag File (scan + odom)
         ↓
occupancy_grid_node → /map (OccupancyGrid)
         ↓
map_saver → ~/maps/map.{pgm,yaml}
         ↓
path_planner_rrt_star ← /goal_pose (RViz click)
         ↓
path_planner_rrt_star → /plan (Path with waypoints)
         ↓
node_waypoint_controller
         ↓
node_waypoint_controller → /turtlebot/cmd_vel
         ↓
Robot Motion
```

## Key Parameters

### RRT* Path Planning
- `robot_radius`: 0.2m (collision checking radius)
- `rrt_max_iterations`: 2000 (balance quality vs speed)
- `rrt_step_size`: 0.3m (path granularity)

### Waypoint Control
- `k_v`: 0.5 (linear velocity gain)
- `k_w`: 2.0 (angular velocity gain)  
- `dist_tolerance`: 0.1m (waypoint reached threshold)
- `v_max`: 0.5 m/s (max linear speed)
- `w_max`: 1.0 rad/s (max angular speed)

## Topics & Services

### Subscriptions
- `/map` - Occupancy grid
- `/goal_pose` - Goal position (from RViz)
- `/turtlebot/odom` - Robot odometry
- `/scan` - Laser scans
- `/tf` - Coordinate transforms

### Publications
- `/plan` - Planned waypoints
- `/turtlebot/cmd_vel` - Velocity commands
- `/path_markers` - Path visualization
- `/controller_status` - Progress feedback

### Services
- `/save_map` - Save current occupancy grid

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| "No module scipy" | `pip install scipy pyyaml pillow` |
| Path planning fails | Reduce `robot_radius`, increase `rrt_max_iterations` |
| Robot not moving | Check `/turtlebot/cmd_vel` is published, bag file playing |
| Transform errors | Increase TF buffer time, verify frame names |

See [QUICK_REFERENCE.md](QUICK_REFERENCE.md) for more troubleshooting.

## Performance

| Operation | Time |
|-----------|------|
| Map generation (40s bag) | 5-10 minutes |
| Map saving | 0.5-2 seconds |
| Path planning | 0.5-2 seconds |
| Robot execution | ~0.5 m/s (configurable) |

## Architecture Highlights

✓ **RRT* Algorithm** - Asymptotically optimal path planning  
✓ **Circular Footprint** - Realistic collision checking  
✓ **Path Smoothing** - Removes unnecessary waypoints  
✓ **Pure Pursuit Control** - Smooth following behavior  
✓ **Standard Formats** - PGM+YAML maps (nav2 compatible)  
✓ **Easy Launch** - Single launch file runs everything  

## Next Steps

1. **First Time**: Read [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
2. **Understand System**: Check [ARCHITECTURE.md](ARCHITECTURE.md)
3. **Deep Dive**: See [NAVIGATION_GUIDE.py](NAVIGATION_GUIDE.py)
4. **Customize**: Adjust parameters in [node files](lab1_1/)

## Requirements

**Python Packages:**
```bash
pip install scipy pyyaml pillow
```

**ROS Packages:**
```
rclpy, geometry_msgs, nav_msgs, sensor_msgs, 
visualization_msgs, std_srvs, std_msgs, tf2_ros
```

## Commands Summary

```bash
# Build the package
colcon build --packages-select lab1_1

# Validate installation
python3 src/lab1_1/validate_system.py

# Run individually
ros2 run lab1_1 occupancy_grid_node
ros2 run lab1_1 map_saver
ros2 run lab1_1 path_planner_rrt_star
ros2 run lab1_1 node_waypoint_controller

# Run all at once
ros2 launch lab1_1 navigation_pipeline.launch.py

# Monitoring
ros2 topic echo /controller_status
ros2 topic echo /plan

# Save map
ros2 service call /save_map std_srvs/srv/Empty
```

For detailed usage, see the documentation files above.

---

**Status**: ✅ Complete and ready for use!

Happy autonomous navigation! 🤖
