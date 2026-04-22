#!/usr/bin/env python3
"""
QUICK REFERENCE GUIDE - Navigation System
"""

QUICK_COMMANDS = """
================================================================================
QUICK COMMANDS - Copy & Paste Ready
================================================================================

1. BUILD THE PACKAGE (after any changes):
   cd ~/ROS2_Crash_Course/ros2_ws
   colcon build --packages-select lab1_1
   source install/setup.bash

2. PLAY BAG FILE & GENERATE OCCUPANCY MAP: (Terminal 1)
   ros2 bag play ~/path_to_bag.mcap --clock

3. CREATE OCCUPANCY GRID MAP: (Terminal 2)
   ros2 run lab1_1 occupancy_grid_node

4. SAVE THE MAP: (Terminal 3)
   ros2 service call /save_map std_srvs/srv/Empty

5. START PATH PLANNER: (Terminal 4)
   ros2 run lab1_1 path_planner_rrt_star

6. START WAYPOINT CONTROLLER: (Terminal 5)
   ros2 run lab1_1 node_waypoint_controller

7. START RVIZ: (Terminal 6)
   rviz2

================================================================================
RVIZ SETUP STEPS
================================================================================

In RViz:
  1. Set Fixed Frame: "odom"
  2. Add Display (+ button at bottom):
     - OccupancyGrid: Topic=/map
     - Path: Topic=/plan
     - MarkerArray: Topic=/path_markers
     - TF (optional)

  3. Use "Publish Point" tool to click goal on map
  4. Watch path appear in red

================================================================================
TOPIC NAMES
================================================================================

/map                   - OccupancyGrid (output from occupancy_grid_node)
/goal_pose             - PoseStamped (input: click goal in RViz)
/plan                  - Path/waypoints (output from path_planner_rrt_star)
/turtlebot/cmd_vel     - Twist (output to robot: from waypoint_controller)
/controller_status     - String feedback (progress updates)
/path_markers          - MarkerArray (visualization of path)

================================================================================
SERVICE CALLS
================================================================================

Save map:
  ros2 service call /save_map std_srvs/srv/Empty

Saved maps location: ~/maps/map_YYYYMMDD_HHMMSS.*

================================================================================
DEBUG TOPICS (ros2 topic echo)
================================================================================

Monitor controller status:
  ros2 topic echo /controller_status

Monitor map updates:
  ros2 topic echo /map | grep -v data

Monitor planned path:
  ros2 topic echo /plan

Monitor velocity commands:
  ros2 topic echo /turtlebot/cmd_vel

================================================================================
PARAMETER TUNING (ros2 param set)
================================================================================

Path planning speed/quality tradeoff:
  ros2 param set /path_planner_rrt_star rrt_step_size 0.3
  (smaller = better paths, slower planning)
  (larger = faster planning, rougher paths)

Robot speed:
  ros2 param set /waypoint_controller k_v 0.5
  (increase for faster linear motion)

Robot turn rate:
  ros2 param set /waypoint_controller k_w 2.0
  (increase for faster turns)

Position tolerance (how close to waypoint):
  ros2 param set /waypoint_controller dist_tolerance 0.1
  (smaller = more precise, slower)

================================================================================
COMMON ISSUES & FIXES
================================================================================

Issue: "No module named scipy"
Fix: pip install scipy pyyaml

Issue: Path planning fails
Fix: 
  - Ensure start and goal are in free space
  - Lower robot_radius value temporarily
  - Increase rrt_max_iterations parameter

Issue: Robot not moving
Fix:
  - Check /turtlebot/cmd_vel topic is being published
  - Verify bag file is still playing (with --clock flag)
  - Check robot is not blocked by obstacles

Issue: Map looks corrupted
Fix:
  - Use timer-based update in occupancy_grid_node
  - Increase TF buffer time

================================================================================
FILE LOCATIONS
================================================================================

Source files:
  ~/ROS2_Crash_Course/ros2_ws/src/lab1_1/lab1_1/

Saved maps:
  ~/maps/

Launch file:
  ~/ROS2_Crash_Course/ros2_ws/src/lab1_1/launch/navigation_pipeline.launch.py

Full documentation:
  ~/ROS2_Crash_Course/ros2_ws/src/lab1_1/NAVIGATION_GUIDE.py

================================================================================
SINGLE COMMAND LAUNCH (after build):
================================================================================

Launch everything at once:
  ros2 launch lab1_1 navigation_pipeline.launch.py

With custom map:
  ros2 launch lab1_1 navigation_pipeline.launch.py \\
    map_file:=~/maps/your_map.yaml \\
    robot_radius:=0.25 \\
    rrt_max_iterations:=3000

================================================================================
"""

if __name__ == '__main__':
    print(QUICK_COMMANDS)
