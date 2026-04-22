#!/usr/bin/env python3
"""
RUNNING NODES SEPARATELY - Complete Guide

This document explains how to run individual nodes for modular workflows.
"""

MAP_SAVER_STANDALONE = """
================================================================================
HOW TO RUN map_saver.py SEPARATELY
================================================================================

SCENARIO: You have an occupancy grid being published to /map topic and want
to save maps on-demand without running the full pipeline.

STEP 1: Prepare your system
  Terminal 1 - Run occupancy grid node:
    ros2 run lab1_1 occupancy_grid_node

  Terminal 2 - Play your bag file:
    ros2 bag play your_bag.mcap --clock
  
  Terminal 3 - (Optional) Start RViz to monitor map building:
    rviz2

STEP 2: Start map_saver node
  Terminal 4 - Run map saver:
    ros2 run lab1_1 map_saver

  You should see:
    [INFO] [map_saver]: Map Saver started. Maps will be saved to ~/maps
    [INFO] [map_saver]: Send a trigger by publishing empty message to /save_map

STEP 3: Wait for occupancy grid to build
  In RViz, watch /map topic until you're satisfied with coverage
  (typically 5-10 minutes for good mapping)

STEP 4: Save the map
  Terminal 5 - Call the save service:
    ros2 service call /save_map std_srvs/srv/Empty

  You should see:
    [INFO] [map_saver]: Saved PGM image to /home/haadi/maps/map_20260314_123456.pgm
    [INFO] [map_saver]: Saved YAML metadata to /home/haadi/maps/map_20260314_123456.yaml
    [INFO] [map_saver]: Map saved successfully: map_20260314_123456

STEP 5: Verify the map was saved
  Terminal 5 - Check the directory:
    ls -lh ~/maps/

  You should see:
    map_20260314_123456.pgm  (20KB - 200KB depending on resolution)
    map_20260314_123456.yaml (1KB - contains metadata)

OPTIONAL: Create a symlink to latest map
  Terminal 5 - Save as "latest":
    ln -sf ~/maps/map_20260314_123456.yaml ~/maps/map_latest.yaml
    ln -sf ~/maps/map_20260314_123456.pgm ~/maps/map_latest.pgm

  This way, path planner can always use ~/maps/map_latest.yaml


WITH PARAMETERS
================================================================================

Run with custom save directory:
  ros2 run lab1_1 map_saver --ros-args -p save_dir:=/path/to/custom/dir

Run with remapping (if /map comes from different topic):
  ros2 run lab1_1 map_saver --ros-args --remap /map:=/custom_map_topic


QUICK COMMAND REFERENCE
================================================================================

Start map saver:
  ros2 run lab1_1 map_saver

Save current map:
  ros2 service call /save_map std_srvs/srv/Empty

Save multiple times (for different map regions):
  ros2 service call /save_map std_srvs/srv/Empty  # First save
  ros2 service call /save_map std_srvs/srv/Empty  # Second save
  ros2 service call /save_map std_srvs/srv/Empty  # etc...

Monitor bags being saved:
  ros2 topic hz /map  # See update frequency

List saved maps:
  ls ~/maps/

View metadata of saved map:
  cat ~/maps/map_20260314_123456.yaml

Create symlink to latest:
  cd ~/maps && ln -sf map_20260314_123456.yaml map_latest.yaml

================================================================================
"""

PATH_PLANNING_CONTROLLER_PIPELINE = """
================================================================================
HOW TO RUN Path Planning + Controller WITHOUT Mapping
================================================================================

SCENARIO: You already have a saved occupancy map and want to run ONLY the
path planning and waypoint execution nodes (skip map generation).

STEP 1: Prepare your saved map
  Make sure you have a saved map at:
    ~/maps/map_YYYYMMDD_HHMMSS.yaml
    ~/maps/map_YYYYMMDD_HHMMSS.pgm

  (Or create a symlink to your latest map):
    ln -sf ~/maps/map_20260314_123456.yaml ~/maps/map_latest.yaml

STEP 2: Start your robot/simulation
  Terminal 1 - Play bag file (or start real robot):
    ros2 bag play your_bag.mcap --clock

STEP 3: Use the planning_and_control launch file
  Terminal 2 - Launch path planning + controller:
    ros2 launch lab1_1 planning_and_control.launch.py

  You should see:
    [INFO] [path_planner_rrt_star]: Path Planner Node initialized
    [INFO] [waypoint_controller]: Waypoint Controller initialized

STEP 4: Start RViz for visualization
  Terminal 3:
    rviz2
    
  Configure RViz:
    - Set Fixed Frame to "odom"
    - Add OccupancyGrid display: Topic=/map (OPTIONAL - no /map published if not using mapping)
    - Add Path display: Topic=/plan
    - Add MarkerArray display: Topic=/path_markers

STEP 5: Send goals and watch execution
  In RViz:
    - Use "Publish Point" tool
    - Click on a goal location
    - Watch red path appear
    - Robot executes waypoints automatically

STEP 6: Monitor progress
  Terminal 4 - Watch controller status:
    ros2 topic echo /controller_status

  Terminal 5 - Watch planned paths:
    ros2 topic echo /plan


WITH CUSTOM MAP FILE
================================================================================

Use a specific map instead of ~/maps/map_latest.yaml:
  ros2 launch lab1_1 planning_and_control.launch.py \
    map_file:=/path/to/your/map.yaml

Example:
  ros2 launch lab1_1 planning_and_control.launch.py \
    map_file:=~/maps/map_20260314_101234.yaml


WITH CUSTOM PARAMETERS
================================================================================

Adjust robot size (for collision checking):
  ros2 launch lab1_1 planning_and_control.launch.py \
    robot_radius:=0.25

Adjust planning quality/speed:
  ros2 launch lab1_1 planning_and_control.launch.py \
    rrt_max_iterations:=5000 \
    rrt_step_size:=0.2

Adjust control parameters individually:
  # Faster movement
  ros2 param set /waypoint_controller k_v 0.8
  ros2 param set /waypoint_controller k_w 3.0

  # More aggressive planning
  ros2 param set /path_planner_rrt_star rrt_max_iterations 3000


RUNNING NODES SEPARATELY (instead of launch file)
================================================================================

Terminal 1 (bag file):
  ros2 bag play your_bag.mcap --clock

Terminal 2 (path planner):
  ros2 run lab1_1 path_planner_rrt_star

Terminal 3 (controller):
  ros2 run lab1_1 node_waypoint_controller

Terminal 4 (visualization):
  rviz2


WORKFLOW COMPARISON
================================================================================

WORKFLOW 1: Map Generation + Planning + Execution (Full Pipeline)
  ros2 launch lab1_1 navigation_pipeline.launch.py
  
  Includes:
    - occupancy_grid_node (generates map)
    - map_saver (for on-demand saving)
    - path_planner_rrt_star (plans paths)
    - node_waypoint_controller (executes paths)
  
  Use this when: Starting from scratch with sensor data

WORKFLOW 2: Map Saving Only
  ros2 run lab1_1 occupancy_grid_node
  ros2 run lab1_1 map_saver
  ros2 service call /save_map std_srvs/srv/Empty
  
  Use this when: You want to just build and save a map

WORKFLOW 3: Planning + Execution (Requires Pre-saved Map)
  ros2 launch lab1_1 planning_and_control.launch.py
  
  Includes:
    - path_planner_rrt_star (uses saved map)
    - node_waypoint_controller (executes paths)
  
  Use this when: You already have a good map and want to test planning/control


TYPICAL USE CASES
================================================================================

USE CASE 1: Generate a map and save it
  Step 1: ros2 bag play bag.mcap --clock
  Step 2: ros2 run lab1_1 occupancy_grid_node
  Step 3: rviz2 (watch mapping)
  Step 4: ros2 run lab1_1 map_saver
  Step 5: ros2 service call /save_map std_srvs/srv/Empty
  Step 6: Create symlink: ln -sf ~/maps/map_*.yaml ~/maps/map_latest.yaml

USE CASE 2: Test path planning and control on saved map
  Step 1: ros2 bag play bag.mcap --clock
  Step 2: ros2 launch lab1_1 planning_and_control.launch.py
  Step 3: rviz2
  Step 4: Click goals in RViz

USE CASE 3: Full autonomous navigation from scratch
  Step 1: ros2 bag play bag.mcap --clock
  Step 2: ros2 launch lab1_1 navigation_pipeline.launch.py
  Step 3: rviz2 (watch everything)
  Step 4: ros2 service call /save_map std_srvs/srv/Empty (when satisfied)
  Step 5: Click goals in RViz for path planning/execution


DEBUGGING
================================================================================

Check what map files exist:
  ls -lh ~/maps/

Check if map_saver is running:
  ros2 node list | grep map_saver

Check if path planner is running:
  ros2 node list | grep path_planner

Check if controller is running:
  ros2 node list | grep waypoint

Monitor map updates:
  ros2 topic echo /map -n 1

Monitor path plans:
  ros2 topic echo /plan -n 1

Monitor velocity commands:
  ros2 topic echo /turtlebot/cmd_vel

Monitor status:
  ros2 topic echo /controller_status


QUICK REFERENCE
================================================================================

MAP SAVER ONLY:
  ros2 run lab1_1 map_saver
  ros2 service call /save_map std_srvs/srv/Empty

PLANNING + CONTROL ONLY:
  ros2 launch lab1_1 planning_and_control.launch.py

PLANNING + CONTROL WITH CUSTOM MAP:
  ros2 launch lab1_1 planning_and_control.launch.py \
    map_file:=~/maps/your_map.yaml

ALL TOGETHER:
  ros2 launch lab1_1 navigation_pipeline.launch.py

================================================================================
"""

if __name__ == '__main__':
    print(MAP_SAVER_STANDALONE)
    print("\n" * 5)
    print(PATH_PLANNING_CONTROLLER_PIPELINE)
