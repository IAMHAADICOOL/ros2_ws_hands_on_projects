"""
SYSTEM DATA FLOW & ARCHITECTURE
================================

                        ┌─────────────────┐
                        │   Bad File      │
                        │ (with --clock)  │
                        └────────┬────────┘
                                 │
                          /scan, /odom topics
                                 │
                ┌────────────────┴────────────────┐
                │                                 │
        ┌───────▼───────────┐          ┌─────────▼────────┐
        │ occupancy_grid    │          │                  │
        │ node.py           │          │  (TF tree needed)│
        │ - Ray casting     │          │                  │
        └───────┬───────────┘          └──────────────────┘
                │
           /map topic
          (OccupancyGrid)
                │
        ┌───────▼────────────────┐
        │    map_saver.py        │
        │ - Saves as PGM + YAML  │
        │ - Format: ~/maps/      │
        └────────────────────────┘
                │
        ┌───────▼──────────────────────────────────────────┐
        │         path_planner_rrt_star.py                 │
        │                                                  │
        │ Loads: occupancy map (from file or /map topic) │
        │ Subscribes: /goal_pose (RViz click)            │
        │ Algorithm: RRT* with circular footprint         │
        │ Publishes:                                      │
        │  - /plan (nav_msgs::Path with waypoints)       │
        │  - /path_markers (visualization)               │
        └───────┬──────────────────────────────────────────┘
                │
        ┌───────┴─────────────────────────────────────────────┐
        │                                                     │
    /plan topic                                     /path_markers (visualization)
   (waypoints)                                                 │
        │                                                     │
        └──────────┬──────────────────────────────────────────┤
                   │                                          │
        ┌──────────▼───────────────────┐                      │
        │  node_waypoint_controller.py │                      │
        │                              │──────────────────────┘
        │ Reads: /plan (waypoints)     │   │
        │ Reads: /turtlebot/odom       │   └──► RViz visualization
        │ Algorithm: Pure pursuit      │   │
        │ Publishes:                   │   └──► Red path in RViz
        │  - /turtlebot/cmd_vel        │   │
        │  - /controller_status        │   └──► Waypoint spheres
        └──────────┬────────────────────┘
                   │
         /turtlebot/cmd_vel
           (Twist commands)
                   │
        ┌──────────▼────────────┐
        │    TurtleBot Robot    │
        │  (via bag playback    │
        │   or real hardware)   │
        └───────────────────────┘


MESSAGE FLOW SEQUENCE
====================

1. BAG FILE PLAYBACK (Terminal with --clock):
   Bag → /scan messages → occupancy_grid_node
   Bag → /odom messages → occupancy_grid_node

2. MAP GENERATION:
   occupancy_grid_node → /map (OccupancyGrid)

3. MAP SAVING:
   User calls: ros2 service call /save_map std_srvs/srv/Empty
   map_saver.py → ~/maps/map_TIMESTAMP.pgm
                → ~/maps/map_TIMESTAMP.yaml

4. PATH PLANNING:
   RViz click → /goal_pose (PoseStamped)
   path_planner_rrt_star.py reads /goal_pose
   path_planner_rrt_star.py → /plan (Path with waypoints)
   path_planner_rrt_star.py → /path_markers (visualization)

5. WAYPOINT EXECUTION:
   node_waypoint_controller.py reads /plan
   For each waypoint:
     - Read current pose from /turtlebot/odom
     - Compute control commands (v, w)
     - Publish /turtlebot/cmd_vel
   Robot moves to all waypoints

6. VISUALIZATION IN RVIZ:
   Display /map → shows occupancy grid
   Display /plan → shows planned path
   Display /path_markers → shows spheres at waypoints


NODE TIMING
===========

occupancy_grid_node:
  - Timer: 0.5 Hz (every 2 seconds)
    - Updates map from last received scan
    - Publishes /map
  - Bag playback: Scans arrive at 10+ Hz

map_saver:
  - On-demand via service call
  - Blocking: Takes 1-2 seconds per save

path_planner_rrt_star:
  - On-demand when goal received
  - Blocking: Takes 0.5-2 seconds depending on parameters
  - Then publishes /plan and /path_markers

node_waypoint_controller:
  - Timer: 10 Hz (every 0.1 seconds)
    - Computes velocity commands
    - Publishes /turtlebot/cmd_vel
  - Continues until all waypoints reached


COORDINATE FRAMES
=================

Fixed Frames (from bag file):
  - "odom" (odometry frame - map reference)
  
Robot Frames:
  - "base_footprint" (robot center)
  - "rplidar" (laser sensor)

Map Frame: "odom" (where map is anchored)
Robot Frame: "base_footprint" (robot position)
Laser Frame: "rplidar" (where laser measurements come from)

All coordinates are in "odom" frame for map and path planning.


QUALITY OF RESULTS DEPENDS ON
=============================

1. Occupancy Grid Quality:
   - Grid resolution (0.05m = good detail)
   - Grid size (must cover area of interest)
   - Ray casting parameters
   - P(occupied) threshold

2. Path Planning Quality:
   - RRT* iterations (2000 = balanced)
   - Step size (0.3m = balanced)
   - Robot radius (must match actual size)
   - Map resolution relative to step size

3. Execution Quality:
   - Control gains (k_v, k_w)
   - Linear velocity limit
   - Angular velocity limit
   - Position tolerance


TYPICAL EXECUTION FLOW
=====================

BEFORE STARTING:
  1. Prepare bag file with scan and odom data
  2. Open 6 terminals or use launch file

STEP 1: GENERATE MAP (5-10 minutes)
  T1: ros2 bag play bag.mcap --clock
  T2: ros2 run lab1_1 occupancy_grid_node
  T3: rviz2 (watch map build)
  T4: ros2 service call /save_map std_srvs/srv/Empty (when satisfied)

STEP 2: SETUP PATH PLANNING (1-2 minutes)
  T5: ros2 run lab1_1 path_planner_rrt_star
  (wait for "Loaded map from..." message)

STEP 3: START CONTROLLER (immediate)
  T6: ros2 run lab1_1 node_waypoint_controller

STEP 4: PLAN AND EXECUTE (1-5 minutes)
  In RViz: Click on a goal position
  Watch red path appear
  Watch robot execute waypoints
  Monitor /controller_status for progress

SUMMARY
  1. Map gives spatial layout
  2. Path planner computes optimal path (RRT*)
  3. Controller executes path synchronously
  4. Visualization helps understand behavior
"""

if __name__ == '__main__':
    print(__doc__)
