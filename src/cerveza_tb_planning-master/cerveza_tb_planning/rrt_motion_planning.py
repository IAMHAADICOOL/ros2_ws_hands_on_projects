#!/usr/bin/env python3

# Import required libraries
import rclpy
from rclpy.node import Node
from cerveza_tb_planning.rrt_star import RRTStar
from cerveza_tb_planning.grid_map import GridMap, map_to_msg, msg_to_map
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np

class RRTMotionPlanning(Node):
    def __init__(self):
        super().__init__('rrt_motion_planning')

        # Init RRT* planner with a simple is_valid function (no obstacles for now)
        self.rrt_star = RRTStar(is_valid_fn=self.is_cell_valid, x_min=0, x_max=10, y_min=0, y_max=10, delta_q=0.4, max_iter=2000, goal_bias=0.2, radius=0.5)

        # Internal state for robot pose and obstacle
        self.x = 0.0
        self.y = 0.0
        self.odom_received = False  # Flag to ensure we have a valid start pose before planning

        # map
        self.grid_map = None

        # path
        self.x_goal = None
        self.y_goal = None
        self.current_path = None
        self.goal_tol = 0.1  # Tolerance to consider goal reached (meters)
        self.replan_preriod = 10.0  # Time in seconds to wait before replanning if the path is not yet completed
        self.last_plan_time = self.get_clock().now()  # Timestamp of the last plan
        self.is_goal_reached = True  # Flag to indicate if the goal has been reached
        self.n_trial = 3  # Number of trials to find a path before giving up

        # Create a subscriber to receive current robot pose from odometry (for start position)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # Create a subscriber to receive goal poses from RViz
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Create a subscriber to inflated obstacle map
        self.create_subscription(OccupancyGrid, '/inflated_map', self.obstacle_cb, 10)

        # Create a publisher to publish the planned path (for visualization in RViz)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Create a publisher to send the next waypoint to the controller
        self.controller_goal_pub = self.create_publisher(PoseStamped, '/path_goal_pose', 10)

        # Repulish the map for debugging
        self.map_pub = self.create_publisher(OccupancyGrid, '/debug_map', 10)

        # Create timer to send the goal to the controller
        self.create_timer(0.1, self.publish_goal_pose_to_controller)

    def publish_goal_pose_to_controller(self):
        """ This function can be used to publish the next waypoint to the controller 
            and continously update it as the robot moves if reach to point the new goal will be published """
        
        # No goal to reach
        if self.is_goal_reached:
            if self.current_path is not None:
                self.current_path = None
            return

        # Check if we reached the final goal
        distance_to_goal = np.hypot(self.x - self.x_goal, self.y - self.y_goal)
        if distance_to_goal < self.goal_tol:
            self.get_logger().info('Goal reached!')
            self.is_goal_reached = True
            self.current_path = None
            return
        
        # Check if we need to replan
        time_since_last_plan = (self.get_clock().now() - self.last_plan_time).nanoseconds / 1e9
        needs_replan = False

        if self.current_path is None:
            needs_replan = True
        elif time_since_last_plan > self.replan_preriod and not self.is_path_valid():
            needs_replan = True

        if needs_replan:
            self.replan()

        # Follow current path
        if self.current_path is None or len(self.current_path) == 0:
            return
        
        # Publish next waypoint to controller
        next_waypoint = self.current_path[0]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'odom'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = next_waypoint[0]
        goal_msg.pose.position.y = next_waypoint[1]
        goal_msg.pose.orientation.w = 1.0
        self.controller_goal_pub.publish(goal_msg)

        # Check if waypoint reached
        distance_to_waypoint = np.hypot(self.x - next_waypoint[0], self.y - next_waypoint[1])
        if distance_to_waypoint < self.goal_tol:
            self.current_path.pop(0)

        # Publish path for visualization
        self.publish_path_visualization()

    def replan(self):
        """ Try to find a new path """
        for trial in range(self.n_trial):
            self.get_logger().info(f'Planning path (trial {trial + 1}/{self.n_trial})...')
            path = self.rrt_star.plan((self.x, self.y), (self.x_goal, self.y_goal))

            if path is not None:
                # path = self.rrt_star.smooth_path(path)
                self.get_logger().info(f'Path found with {len(path)} waypoints.')
                self.current_path = path
                self.last_plan_time = self.get_clock().now()
                return

            self.get_logger().warn(f'No path found on trial {trial + 1}/{self.n_trial}.')

        self.get_logger().error(f'No path found after {self.n_trial} trials.')
        self.current_path = None

    def is_path_valid(self):
        """ Check if all waypoints in current path are still free """
        if self.current_path is None:
            return False
        for wp in self.current_path:
            if not self.is_cell_valid(wp[0], wp[1]):
                return False
        return True
    
    def publish_path_visualization(self):
        """ Publish path for RViz """
        if self.current_path is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Add current position
        pose = PoseStamped()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

        # Add waypoints
        for waypoint in self.current_path:
            pose = PoseStamped()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def goal_cb(self, msg):
        if not self.odom_received:
            self.get_logger().warn('No odometry received yet. Cannot plan path without start pose.')
            return
        
        if self.is_goal_reached == False:
            self.get_logger().warn('Already have an active goal. Please wait until it is reached before sending a new one.')
            return
        # Extract goal position from the received message
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y

        self.is_goal_reached = False  # Reset goal reached flag for new goal

        self.get_logger().info(f'Received goal pose: ({self.x_goal:.2f}, {self.y_goal:.2f})')

    
    def odom_cb(self, msg):
        # Extract current robot position from odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_received = True
        self.get_logger().info(f'Current robot pose: ({self.x:.2f}, {self.y:.2f})', once=True)

    
    def obstacle_cb(self, msg):
        # Update the internal map representation with the received obstacle map
        self.grid_map = msg_to_map(msg)  # Convert the OccupancyGrid message to internal GridMap representation

        # update rrt_star with the new map
        self.rrt_star.set_map_min_max(
            self.grid_map.get_min_x(),
            self.grid_map.get_max_x(),
            self.grid_map.get_min_y(),
            self.grid_map.get_max_y()
        )

        self.get_logger().info('Received updated obstacle map.', once=True)

        # Republish the map for debugging
        msg = map_to_msg(self.grid_map)
        msg.info.map_load_time = self.get_clock().now().to_msg()  # Set map load time to current time
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Map frame for the occupancy grid message
        self.map_pub.publish(msg)

    def is_cell_valid(self, x, y):
        # Check if the cell (x, y) is free of obstacles based on the internal map representation
        if self.grid_map is None:
            return True  # If no map is available, assume all cells are valid
        # Convert (x, y) to map indices if necessary and check for obstacles
        map_x, map_y = self.grid_map.position_to_cell((x, y))
        cell_value = self.grid_map.get_cell_value((map_x, map_y))
        # self.get_logger().info(f'Checking cell ({map_x}, {map_y}) with value {cell_value}')
        if cell_value is None:
            return False  # Out of bounds is considered invalid
        # if cell_value == 100:
        if cell_value > 0: # Assume positive value is obstacle
            # self.get_logger().info(f'Cell ({map_x}, {map_y}) is occupied with value {cell_value}')
            return False  # Occupied cell
        return True  # Free cell


def main(args=None):
    rclpy.init(args=args)
    node = RRTMotionPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()