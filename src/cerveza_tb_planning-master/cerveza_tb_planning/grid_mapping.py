#!/usr/bin/env python3

# Import required libraries
import rclpy
from rclpy.node import Node                         # For creating nodes
from nav_msgs.msg import Odometry, OccupancyGrid    # For receiving robot position and publish map
from geometry_msgs.msg import TransformStamped        # For TF broadcasting
from sensor_msgs.msg import LaserScan               # For receiving lidar data
import numpy as np
import math
import tf_transformations                           # For quaternions conversion

from cerveza_tb_planning.grid_map import GridMap, map_to_msg    # For internal map representation and updates


from tf2_ros import TransformBroadcaster


class GridMappingNode(Node):
    def __init__(self):
        super().__init__('grid_mapping')

        # Robot state and flags
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False      # Flag for not processing scans without odom   
        self.map_initialized = False    # Flag for waiting first odom to create the map

        self.odom_buffer = []           # Store odom with timestamp (for synchronization)
        self.gridmap = None             # Grid map will be created when first odom is recieved
        self.map_tf = None              # Store map transform for TF broadcasting

        self.last_mapping_x = None
        self.last_mapping_y = None
        self.last_mapping_yaw = None
        self.min_move_distance = 0.1

        self.min_rotation_diff = 0.05

        # Declare lidar frame name
        self.is_sim = self.declare_parameter('is_sim', False).get_parameter_value().bool_value

        if self.is_sim:
            self.get_logger().info('Running in SIMULATION mode')
            self.lidar_offset = np.pi
            self.lidar_rot_direction = -1
        else:
            self.get_logger().info('Running in REAL ROBOT mode')
            self.lidar_offset = np.pi/2
            self.lidar_rot_direction = 1


        # Subscribers
        self.create_subscription(Odometry, '/turtlebot/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/turtlebot/scan', self.scan_cb, 10)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)

        # TF broadcaster for map as a child of world_enu
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish map (every 1 second)
        self.create_timer(0.1, self.publish_map)

        self.get_logger().info('Grid Mapping node started!')

    def odom_cb(self, msg):
        # Update robot's position and orientation (get only the yaw from quaternions)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_received = True   # Update odom flag

        # Initialize map centered on robot's first position (cell size 0.01m and map size 5m)
        if not self.map_initialized:
            self.gridmap = GridMap(np.array([self.x, self.y]), 0.05, 20)
            self.map_initialized = True
            self.get_logger().info(f'Map centered at ({self.x:.2f}, {self.y:.2f})')
            self.map_tf = TransformStamped()
            self.map_tf.header.frame_id = 'odom'
            self.map_tf.child_frame_id = 'map'
            self.map_tf.transform.translation.x = 0.0
            self.map_tf.transform.translation.y = 0.0
            self.map_tf.transform.translation.z = 0.0
            # No rotation between world_enu and map frames (map is axis-aligned with world)
            self.map_tf.transform.rotation.x = 0.0
            self.map_tf.transform.rotation.y = 0.0
            self.map_tf.transform.rotation.z = 0.0
            self.map_tf.transform.rotation.w = 1.0
           
        
        # Store odom + timestamp in buffer for synchronization
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_buffer.append((stamp, self.x, self.y, self.theta))
        # Keep only last 200 entries (enough for multiple seconds history)
        if len(self.odom_buffer) > 200:
            self.odom_buffer.pop(0)

    # Given a scan timestamp, get the closest odom timestamp
    def get_closest_odom(self, stamp):
        if not self.odom_buffer:
            return None

        min_diff = float('inf')
        closest = None

        # Search for the closest odom
        for odom in self.odom_buffer:
            diff = abs(odom[0] - stamp)
            if diff < min_diff:
                min_diff = diff
                closest = odom
        return closest

    def scan_cb(self, msg):
        if not self.map_initialized:
            return
        
        if not self.odom_received:
            return
        
        # Get scan timestamp and find the closest odom
        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom = self.get_closest_odom(scan_stamp)
        if odom is None:
            return
        
        # Transform lidar offset from laser frame to base_footprint frame (assuming it's a static transform)
        # if self.laser_transform is not None:

        # Robot position synchronized with the scan measurement
        _, robot_x, robot_y, robot_theta = odom


        # Always add ray at first time, then only when robot moves
        if self.last_mapping_x is not None:
            dist = math.sqrt((self.x - self.last_mapping_x)**2 + (self.y - self.last_mapping_y)**2)
            rotation_diff = abs(self.theta - self.last_mapping_yaw)
            if dist < self.min_move_distance and rotation_diff < self.min_rotation_diff:
                return

        # Add the ray to the map with 0.9 probability in end point 
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:   # Process only valid range
                # Ray angle = Robot theta + lidar offset + beam angle
                beam_angle = (msg.angle_min + i * msg.angle_increment * self.lidar_rot_direction) + self.lidar_offset
                if beam_angle < -math.pi:
                    beam_angle += 2 * math.pi
                elif beam_angle > math.pi:
                    beam_angle -= 2 * math.pi

                ray_angle = beam_angle + robot_theta

                self.gridmap.add_ray([self.x, self.y], ray_angle, r, 0.9)

        self.last_mapping_x = self.x
        self.last_mapping_y = self.y
        self.last_mapping_yaw = self.theta

    def publish_map(self):
        if not self.map_initialized:
            return
    
        # Get grid data
        grid = self.gridmap.get_map()
        origin = self.gridmap.get_origin()

        # Convert log-odds to occupancy values [0-100], -1 for unknown 
        occupancy = np.full(grid.shape, -1, dtype=np.int8) # unknown (gray)
        occupancy[grid > 0.5] = 100   # occupied (black)
        occupancy[grid < -0.5] = 0    # free (white)

        msg = map_to_msg(self.gridmap)
        msg.info.map_load_time = self.get_clock().now().to_msg()  # Set map load time to current time
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Map frame for the occupancy grid message

        # Publish the map
        self.map_pub.publish(msg)

        # Publish the inflated map
        inflated_msg = map_to_msg(self.gridmap, inflated=True, inflation_radius=0.25)
        inflated_msg.header = msg.header  # Keep same header for consistency
        inflated_msg.info.map_load_time = msg.info.map_load_time
        self.inflated_map_pub.publish(inflated_msg)

        # Publish the map TF (map -> world_enu)
        if self.map_tf is not None:
            self.map_tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.map_tf)
        else:
            self.get_logger().warn('Map TF not available yet!')

def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()