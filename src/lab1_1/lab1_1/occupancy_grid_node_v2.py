#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import Buffer, TransformListener, TransformException




class GridMap:
    LMAX, LMIN = 6.91, -6.91 # Log-odds saturation
    
    def __init__(self, center, cell_size=0.1, map_size=20):
        self.cell_size = cell_size
        self.grid = np.zeros((int(map_size / cell_size), int(map_size / cell_size)))
        self.origin = np.array(center) - np.array([map_size, map_size]) / 2
        self.height, self.width = self.grid.shape
    
    def position_to_cell(self, position):
        x_cell = int(np.floor((position[0] - self.origin[0]) / self.cell_size))
        y_cell = int(np.floor((position[1] - self.origin[1]) / self.cell_size))
        return (x_cell, y_cell)
    
    def update_cell(self, uv, p):
        if p <= 0.0 or p >= 1.0: return
        x_cell, y_cell = uv
        if 0 <= x_cell < self.width and 0 <= y_cell < self.height:
            l = np.log(p / (1.0 - p))
            self.grid[y_cell, x_cell] = np.clip(self.grid[y_cell, x_cell] + l, self.LMIN, self.LMAX)

    def add_ray(self, start_pos, ray_angle, ray_range, p_occ, p_free, mark_occupied=True):
        x_final = start_pos[0] + ray_range * np.cos(ray_angle)
        y_final = start_pos[1] + ray_range * np.sin(ray_angle)
        
        c0 = self.position_to_cell(start_pos)
        c1 = self.position_to_cell((x_final, y_final))
        
        points = self.bresenham(c0[0], c0[1], c1[0], c1[1])
        if not points: return
        
        # Mark free space using the WEAKER probability (so it doesn't erase walls instantly)
        for pt in points[:-1]:
            self.update_cell(pt, p_free)
        
        # Mark endpoint using the STRONGER probability
        if mark_occupied:
            self.update_cell(points[-1], p_occ)
        else:
            self.update_cell(points[-1], p_free)

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        points = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return points

    def get_occupancy_grid_array(self):
        # prob = 1 - 1/(1 + exp(l))
        prob = 100 - (100 / (1 + np.exp(self.grid)))
        prob[np.abs(self.grid) < 0.1] = -1 # Unknown
        return prob.astype(np.int8)

class OccupancyGridNodeV2(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node_v2')
        
        # Parameters
        self.declare_parameter('grid_size', 30.0)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('map_frame', 'world_enu')
        
        self.map_frame = self.get_parameter('map_frame').value
        res = self.get_parameter('grid_resolution').value
        size = self.get_parameter('grid_size').value
        
        self.grid_map = GridMap(center=[0.0, 0.0], cell_size=res, map_size=size)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_sub = Subscriber(self, LaserScan, '/turtlebot/scan')
        self.odom_sub = Subscriber(self, Odometry, '/turtlebot/odom')
        self.sync = ApproximateTimeSynchronizer([self.odom_sub, self.scan_sub], 10, 0.1)
        self.sync.registerCallback(self.synchronized_callback)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info(f"Mapping started. Target frame: {self.map_frame}")

    def synchronized_callback(self, odom_msg, scan_msg):
        try:
            # We look up the transform from global map to the LIDAR sensor frame
            # This handles all rotations shown in your screenshots
            t = self.tf_buffer.lookup_transform(
                self.map_frame, 
                scan_msg.header.frame_id, 
                rclpy.time.Time(), # Get latest to avoid extrapolation errors
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            
            # Position of the sensor
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # Calculate direction of the LIDAR's X-axis (forward)
            dir_x = 1.0 - 2.0 * (q.y**2 + q.z**2)
            dir_y = 2.0 * (q.x * q.y + q.w * q.z)

            # Calculate direction of the LIDAR's Y-axis (left)
            # This helps us check if the coordinate system is right-handed
            side_x = 2.0 * (q.x * q.y - q.w * q.z)
            side_y = 1.0 - 2.0 * (q.x**2 + q.z**2)

            # sensor_yaw = math.atan2(dir_y, dir_x)

            sensor_yaw = self.get_yaw_from_quaternion(q.x, q.y, q.z, q.w)

            self.process_scan(scan_msg, x, y, sensor_yaw)
            
        except TransformException as e:
            self.get_logger().warn(f"TF Wait: {e}")

    def get_yaw_from_quaternion(self, qx, qy, qz, qw):
        """
        Extracts the yaw (rotation around Z) from a quaternion.
        Standard ROS/ENU convention.
        """
        # 1. Calculate the numerator (sin of yaw * cos of pitch)
        # This represents the Y-component of the rotation
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        
        # 2. Calculate the denominator (cos of yaw * cos of pitch)
        # This represents the X-component of the rotation
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        
        # 3. Use atan2 to handle all quadrants and return angle in radians
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def process_scan(self, scan, x, y, theta):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Weights for persistence
        P_OCC_HIT = 0.90
        P_FREE_MISS = 0.45
        
        for r, a in zip(ranges, angles):
            is_hit = np.isfinite(r) and r < scan.range_max
            eff_r = r if is_hit else scan.range_max
            if eff_r < scan.range_min: continue
            
            # CHANGE: Try subtracting the angle if adding it causes a flip
            # Or, if theta is global ENU, and your scan is mirrored:
            beam_angle = theta - a  # <--- Change '+' to '-' here
            
            self.grid_map.add_ray((x, y), beam_angle, eff_r, P_OCC_HIT, P_FREE_MISS, is_hit)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.grid_map.cell_size
        msg.info.width = self.grid_map.width
        msg.info.height = self.grid_map.height
        msg.info.origin.position.x = self.grid_map.origin[0]
        msg.info.origin.position.y = self.grid_map.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid_map.get_occupancy_grid_array().flatten().tolist()
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = OccupancyGridNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()