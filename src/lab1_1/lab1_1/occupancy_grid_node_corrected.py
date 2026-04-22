#!/usr/bin/env python3
"""
Occupancy Grid Mapping Node
Subscribes to /odom and /scan, builds probabilistic occupancy grid,
publishes nav_msgs/OccupancyGrid for visualization in RViz.
Supports both live ROS topics and offline CSV file playback.
"""

import math

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class GridMap:
    """Probabilistic occupancy grid map using log-odds representation."""

    LMAX = 6.91
    LMIN = -6.91

    def __init__(self, center, cell_size=0.1, map_size=20):
        self.cell_size = cell_size
        self.grid = np.zeros((int(map_size / cell_size), int(map_size / cell_size)))
        self.origin = np.array(center) - np.array([map_size, map_size]) / 2
        self.height, self.width = self.grid.shape

    def position_to_cell(self, position):
        x_pos, y_pos = position
        x_cell = int(np.floor((x_pos - self.origin[0]) / self.cell_size))
        y_cell = int(np.floor((y_pos - self.origin[1]) / self.cell_size))
        return (x_cell, y_cell)

    def update_cell(self, uv, p):
        if p <= 0.0 or p >= 1.0:
            return

        x_cell, y_cell = uv
        if x_cell < 0 or x_cell >= self.width or y_cell < 0 or y_cell >= self.height:
            return

        l = np.log(p / (1.0 - p))
        self.grid[y_cell, x_cell] += l
        self.grid[y_cell, x_cell] = np.clip(self.grid[y_cell, x_cell], self.LMIN, self.LMAX)

    def add_ray(self, ray_init_position, ray_angle, ray_range, p_occ, mark_occupied=True):
        x_init, y_init = ray_init_position
        x_final = x_init + ray_range * np.cos(ray_angle)
        y_final = y_init + ray_range * np.sin(ray_angle)

        x_init_cell, y_init_cell = self.position_to_cell(ray_init_position)
        x_final_cell, y_final_cell = self.position_to_cell((x_final, y_final))

        points = list(self.bresenham(x_init_cell, y_init_cell, x_final_cell, y_final_cell))
        if len(points) == 0:
            return

        for pt in points[:-1]:
            self.update_cell(pt, 1.0 - p_occ)

        if mark_occupied:
            self.update_cell(points[-1], p_occ)
        else:
            self.update_cell(points[-1], 1 - p_occ)

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return points

    def log_odds_to_probability(self, log_odds):
        prob = 1.0 - (1.0 / (1.0 + np.exp(log_odds)))
        return int(np.clip(prob * 100, 0, 100))

    def get_occupancy_grid_array(self):
        grid_prob = np.zeros_like(self.grid)
        for i in range(self.height):
            for j in range(self.width):
                grid_prob[i, j] = self.log_odds_to_probability(self.grid[i, j])
        return grid_prob.astype(np.int8)

    def get_origin(self):
        return self.origin


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__("occupancy_grid_node")

        self.declare_parameter("grid_size", 20.0)
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("laser_frame", "rplidar")
        self.declare_parameter("p_occ", 0.9)
        self.declare_parameter("clear_on_max_range", False)

        grid_size = self.get_parameter("grid_size").value
        self.grid_resolution = self.get_parameter("grid_resolution").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.p_occ = self.get_parameter("p_occ").value
        self.clear_on_max_range = self.get_parameter("clear_on_max_range").value

        self.grid_map = GridMap(center=[0.0, 0.0], cell_size=self.grid_resolution, map_size=grid_size)
        self.grid_size = grid_size

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.latest_scan = None
        self.scan_received = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(Odometry, "/turtlebot/odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/turtlebot/scan", self.scan_callback, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            f"Occupancy Grid Node Started. Grid: {grid_size}x{grid_size}m @ {self.grid_resolution}m/cell"
        )
        self.get_logger().info(
            f"Frame configuration: map_frame='{self.map_frame}', base_frame='{self.base_frame}', laser_frame='{self.laser_frame}'"
        )
        self.get_logger().info(f"Max-range policy: clear_on_max_range={self.clear_on_max_range}")

        self.odom_buffer = []

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_theta = self.quaternion_to_yaw(qx, qy, qz, qw)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_buffer.append((stamp, self.robot_x, self.robot_y, self.robot_theta))
        if len(self.odom_buffer) > 200:
            self.odom_buffer.pop(0)

    def get_closest_odom(self, stamp):
        if not self.odom_buffer:
            return None

        min_diff = float("inf")
        closest = None
        for odom in self.odom_buffer:
            diff = abs(odom[0] - stamp)
            if diff < min_diff:
                min_diff = diff
                closest = odom
        return closest

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received = True
        self.get_logger().debug(f"Scan received: {len(msg.ranges)} ranges from frame '{msg.header.frame_id}'")

    def timer_callback(self):
        if self.scan_received and self.latest_scan is not None:
            self.scan_received = False
            self.update_grid_from_scan(self.latest_scan)

            grid_min = np.min(self.grid_map.grid)
            grid_max = np.max(self.grid_map.grid)
            grid_mean = np.mean(self.grid_map.grid)
            self.get_logger().info(
                f"Grid updated. Stats - Min: {grid_min:.2f}, Max: {grid_max:.2f}, Mean: {grid_mean:.2f}"
            )

        self.publish_occupancy_grid()

    def update_grid_from_scan(self, scan_msg):
        try:
            scan_time = rclpy.time.Time.from_msg(scan_msg.header.stamp)
            self.get_logger().debug(
                f"Looking up transform from '{self.laser_frame}' to '{self.map_frame}' at time {scan_time.nanoseconds}"
            )

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.laser_frame,
                    scan_time,
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except tf2_ros.TransformException:
                self.get_logger().debug("Exact timestamp lookup failed, using latest available transform")
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.laser_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )

            scan_stamp = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
            odom = self.get_closest_odom(scan_stamp)
            if odom is None:
                return

            _, robot_x, robot_y, robot_theta = odom
            robot_x_map = transform.transform.translation.x
            robot_y_map = transform.transform.translation.y

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            laser_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

            self.get_logger().debug(f"Laser at ({robot_x_map:.2f}, {robot_y_map:.2f}), yaw={laser_yaw:.2f}")

            min_range = scan_msg.range_min
            max_range = scan_msg.range_max

            ranges = np.array(scan_msg.ranges)
            angles_laser = np.arange(
                scan_msg.angle_min,
                scan_msg.angle_max + scan_msg.angle_increment,
                scan_msg.angle_increment,
            )

            valid_rays = 0
            for i, (range_val, angle_laser) in enumerate(zip(ranges, angles_laser)):
                is_max_range = not np.isfinite(range_val) or range_val >= max_range

                # Preserve mapped obstacles by default when there is no hit.
                if is_max_range and not self.clear_on_max_range:
                    continue

                if is_max_range:
                    effective_range = scan_msg.range_max
                else:
                    effective_range = range_val

                if effective_range < min_range:
                    continue

                beam_angle = robot_theta + angle_laser + math.pi / 2.0

                self.grid_map.add_ray(
                    (robot_x_map, robot_y_map),
                    beam_angle,
                    effective_range,
                    self.p_occ,
                    mark_occupied=not is_max_range,
                )
                valid_rays += 1

            self.get_logger().info(f"Processed {valid_rays}/{len(ranges)} valid rays")

        except tf2_ros.TransformException as ex:
            self.get_logger().error(
                f"CRITICAL: Transform lookup failed. Frames: '{self.laser_frame}' -> '{self.map_frame}'. Error: {ex}"
            )
        except Exception as ex:
            self.get_logger().error(f"Error during grid update: {type(ex).__name__}: {ex}")

    def publish_occupancy_grid(self):
        grid_data = self.grid_map.get_occupancy_grid_array()

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.grid_map.cell_size
        msg.info.width = self.grid_map.width
        msg.info.height = self.grid_map.height

        origin = self.grid_map.get_origin()
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid_data.flatten().tolist()

        occupied_count = sum(1 for v in msg.data if v > 50)
        free_count = sum(1 for v in msg.data if 0 <= v <= 50)
        unknown_count = sum(1 for v in msg.data if v == -1)

        self.get_logger().debug(
            f"Publishing map: frame_id='{msg.header.frame_id}', size={msg.info.width}x{msg.info.height}, "
            f"occupied={occupied_count}, free={free_count}, unknown={unknown_count}"
        )

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
