#!/usr/bin/env python3
"""
ROS2 diagnostic node to investigate image topic message drops.
Subscribes to multiple image topics and tracks delivery statistics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
import time
from collections import deque
from datetime import datetime


class ImageDropDiagnostic(Node):
    def __init__(self):
        super().__init__('image_drop_diagnostic')
        
        # Statistics tracking
        self.stats = {
            'color_raw': {
                'received': 0,
                'timestamps': deque(maxlen=100),
                'frame_ids': deque(maxlen=100),
                'last_seq': None,
                'drops': 0,
                'last_time': None,
            },
            'color_compressed': {
                'received': 0,
                'timestamps': deque(maxlen=100),
                'frame_ids': deque(maxlen=100),
                'last_seq': None,
                'drops': 0,
                'last_time': None,
            },
            'depth_rect': {
                'received': 0,
                'timestamps': deque(maxlen=100),
                'frame_ids': deque(maxlen=100),
                'last_seq': None,
                'drops': 0,
                'last_time': None,
            },
        }
        
        # Create QoS profiles with different settings to test
        # Sensor data QoS (typically used by cameras)
        self.qos_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Best effort with larger queue
        self.qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Reliable with larger queue
        self.qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.get_logger().info('=== Image Drop Diagnostic Node ===')
        self.get_logger().info(f'Starting subscriptions with QoS settings:')
        self.get_logger().info(f'  - Sensor Data: BEST_EFFORT, depth=5')
        self.get_logger().info(f'  - Best Effort: depth=10')
        self.get_logger().info(f'  - Reliable: depth=10')
        
        # Subscribe to color raw image (try sensor data QoS first)
        self.sub_color_raw = self.create_subscription(
            Image,
            '/turtlebot/camera/color/image_raw',
            self.callback_color_raw,
            self.qos_sensor_data
        )
        self.get_logger().info('✓ Subscribed to /turtlebot/camera/color/image_raw (SENSOR_DATA)')
        
        # Subscribe to color compressed image (try sensor data QoS first)
        self.sub_color_compressed = self.create_subscription(
            CompressedImage,
            '/turtlebot/camera/color/image_compressed',
            self.callback_color_compressed,
            self.qos_sensor_data
        )
        self.get_logger().info('✓ Subscribed to /turtlebot/camera/color/image_compressed (SENSOR_DATA)')
        
        # Subscribe to depth image (try sensor data QoS first)
        self.sub_depth = self.create_subscription(
            Image,
            '/turtlebot/camera/depth/image_rect_raw',
            self.callback_depth,
            self.qos_sensor_data
        )
        self.get_logger().info('✓ Subscribed to /turtlebot/camera/depth/image_rect_raw (SENSOR_DATA)')
        
        # Check topic info
        self.get_logger().info('\n--- Topic Information ---')
        self.check_topic_info()
        
        # Timer for periodic statistics reporting
        self.timer = self.create_timer(2.0, self.report_stats)
    
    def check_topic_info(self):
        """Check if topics exist and get their info."""
        topics = {
            'color_raw': '/turtlebot/camera/color/image_raw',
            'color_compressed': '/turtlebot/camera/color/image_compressed',
            'depth': '/turtlebot/camera/depth/image_rect_raw',
        }
        
        topic_names = self.get_topic_names_and_types()
        available_topics = {name: types for name, types in topic_names}
        
        for key, topic in topics.items():
            if topic in available_topics:
                self.get_logger().info(f'✓ Topic exists: {topic}')
                self.get_logger().info(f'  Types: {available_topics[topic]}')
            else:
                self.get_logger().warn(f'✗ Topic NOT found: {topic}')
        
    def process_header(self, header: Header, topic_key: str):
        """Extract sequence number from header."""
        try:
            # Try to extract sequence number from frame_id or use timestamp
            if hasattr(header, 'seq'):
                seq = header.seq
            else:
                # Use nanoseconds as pseudo-sequence
                seq = header.stamp.sec * 1e9 + header.stamp.nanosec
            
            current_time = time.time()
            stats = self.stats[topic_key]
            
            # Store timing data
            stats['timestamps'].append(current_time)
            stats['frame_ids'].append(header.frame_id)
            stats['received'] += 1
            
            # Check for drops (if sequence number exists)
            if stats['last_seq'] is not None and seq != stats['last_seq'] + 1:
                drop_count = seq - stats['last_seq'] - 1
                stats['drops'] += drop_count
            
            stats['last_seq'] = seq
            stats['last_time'] = current_time
            
        except Exception as e:
            self.get_logger().warn(f'Error processing header for {topic_key}: {e}')
    
    def callback_color_raw(self, msg: Image):
        """Callback for color raw image."""
        self.process_header(msg.header, 'color_raw')
    
    def callback_color_compressed(self, msg: CompressedImage):
        """Callback for color compressed image."""
        self.process_header(msg.header, 'color_compressed')
    
    def callback_depth(self, msg: Image):
        """Callback for depth image."""
        self.process_header(msg.header, 'depth_rect')
    
    def calculate_fps(self, timestamps):
        """Calculate FPS from timestamp deque."""
        if len(timestamps) < 2:
            return 0.0
        
        time_diff = timestamps[-1] - timestamps[0]
        if time_diff == 0:
            return 0.0
        
        fps = (len(timestamps) - 1) / time_diff
        return fps
    
    def report_stats(self):
        """Report statistics for all topics."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'[{datetime.now().strftime("%H:%M:%S")}] IMAGE DROP DIAGNOSTICS')
        self.get_logger().info('='*60)
        
        for topic_key, stats in self.stats.items():
            if stats['received'] == 0:
                self.get_logger().info(f'\n{topic_key.upper()}: No messages received')
                continue
            
            fps = self.calculate_fps(stats['timestamps'])
            drop_rate = (stats['drops'] / (stats['received'] + stats['drops']) * 100) if (stats['received'] + stats['drops']) > 0 else 0
            
            self.get_logger().info(f'\n{topic_key.upper()}:')
            self.get_logger().info(f'  Messages received: {stats["received"]}')
            self.get_logger().info(f'  Estimated FPS: {fps:.2f}')
            self.get_logger().info(f'  Detected drops: {stats["drops"]}')
            self.get_logger().info(f'  Drop rate: {drop_rate:.2f}%')
            
            if len(stats['timestamps']) >= 2:
                avg_interval = (stats['timestamps'][-1] - stats['timestamps'][0]) / (len(stats['timestamps']) - 1)
                self.get_logger().info(f'  Avg interval: {avg_interval*1000:.2f}ms')
        
        self.get_logger().info('\n' + '='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = ImageDropDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
