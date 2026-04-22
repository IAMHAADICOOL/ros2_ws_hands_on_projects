#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import math
import tf_transformations
from cerveza_tb_interfaces.srv import GoTo

class ControlTB(Node):
    def __init__(self):
        super().__init__('control_tb')

        # Control gains - these can be tuned for better performance
        self.k_v = 0.4 # linear velocity (tests 0.5, 1.0)
        self.k_w = 0.8 # angular velocity (tests 1.0, 1.5) 

        # Robot state - initialized to zero until we receive odometry updates
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal position - initialized to None until we receive a goal pose
        self.x_g = None
        self.y_g = None

        # Subscribers (robot odometry and goal pose)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/path_goal_pose', self.goal_cb, 10)

        # Service (for receiving goal positions via a service call)
        self.create_service(GoTo, '/goto', self.goto_cb)

        # Publisher (for velocity commands)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control loop timer - runs every 0.1 seconds
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Control TB node started!')

    # Callback for odometry updates
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x   # Update x position from odometry
        self.y = msg.pose.pose.position.y   # Update y position from odometry
        q = msg.pose.pose.orientation       # Get orientation quaternion from odometry

        # Get theta (yaw) angle from quaternions
        _, _, self.theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Callback for receiving new goal positions from RViz (via PoseStamped messages)
    def goal_cb(self, msg):
        self.x_g = msg.pose.position.x  # Update goal x position from received message
        self.y_g = msg.pose.position.y  # Update goal y position from received message

        new_x = msg.pose.position.x
        new_y = msg.pose.position.y

        # Only log if goal actually changed
        if self.x_g is None or abs(new_x - self.x_g) > 0.01 or abs(new_y - self.y_g) > 0.01:
            self.get_logger().info(f'New goal: ({new_x:.2f}, {new_y:.2f})')
        
        self.x_g = new_x
        self.y_g = new_y

    # Callback for receiving new goal positions via service calls 
    def goto_cb(self, request, response):
        self.x_g = request.x  # Update goal x position from service request
        self.y_g = request.y  # Update goal y position from service request

        # Log the new goal position received via service call
        self.get_logger().info(f'Goal from service: ({self.x_g:.2f}, {self.y_g:.2f})', throttle_duration_sec=0.5)  # Throttle logging to once per second to avoid spamming

        response.success = True  # Indicate that the service call was successful
        return response

    # Control loop that runs periodically to compute and publish velocity commands
    def control_loop(self):

        # If no goal is set, do nothing
        if self.x_g is None:
            return

        # Position error - compute the difference between current position and goal position
        inc_x = self.x_g - self.x                   # Distance to goal in x direction
        inc_y = self.y_g - self.y                   # Distance to goal in y direction
        distance = math.sqrt(inc_x**2 + inc_y**2)   # Euclidean distance to the goal

        # Goal reached - if we are within 0.1 meters of the goal, stop the robot and clear the goal
        if distance < 0.1:
            if self.x_g is not None:  # Only log once
                self.get_logger().info('Goal reached!')
            msg = Twist()                           # Create a Twist message to stop the robot
            self.vel_pub.publish(msg)               # Publish zero velocity to stop the robot
            self.x_g = None                         # Clear the goal so we don't keep trying to reach it
            return
    
        # Compute the angle to the goal
        angle_to_goal = math.atan2(inc_y, inc_x)

        # Compute the angle error (difference between current orientation and angle to goal) and wrap it to [-pi, pi]
        angle_error = angle_to_goal - self.theta
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Angular velocity - proportional to the angle error
        w = self.k_w * angle_error

        # Linear velocity - only move forward if the angular velocity is small, otherwise stop to turn in place
        # if abs(w) < 0.2:
        #     v = self.k_v * math.sqrt(inc_x**2 + inc_y**2) # Proportional to the distance to the goal
        # else:
        #     v = 0.0

        # Smooth motion
        v = self.k_v * distance * max(0.0, 1.0 - abs(angle_error) / (math.pi/2))

        # Publish velocity
        msg = Twist()               # Create a Twist message to send velocity commands
        msg.linear.x = v            # Set linear velocity in the x direction
        msg.angular.z = w           # Set angular velocity around the z axis
        self.vel_pub.publish(msg)   # Publish the velocity command to the robot

# Entry point of the script
def main(args=None):
    rclpy.init(args=args)   # Initialize the ROS 2 Python client library
    node = ControlTB()      # Create an instance of the ControlTB node
    rclpy.spin(node)        # Keep the node running and processing callbacks until it is shut down
    node.destroy_node()     # Clean up the node resources
    rclpy.shutdown()        # Shut down the ROS 2 client library


if __name__ == '__main__':
    main()