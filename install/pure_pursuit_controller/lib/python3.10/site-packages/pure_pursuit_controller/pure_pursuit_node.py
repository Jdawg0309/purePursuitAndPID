#!/usr/bin/env python3
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # --- Load parameters ---
        # If you passed waypoint_file as a ROS2 param, use:
        wp_file = self.declare_parameter(
            'waypoint_file',
            os.path.join(
                get_package_share_directory('pure_pursuit_controller'),
                'waypoints',
                'waypoints.csv'
            )
        ).get_parameter_value().string_value

        self.lookahead_dist = self.declare_parameter('lookahead_dist', 2.0).value
        self.wheelbase      = self.declare_parameter('wheelbase', 0.5).value

        # --- Load waypoints CSV (skip header) ---
        self.waypoints = np.loadtxt(wp_file, delimiter=',', skiprows=1)
        self.current_wp_index = 0

        # --- Subscriptions & Publishers ---
        self.sub_odom   = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_cmd    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, 'lookahead_marker', 10)

    def odom_cb(self, msg: Odometry):
        # DEBUG: log incoming odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"ODOM ▶ x={x:.2f}, y={y:.2f}")

        # For now, publish a fixed forward speed
        cmd = Twist()
        cmd.linear.x  = 0.2  # constant 0.2 m/s
        cmd.angular.z = 0.0  # no turning yet
        self.pub_cmd.publish(cmd)

        self.get_logger().info(f"CMD ▶ linear.x={cmd.linear.x:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
