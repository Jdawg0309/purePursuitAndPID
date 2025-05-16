#!/usr/bin/env python3
import os, math, numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # --- Parameters ---
        pkg_share = get_package_share_directory('pure_pursuit_controller')
        default_wp = os.path.join(pkg_share, 'waypoints', 'waypoints.csv')
        wp_file = self.declare_parameter('waypoint_file', default_wp).get_parameter_value().string_value
        self.lookahead_dist = self.declare_parameter('lookahead_dist', 2.0).value
        self.wheelbase      = self.declare_parameter('wheelbase', 0.5).value

        # --- Load waypoints ---
        self.waypoints = np.loadtxt(wp_file, delimiter=',', skiprows=1)
        self.current_wp_index = 0

        # --- State from PID speed controller ---
        self.speed_cmd = 0.0
        self.sub_speed = self.create_subscription(
            Float32, '/speed_cmd', self.speed_callback, 10
        )

        # --- ROS pubs/subs ---
        self.sub_odom   = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_cmd    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, 'lookahead_marker', 10)

    def speed_callback(self, msg: Float32):
        self.speed_cmd = msg.data

    def odom_cb(self, msg: Odometry):
        # 1) extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2) find lookahead waypoint
        while self.current_wp_index < len(self.waypoints):
            wx, wy, _ = self.waypoints[self.current_wp_index]
            if math.hypot(wx - x, wy - y) >= self.lookahead_dist:
                break
            self.current_wp_index += 1

        if self.current_wp_index >= len(self.waypoints):
            # finished path → stop
            self.pub_cmd.publish(Twist())
            return

        # 3) compute curvature k = 2*sin(alpha)/Ld
        wx, wy, _ = self.waypoints[self.current_wp_index]
        angle_to_wp = math.atan2(wy - y, wx - x)
        alpha = math.atan2(math.sin(angle_to_wp - yaw), math.cos(angle_to_wp - yaw))
        k = 2.0 * math.sin(alpha) / self.lookahead_dist

        # 4) compute commands
        v = self.speed_cmd
        omega = k * v

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)

        # 5) publish RViz marker at (wx, wy)
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = wx
        marker.pose.position.y = wy
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
