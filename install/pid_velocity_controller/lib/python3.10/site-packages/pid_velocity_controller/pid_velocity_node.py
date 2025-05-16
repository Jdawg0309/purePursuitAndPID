#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class PIDVelocityNode(Node):
    def __init__(self):
        super().__init__('pid_velocity_controller')
        # PID gains
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1

        # Desired speed (m/s)
        self.target_speed = 0.5

        # State for integral & derivative terms
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

        # Subscription and publisher
        self.sub = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub = self.create_publisher(Float32, '/speed_cmd', 10)

    def cb_odom(self, msg: Odometry):
        now = self.get_clock().now()
        v_cur = msg.twist.twist.linear.x

        # First callback: just initialize timing and error
        if self.last_time is None:
            self.last_time = now
            self.last_error = self.target_speed - v_cur
            return

        # Compute time step and error
        dt = (now - self.last_time).nanoseconds * 1e-9
        error = self.target_speed - v_cur

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Form and publish speed command
        cmd = Float32()
        # Clamp the command to [0, target_speed * 2] for safety
        cmd.data = float(max(0.0, min(self.target_speed + u, self.target_speed * 2.0)))
        self.pub.publish(cmd)

        # Save state for next iteration
        self.last_error = error
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
