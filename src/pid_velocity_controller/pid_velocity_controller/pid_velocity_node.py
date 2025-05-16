import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class PIDVelocityNode(Node):
    def __init__(self):
        super().__init__('pid_velocity_controller')
        # init gains, state
        self.sub = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub = self.create_publisher(Float32, '/speed_cmd', 10)

    def cb_odom(self, msg: Odometry):
        # compute v_cur, error, u = Kp e + Ki ∫e + Kd de/dt
        # publish Float32(speed_cmd)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()