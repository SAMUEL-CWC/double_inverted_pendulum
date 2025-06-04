# lqr_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # Load LQR gain K
        k_path = os.path.join(
            get_package_share_directory('dip_control'),
            'config',
            'lqr_gain.csv'
        )
        self.K = np.loadtxt(os.path.abspath(k_path), delimiter=',')

        # Initialize state
        self.state = np.zeros(4)

        # Subscribe to joint_states
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Publisher for torque command
        self.torque_pub = self.create_publisher(Float64, 'torque_cmd', 10)

        # Timer to run control loop
        self.timer = self.create_timer(0.01, self.control_callback)

    def joint_state_callback(self, msg):
        # Make sure names match your URDF joint names!
        try:
            idx1 = msg.name.index('joint1')
            idx2 = msg.name.index('joint2')
            theta1 = msg.position[idx1]
            theta2 = msg.position[idx2]
            theta1_dot = msg.velocity[idx1]
            theta2_dot = msg.velocity[idx2]
            self.state = np.array([theta1, theta2, theta1_dot, theta2_dot])
        except ValueError:
            self.get_logger().warn('Joint names not found in joint_states.')

    def control_callback(self):
        u = -self.K @ self.state
        self.torque_pub.publish(Float64(data=u))
        self.get_logger().info(f"u = {u:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
