# lqr_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dip_interfaces.msg import DipState  # custom message
from std_msgs.msg import Float64
import numpy as np
import os
import control

from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class LQRController(Node):
    def __init__(self):
        super().__init__("lqr_controller")

        # --- Parameter Declarations ---
        # 1. Mass
        mass_descriptor = ParameterDescriptor(
            name="m",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Mass of the dip [kg]",
        )
        self.declare_parameter("m", 0.04556, mass_descriptor)

        # 2. Gravitational acceleration
        g_descriptor = ParameterDescriptor(
            name="g",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Gravitational acceleration [m/s^2]",
        )
        self.declare_parameter("g", 9.81, g_descriptor)

        # 3. Length and width of the dip
        a_descriptor = ParameterDescriptor(
            name="a",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Length of the dip [m]",
        )
        self.declare_parameter("a", 0.0508, a_descriptor)

        b_descriptor = ParameterDescriptor(
            name="b",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Width of the dip [m]",
        )
        self.declare_parameter("b", 0.01, b_descriptor)

        # 4. Cost matrices for LQR
        Q_descriptor = ParameterDescriptor(
            name="Q",
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description="State cost matrix Q (4x4 matrix)",
        )
        self.declare_parameter(
            "Q",
            [100, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 100, 0, 0, 0, 0, 10],
            Q_descriptor,
        )

        R_descriptor = ParameterDescriptor(
            name="R",
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description="Input cost matrix R (1x1 matrix)",
        )
        self.declare_parameter(
            "R",
            [1],
            R_descriptor,
        )

        # Load LQR gain K
        self.K = self._compute_lqr_gain()
        if self.K is None:
            self.get_logger().error("Failed to compute LQR gain K.")
            return
        self.get_logger().info(f"LQR gain K: {self.K.tolist()}")

        # k_path = os.path.join(os.path.dirname(__file__), "..", "config", "lqr_gain.csv")
        # self.K = np.loadtxt(os.path.abspath(k_path), delimiter=",")

        # Initialize state
        self.state = np.zeros(4)

        # Subscribe to joint_states
        self.create_subscription(DipState, "dip_state", self.dip_state_callback, 10)

        # Publisher for torque command
        self.torque_pub = self.create_publisher(Float64, "torque_cmd", 10)

        # Timer to run control loop
        self.timer = self.create_timer(0.01, self.control_callback)

    def _compute_lqr_gain(self):
        # Load system parameters
        m = self.get_parameter("m").get_parameter_value().double_value
        g = self.get_parameter("g").get_parameter_value().double_value
        a = self.get_parameter("a").get_parameter_value().double_value
        b = self.get_parameter("b").get_parameter_value().double_value
        d = a / 2.0

        # Moments of inertia
        J = (1 / 12) * m * (a**2 + b**2) + m * d**2

        M11 = 0.0147 * m * (2 * a**2 + 2 * b**2 + 144 * d**2)
        M12 = 2 * d**2 * m
        M21 = 2 * d**2 * m
        M22 = 0.0417 * m * (2 * a**2 + 2 * b**2 + 48 * d**2)
        M = np.array([[M11, M12], [M21, M22]])

        G1_q = -3 * d * g * m
        G2_q = -d * g * m
        dG_dq = np.array([[G1_q, 0], [0, G2_q]])
        dC_dqd = np.zeros((2, 2))  # Linearized at zero velocity

        A = np.block(
            [
                [np.zeros((2, 2)), np.eye(2)],
                [-np.linalg.inv(M) @ dG_dq, -np.linalg.inv(M) @ dC_dqd],
            ]
        )
        self.get_logger().info(f"A matrix:\n{A}")

        B = np.vstack([np.zeros((2, 1)), np.linalg.inv(M) @ np.array([[1], [0]])])
        print("B matrix:", B)

        # Load LQR weights
        Q = np.array(
            self.get_parameter("Q").get_parameter_value().integer_array_value
        ).reshape(4, 4)
        R = np.array(
            self.get_parameter("R").get_parameter_value().integer_array_value
        ).reshape(1, 1)

        # Solve Continuous Algebraic Riccati Equation and get K
        from scipy.linalg import solve_continuous_are

        K, S, E = control.lqr(A, B, Q, R)

        self.get_logger().info(f"LQR gain K:\n{K}")

        return K

    def dip_state_callback(self, msg):
        # Make sure names match your URDF joint names!
        self.state = np.array(
            [
                msg.theta1,
                msg.theta2,
                msg.theta1_dot,
                msg.theta2_dot,
            ]
        )

    def control_callback(self):
        u = -self.K @ self.state
        self.torque_pub.publish(Float64(data=float(u)))
        self.get_logger().info(f"u = {u[0]:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
