# lqr_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float64
from dip_interfaces.msg import DipState
import numpy as np
import control  # python-control


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class LQRController(Node):
    def __init__(self):
        super().__init__("lqr_controller_opt")

        # --- Parameters ---
        # plant params
        self._decl_double("m", 0.04556, "Mass [kg]")
        self._decl_double("g", 9.81, "Gravity [m/s^2]")
        self._decl_double("a", 0.0508, "Length a [m]")
        self._decl_double("b", 0.01, "Width b [m]")
        # LQR weights
        self._decl_int_array(
            "Q",
            [100, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 100, 0, 0, 0, 0, 10],
            "Q (4x4 row-major)",
        )
        self._decl_int_array("R", [1], "R (1x1)")
        # Torque constraints (optional but common in industry)
        self.declare_parameter(
            "torque_limit", 5_000.0
        )  # N·m (set realistically for your model)
        self.declare_parameter("torque_slew", 1e9)  # N·m/s; set high to start (no slew)

        self.torque_limit = float(self.get_parameter("torque_limit").value)
        self.torque_slew = float(self.get_parameter("torque_slew").value)

        # Build LQR
        self.K = self._compute_lqr_gain()
        if self.K is None:
            self.get_logger().error("Failed to compute LQR gain K.")
            raise RuntimeError("K is None")
        self.get_logger().info(f"LQR K: {self.K.tolist()}")

        # QoS (state in is sensor-like, best-effort; cmd out is reliable)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher and subscriber
        self.torque_pub = self.create_publisher(Float64, "torque_cmd", cmd_qos)
        self.create_subscription(DipState, "dip_state", self._on_state, sensor_qos)

        # Event-driven control: no wall-time timer
        self._last_u = 0.0
        self._last_t = self.get_clock().now()

        self.get_logger().info("LQR controller ready (event-driven on dip_state)")

    # ---- param helpers ----
    def _decl_double(self, name, default, desc):
        pd = ParameterDescriptor(
            name=name, type=ParameterType.PARAMETER_DOUBLE, description=desc
        )
        self.declare_parameter(name, default, pd)

    def _decl_int_array(self, name, default, desc):
        pd = ParameterDescriptor(
            name=name, type=ParameterType.PARAMETER_INTEGER_ARRAY, description=desc
        )
        self.declare_parameter(name, default, pd)

    # ---- build LQR ----
    def _compute_lqr_gain(self):
        m = float(self.get_parameter("m").value)
        g = float(self.get_parameter("g").value)
        a = float(self.get_parameter("a").value)
        b = float(self.get_parameter("b").value)
        d = a / 2.0

        # Inertia and linearized model
        M11 = 0.0147 * m * (2 * a * a + 2 * b * b + 144 * d * d)
        M12 = 2 * d * d * m
        M21 = 2 * d * d * m
        M22 = 0.0417 * m * (2 * a * a + 2 * b * b + 48 * d * d)
        M = np.array([[M11, M12], [M21, M22]])

        G1_q = -3 * d * g * m
        G2_q = -d * g * m
        dG_dq = np.array([[G1_q, 0.0], [0.0, G2_q]])
        dC_dqd = np.zeros((2, 2))

        A = np.block(
            [
                [np.zeros((2, 2)), np.eye(2)],
                [-np.linalg.inv(M) @ dG_dq, -np.linalg.inv(M) @ dC_dqd],
            ]
        )

        B = np.vstack([np.zeros((2, 1)), np.linalg.inv(M) @ np.array([[1.0], [0.0]])])

        Q = np.array(self.get_parameter("Q").value).reshape(4, 4)
        R = np.array(self.get_parameter("R").value).reshape(1, 1)

        K, *_ = control.lqr(A, B, Q, R)  # continuous-time LQR
        return K

    # ---- event-driven control ----
    def _on_state(self, msg: DipState):
        # Assemble state vector (make sure ordering matches your A/B)
        x = np.array([msg.theta1, msg.theta2, msg.theta1_dot, msg.theta2_dot])
        u = -self.K @ x

        # Optional: torque limiting and slew-rate limiting (common in real systems)
        # Slew limit needs a dt; use sim time
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0:
            dt = 0.0

        # Slew limit (|u - last| <= max_rate * dt)
        max_step = self.torque_slew * dt
        u = clamp(u, self._last_u - max_step, self._last_u + max_step)

        # Hard clamp
        u = clamp(u, -self.torque_limit, self.torque_limit)

        # Publish
        self.torque_pub.publish(Float64(data=float(u)))
        self._last_u = u

        # Throttled log (1 Hz)
        self.get_logger().info(
            f"u={u[0]:.3f}, state=[{', '.join(f'{v:.3f}' for v in x)}]",
        )


def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
