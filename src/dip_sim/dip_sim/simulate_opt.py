# simulate.py
import rclpy
from rclpy.node import Node
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dip_interfaces.msg import DipState
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock

import pybullet as p
import pybullet_data
import math, os, time, threading
from ament_index_python.packages import get_package_share_directory


class DipSimulator(Node):
    def __init__(self):
        super().__init__("dip_simulator_opt")

        # --- Parameters ---
        self.declare_parameter("physics_dt", 0.001)  # 1 kHz physics
        self.declare_parameter("state_pub_hz", 250.0)  # decimate to 250 Hz ROS I/O
        self.declare_parameter("gui", True)
        self.declare_parameter("angle_mode", "absolute")  # "absolute" or "relative"
        self.declare_parameter("theta1_offset", 0.0)  # radians
        self.declare_parameter("theta2_offset", 0.0)  # radians

        self.physics_dt = self.get_parameter("physics_dt").value
        self.state_pub_hz = self.get_parameter("state_pub_hz").value
        self.gui = self.get_parameter("gui").value

        self.angle_mode = self.get_parameter("angle_mode").value
        self.theta1_offset = self.get_parameter("theta1_offset").value
        self.theta2_offset = self.get_parameter("theta2_offset").value

        self.add_on_set_parameters_callback(self._on_params)

        # --- QoS (typical in robotics) ---
        # State out: best-effort, small history (sensor-like)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Command in: reliable so we don't drop torque commands
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.state_pub = self.create_publisher(DipState, "dip_state", sensor_qos)

        # Subscriber (torque command)
        self.subscription = self.create_subscription(
            Float64, "torque_cmd", self.torque_callback, cmd_qos
        )

        # Shared torque (protected by lock)
        self._torque_lock = threading.Lock()
        self._torque = 0.0

        # Init PyBullet
        self._sim_init()

        # Timing helpers
        self._sim_time = 0.0
        self._publish_interval = 1.0 / self.state_pub_hz
        self._time_since_pub = 0.0

        # Start dedicated physics thread (decoupled from ROS timers)
        self._stop_evt = threading.Event()
        self._physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._physics_thread.start()

        self.get_logger().info(
            f"Physics dt={self.physics_dt:.6f}s ({1.0/self.physics_dt:.0f} Hz), "
            f"ROS state pub={self.state_pub_hz:.0f} Hz, GUI={self.gui}"
        )

    # ---------- PyBullet setup ----------
    def _sim_init(self):
        if self.gui:
            p.connect(p.GUI, options="--opengl3")
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, -9.81, 0)  # Y-up world
        p.setTimeStep(self.physics_dt)
        p.loadURDF("plane.urdf")

        # Load dip URDF
        pkg_share = get_package_share_directory("dip_sim")
        urdf_path = os.path.join(pkg_share, "urdf", "dip_2.urdf")
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF not found: {urdf_path}")
            raise FileNotFoundError(urdf_path)
        self.robot = p.loadURDF(urdf_path, [0, 0, 0.2], useFixedBase=True)

        self.joint1_index = 0
        self.joint2_index = 1

        p.resetJointState(self.robot, self.joint1_index, targetValue=math.radians(90.0))
        p.resetJointState(self.robot, self.joint2_index, targetValue=math.radians(90.0))

        # Disable default motors
        p.setJointMotorControl2(
            self.robot, self.joint1_index, p.VELOCITY_CONTROL, force=0
        )
        p.setJointMotorControl2(
            self.robot, self.joint2_index, p.VELOCITY_CONTROL, force=0
        )

    # ---------- ROS callbacks ----------
    def torque_callback(self, msg: Float64):
        with self._torque_lock:
            self._torque = float(msg.data)

    def _on_params(self, params):
        updated = []
        for p in params:
            if p.name == "angle_mode" and p.type == p.Type.STRING:
                self.angle_mode = p.value
                updated.append("angle_mode")
            elif p.name == "theta1_offset" and p.type in (
                p.Type.DOUBLE,
                p.Type.INTEGER,
            ):
                self.theta1_offset = float(p.value)
                updated.append("theta1_offset")
            elif p.name == "theta2_offset" and p.type in (
                p.Type.DOUBLE,
                p.Type.INTEGER,
            ):
                self.theta2_offset = float(p.value)
                updated.append("theta2_offset")
        if updated:
            self.get_logger().info(f"Updated params: {', '.join(updated)}")
        return SetParametersResult(successful=True)

    # ---------- Helpers ----------
    @staticmethod
    def _wrap_pi(x):
        return (x + math.pi) % (2 * math.pi) - math.pi

    # ---------- Main physics loop ----------
    def _physics_loop(self):
        last_wall = time.perf_counter()
        while not self._stop_evt.is_set():
            # 1) Apply most recent torque (sample-and-hold)
            with self._torque_lock:
                torque = self._torque

            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=self.joint1_index,
                controlMode=p.TORQUE_CONTROL,
                force=torque,
            )

            # 2) Step physics
            p.stepSimulation()
            self._sim_time += self.physics_dt
            self._time_since_pub += self.physics_dt

            # 3) Publish /clock every step (sim-time)
            clk = Clock()
            # rclpy Clock expects nanoseconds
            clk.clock = Time(seconds=self._sim_time).to_msg()
            self.clock_pub.publish(clk)

            # 4) Publish state at decimated rate
            if self._time_since_pub >= self._publish_interval:
                self._time_since_pub = 0.0

                theta1_raw, theta1_dot_raw, _, _ = p.getJointState(
                    self.robot, self.joint1_index
                )
                theta2_raw, theta2_dot_raw, _, _ = p.getJointState(
                    self.robot, self.joint2_index
                )

                if self.angle_mode == "absolute":
                    theta1 = self._wrap_pi(theta1_raw - self.theta1_offset)
                    theta2 = self._wrap_pi(theta1_raw + theta2_raw - self.theta2_offset)
                    theta1_dot = theta1_dot_raw
                    theta2_dot = theta1_dot_raw + theta2_dot_raw
                else:  # relative
                    theta1 = self._wrap_pi(theta1_raw - self.theta1_offset)
                    theta2 = self._wrap_pi(theta2_raw - self.theta2_offset)
                    theta1_dot = theta1_dot_raw
                    theta2_dot = theta2_dot_raw

                # JointState (useful for RViz, plotting)
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = ["joint1", "joint2"]
                js.position = [theta1_raw, theta2_raw]
                js.velocity = [theta1_dot_raw, theta2_dot_raw]
                js.effort = [torque, 0.0]
                self.joint_state_pub.publish(js)

                # Custom DipState (controller input)
                ds = DipState()
                ds.theta1 = theta1
                ds.theta2 = theta2
                ds.theta1_dot = theta1_dot
                ds.theta2_dot = theta2_dot
                self.state_pub.publish(ds)

            # 5) Soft real-time pacing (don’t busy-wait)
            #    Try to keep wall-time close to physics time for smooth GUI.
            now = time.perf_counter()
            elapsed = now - last_wall
            target = self.physics_dt
            sleep_s = target - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
            last_wall = time.perf_counter()

    def destroy(self):
        self._stop_evt.set()
        if self._physics_thread.is_alive():
            self._physics_thread.join(timeout=1.0)
        p.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DipSimulator()
    try:
        # Multi-threaded executor is standard to avoid callback blocking
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
