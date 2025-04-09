# simulate.py (PyBullet + ROS 2 integration)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import pybullet as p
import pybullet_data
import time
import math
import os

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading


class DipSimulator(Node):
    def __init__(self):
        super().__init__('dip_simulator')

        # ROS 2 Publishers``
        self.publisher_theta1 = self.create_publisher(Float64, 'theta1', 10)
        self.publisher_theta2 = self.create_publisher(Float64, 'theta2', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # ROS 2 Subscriber
        self.subscription = self.create_subscription(Float64, 'torque_cmd', self.torque_callback, 10)

        # Start PyBullet simulation
        self.sim_init()

        # Create ROS 2 timer
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        
        # Angle data history for plotting
        self.angle_history_len = 200
        self.theta1_log = deque(maxlen=self.angle_history_len)
        self.theta2_log = deque(maxlen=self.angle_history_len)
        self.time_log = deque(maxlen=self.angle_history_len)

        # Start the live plot in another thread
        threading.Thread(target=self.live_plot_thread, daemon=True).start()
        
    def live_plot_thread(self):
        plt.ion()
        fig, ax = plt.subplots()
        line1, = ax.plot([], [], label='θ₁')
        line2, = ax.plot([], [], label='θ₂')
        ax.set_ylim(-math.pi, math.pi)
        ax.set_xlim(0, self.angle_history_len)
        ax.legend()
        ax.set_title("Joint Angles Over Time")
        ax.set_ylabel("θ (rad)")
        ax.set_xlabel("Time (steps)")

        while True:
            if len(self.theta1_log) > 0:
                line1.set_ydata(self.theta1_log)
                line2.set_ydata(self.theta2_log)
                line1.set_xdata(range(len(self.theta1_log)))
                line2.set_xdata(range(len(self.theta2_log)))
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)


    def sim_init(self):
        # Connect to PyBullet
        p.connect(p.GUI)  # Use p.GUI for visualization
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, -9.81, 0)
        p.setTimeStep(0.01)

        # Load plane and URDF
        p.loadURDF("plane.urdf")
        urdf_path = os.path.join(
            os.path.dirname(__file__), '..', 'urdf', 'double_pendulum.urdf'
        )
        urdf_path = os.path.abspath(urdf_path)

        self.robot = p.loadURDF(urdf_path, [0, 0, 0.2], useFixedBase=True)

        self.joint1_index = 0
        self.joint2_index = 1

        # Disable default motors
        p.setJointMotorControl2(self.robot, self.joint1_index, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(self.robot, self.joint2_index, p.VELOCITY_CONTROL, force=0)
        
        # p.resetDebugVisualizerCamera(
        #     cameraDistance=0.3,
        #     cameraYaw=90,     # horizontal angle
        #     cameraPitch=-90,  # look downward along Y
        #     cameraTargetPosition=[0, -0.05, 0]
        # )
        
        # p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0], 2, 0)  # X (red)
        # p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0], 2, 0)  # Y (green)
        # p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1], 2, 0)  # Z (blue)


    def torque_callback(self, msg):
        self.torque = msg.data
        
    def map_angle(self, angle):
        a = angle + math.pi
        return (a + math.pi) % (2 * math.pi) - math.pi  # wrap to [-pi, pi]

    def timer_callback(self):
        self.torque = 0.01*math.sin(time.time())
        
        # Apply torque to joint1
        p.setJointMotorControl2(
            bodyUniqueId=self.robot,
            jointIndex=self.joint1_index,
            controlMode=p.TORQUE_CONTROL,
            force=self.torque
        )
        self.get_logger().info(f"Applied torque: {self.torque}")


        # Step simulation
        p.stepSimulation()


        # Read joint states
        theta1 = self.map_angle(p.getJointState(self.robot, self.joint1_index)[0])
        theta2 = self.map_angle(p.getJointState(self.robot, self.joint2_index)[0])
        theta1_dot = p.getJointState(self.robot, self.joint1_index)[1]
        self.get_logger().info(f"θ₁̇ = {theta1_dot}")
        theta2_dot = p.getJointState(self.robot, self.joint2_index)[1]
        
        # Remove previous debug text
        if hasattr(self, 'angle_text_ids'):
            for txt_id in self.angle_text_ids:
                p.removeUserDebugItem(txt_id)

        # Display updated angles
        self.angle_text_ids = [
            p.addUserDebugText(
                f"θ₁: {math.degrees(theta1):.1f}°",
                [0, 0.1, 0.1], textColorRGB=[1, 0, 0], textSize=1.2
            ),
            p.addUserDebugText(
                f"θ₂: {math.degrees(theta2):.1f}°",
                [0, -0.1, 0.1], textColorRGB=[0, 0, 1], textSize=1.2
            )
        ]
        
        self.theta1_log.append(theta1)
        self.theta2_log.append(theta2)
        self.time_log.append(time.time())

        # Publish individual joint angles
        self.publisher_theta1.publish(Float64(data=theta1))
        self.publisher_theta2.publish(Float64(data=theta2))

        # Publish joint_states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2']
        joint_state_msg.position = [theta1, theta2]
        joint_state_msg.velocity = [theta1_dot, theta2_dot]
        joint_state_msg.effort = [self.torque, 0.0]
        self.joint_state_pub.publish(joint_state_msg)

        self.get_logger().info(f"theta1: {math.degrees(theta1):.2f}°, theta2: {math.degrees(theta2):.2f}°, torque: {self.torque:.3f}")
        
    def pybullet_loop(self):
        while True:
            p.stepSimulation()
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = DipSimulator()
    rclpy.spin(node)
    # node.run_simulation()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
