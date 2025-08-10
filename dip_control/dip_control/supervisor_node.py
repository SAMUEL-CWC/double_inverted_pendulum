# supervisor_node.py (in dip_control package)
import rclpy
from rclpy.node import Node
from dip_interfaces.msg import DipState  # custom message
from std_msgs.msg import String
import math


class DipSupervisor(Node):
    def __init__(self):
        super().__init__("dip_supervisor")

        # Current mode: "idle", "balance", "swing_up"
        self.mode = "idle"

        # Thresholds for mode switching (example values)
        self.balance_threshold = 20.0  # degrees

        # Publisher: mode topic
        self.mode_pub = self.create_publisher(String, "dip_mode", 10)

        # Subscriber: state feedback
        self.state_sub = self.create_subscription(
            DipState, "dip_state", self.state_callback, 10
        )

        # Timer to publish current mode
        self.timer = self.create_timer(0.1, self.publish_mode)

    def state_callback(self, msg):
        # Extract angle in degrees
        theta1_deg = math.degrees(msg.theta1)

        # Simple logic: if theta1 is upright-ish, go to balance
        if abs(theta1_deg) < self.balance_threshold:
            if self.mode != "balance":
                self.get_logger().info("🟢 Switching to BALANCE mode")
                self.mode = "balance"
        else:
            if self.mode != "swing_up":
                self.get_logger().info("🔄 Switching to SWING-UP mode")
                self.mode = "swing_up"

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DipSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
