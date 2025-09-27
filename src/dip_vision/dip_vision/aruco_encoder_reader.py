import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time


class VisionAnglePublisher(Node):
    def __init__(self):
        super().__init__("vision_angle_publisher")

        # Publishers for joint anglesㄨ
        self.theta1_pub = self.create_publisher(Float64, "theta1", 10)
        self.theta2_pub = self.create_publisher(Float64, "theta2", 10)

        # Timer to publish at fixed rate
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1000 Hz

        # ArUco setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")
            return

        # Camera calibarion
        self.camera_matrix = np.array(
            [[800, 0, 320][0, 800, 240], [0, 0, 1]], dtype=np.float32
        )
        self.dist_coeffs = np.zeros((4, 1))

        # Physical marker size (in meters)
        self.marker_length = 0.016  # 16mm

    def get_theta2(self, frame):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from webcam")
            return None

        # Convertt the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        # Prink the detected markers
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            for rvec, tvec in zip(rvecs, tvecs):
                # Draw pose
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.02
                )

                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Get Z-axis (3rd column) in camera frame
                z_axis = R[:, 2]

                # Compute angle of marker rotation in the XY plane (yaw)
                yaw_rad = math.atan2(R[1, 0], R[0, 0])
                yaw_deg = math.degrees(yaw_rad)
                self.get_logger().info(f"Yaw angle: {yaw_deg:.2f} degrees")
                return yaw_deg
        return None

    def get_theta1(self):
        # Create port for encoder reading
        if self.serial_port is not None and self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline().decode("utf-8").strip()
                angle_deg = float(line)
                return math.radians(angle_deg)
            except Exception as e:
                self.get_logger().warn(f"Failed to read or parse encoder data: {e}")
        return 0.0  # default fallback
