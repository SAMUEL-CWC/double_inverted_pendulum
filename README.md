# Double Inverted Pendulum Control (ROS 2 Jazzy)

This project simulates and controls a double inverted pendulum using **PyBullet**, **ROS 2 Jazzy**, and both classical (LQR) and modern (RL) control methods. It includes modules for simulation, control, vision-based state estimation, and system integration with Jetson hardware.

---

## 📦 Packages

| Package Name         | Description                                         |
|----------------------|-----------------------------------------------------|
| `dip_sim`            | PyBullet-based simulation of the double pendulum    |
| `dip_control`        | Contains LQR controller and RL policy runner        |
| `dip_vision`         | Image processing for camera-based joint estimation  |
| `dip_msgs` (optional) | Custom message types for unified state info         |

---

## ⚙️ System Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.10+
- PyBullet
- OpenCV (for vision)
- (Optional) Jetson Orin Nano (for deployment)

---

## 🛠️ Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourlab/double_pendulum_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
