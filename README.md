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
- PyBullet (`pip install pybullet`)
- OpenCV (`pip install opencv-python`)
- cv_bridge (`sudo apt install ros-${ROS_DISTRO}-cv-bridge`)
- (Optional) Jetson Orin Nano for deployment

---

## 🛠️ Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/SAMUEL-CWC/double_inverted_pendulum.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 🚀 Running the Simulation
### 1. Run the PyBullet Simulator
```bash
ros2 run dip_sim sim_node
```
This launches the double pendulum simulation and publishes joint states.

---

### 2. Run the LQR Controller
```bash
ros2 run dip_control lqr_controller
```
Subscribes to joint states, applies u = -Kx control law, and publishes motor commands.

---

### 3. Run the RL Controller (Optional / Coming Soon)
```bash
ros2 run dip_control rl_controller
```
Loads a trained RL policy and interacts with the simulation or real hardware.

---

### 4. Run the vision node
```bash
ros2 run dip_vision vision_node
```
Captures webcam input, tracks link markers, and estimates angle and angular velocity of the second link.

---

## 📂 Workspace Structure
```bash
ros2_ws/
├── src/
│   ├── dip_sim/         # Simulation node
│   ├── dip_control/     # LQR and RL controllers
│   ├── dip_vision/      # Vision processing node
│   └── dip_msgs/        # (Optional) Custom ROS 2 messages
├── build/
├── install/
└── log/
```

---

## 📈 Project Goals
- Compare LQR vs RL control strategies under noisy sensor feedback

- Test robustness of vision-based state estimation

- Transition simulation to real-world control with a Jetson Orin Nano

- Build a reproducible, shareable ROS 2 control stack for educational and research use

---

## 👨‍🔬 Maintainer Info
- Maintainer: Samuel Chien

- Lab: Mechatronics and Controls Laboratory, UCLA

- Email: samuelbruin0618@g.ucla.edu

## 📜 License
This project is licensed under the BSD 3-Clause License.
See the LICENSE file for more details.
```yaml

---

Would you like me to fill in your actual name, lab, or email? Or generate this as a file you can drop right into your repo?
```
