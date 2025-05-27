# Q-Learning based mobile robot navigation

**An ROS2-based autonomous mobile robot integrating wheel odometry, PID velocity control, LiDAR obstacle detection, and Q-learning path planning for adaptive navigation in grid environments.**

## Overview

This project implements an autonomous mobile robot using ROS2. It combines the following components:

* **Encoder Publisher**: Reads wheel encoder pulses and publishes wheel velocities.
* **Motor Subscriber**: Sends velocity commands to motor driver via serial port and publishes motor direction signs.
* **PID Control Node**: Receives encoder feedback and desired path, computes wheel PWMs with PID loops, and drives the robot along grid paths.
* **Obstacle Detection**: Uses LiDAR to detect obstacles, publishes detection events to trigger replanning.
* **Q-Learning Node**: Learns optimal grid-based navigation policy in the presence of obstacles, outputs next waypoints.

The flow of data is visualized in the included `rqt_graph`:![image](https://github.com/user-attachments/assets/6f59c7e3-ff5b-4e3c-b0d4-f9f9f73c3759)


```
/motor_sub --> enc_pub --> pid_control_node --> obstacle_detection --> q_learning_node
```

(See rqt\_graph.png for full topic connections.)

---

## Table of Contents

* [Requirements](#requirements)
* [Installation](#installation)
* [Configuration](#configuration)
* [Usage](#usage)
* [Nodes & Topics](#nodes--topics)
* [Folder Structure](#folder-structure)
* [Troubleshooting](#troubleshooting)

---

## Requirements

* Ubuntu 20.04 / 22.04
* ROS2 Foxy / Galactic / Humble
* Python 3.8+
* `gpiozero`, `numpy`
* LiDAR with ROS2 driver (e.g., SLAMTEC S1)
* Serial-connected motor driver on `/dev/ttyUSB1`

---

## Installation

```bash
# Clone the application into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/karanpatelkp/autonomous_robot.git

# Install Python dependencies
cd ../
pip3 install -r src/autonomous_robot/requirements.txt

# Build the workspace
colcon build
source install/setup.bash
```

---

## Configuration

* **Encoder Publisher** (`encoder_pub.py`)

  * `diameter`: Wheel diameter in meters.
  * `ppr`: Pulses per revolution.
* **Obstacle Detection** (`obstacle_detection.py`)

  * `obstacle_threshold` ROS2 parameter (default `0.5` m).
* **PID Control Node** (`pid_control_node.py`)

  * PID gains: `kp`, `ki`, `kd` (ros2 params).
  * Grid step size: `0.25` m.
* **Q-Learning Node** (`q_learning_node.py`)

  * `grid_size`: Number of cells per axis.
  * Learning rates: `alpha`, `gamma`, `epsilon_decay`.

Adjust these parameters via ROS2 CLI:

```bash
ros2 param set /pid_control_node kp 1.0
ros2 param set /obstacle_detection obstacle_threshold 0.3
```

---

## Usage

1. **Start LiDAR driver** (if separate):

   ```bash
   ros2 launch slamtec_s1 slamtec.launch.py
   ```
2. **Launch all nodes**:

   ```bash
   # In a single terminal
   ros2 run autonomous_robot encoder_pub
   ros2 run autonomous_robot motor_sub
   ros2 run autonomous_robot pid_control_node
   ros2 run autonomous_robot obstacle_detection
   ros2 run autonomous_robot q_learning_node
   ```
3. **Visualize graph**:

   ```bash
   ros2 run rqt_graph rqt_graph
   ```
4. **Monitor topics**:

   ```bash
   ros2 topic echo /enc_val
   ros2 topic echo /mot_val
   ros2 topic echo /lidar_val
   ```

---

## Nodes & Topics

| Node                 | Subscribes                      | Publishes                          | Purpose                                                         |
| -------------------- | ------------------------------- | ---------------------------------- | --------------------------------------------------------------- |
| `encoder_pub`        | `/mot_sign` (`Int32MultiArray`) | `/enc_val` (`Float32MultiArray`)   | Measures encoder pulses, computes & publishes wheel velocities. |
| `motor_sub`          | `/mot_val` (`Int32MultiArray`)  | `/mot_sign` (`Int32MultiArray`)    | Sends commands to motor driver; publishes direction signs.      |
| `pid_control_node`   | `/enc_val`, `/path_data`        | `/mot_val`, `/Completed`           | PID loops for velocity control; computes path following.        |
| `obstacle_detection` | `/scan`, `/Completed`           | `/lidar_val` (`Float32MultiArray`) | Detects obstacles from LiDAR; triggers replanning.              |
| `q_learning_node`    | `/lidar_val`                    | `/path_data` (`Int32MultiArray`)   | Q-Learning path planner in grid world.                          |

---

## Folder Structure

```
autonomous_robot/
├── package.xml
├── setup.py
├── requirements.txt
├── src/
│   ├── encoder_pub.py
│   ├── motor_sub.py
│   ├── pid_control_node.py
│   ├── obstacle_detection.py
│   └── q_learning_node.py
└── README.md
```

---

## Troubleshooting

* **Serial port errors**: Ensure `/dev/ttyUSB1` exists and permissions are set.
* **No LiDAR data**: Check LiDAR driver and ROS2 topic `/scan`.
* **Unresponsive motors**: Verify power and wiring of motor driver.
* **PID tuning**: Use `ros2 param` to adjust gains and observe response.

---

For further assistance, please open an issue on the repository.
