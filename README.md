# ROS2 Vision Hand Teleop

A ROS2-based vision-driven teleoperation system that maps real-time hand gestures (via MediaPipe) to a simulated humanoid robotic hand in RViz.

This project integrates:
- Computer vision-based hand tracking using MediaPipe
- Gesture-to-joint mapping pipeline
- ROS2-based communication and control
- RViz visualization of a fully articulated robotic hand

---

## System Overview

The system consists of three main components:

### 1. Vision Layer (MediaPipe Node)
- Captures live webcam feed
- Extracts 21 hand landmarks using MediaPipe
- Publishes landmarks as ROS2 messages `/hand/landmarks`

### 2. Teleoperation Layer (Teleop Node)
- Receives hand landmarks
- Computes finger curl angles
- Maps gestures to 15 joint positions
- Publishes `/joint_states`

### 3. Visualization Layer (RViz + Robot State Publisher)
- Loads URDF hand model
- Visualizes joint states in real-time in RViz

---

## ROS2 Launch Modes

### Display Mode (Manual Joint Testing)
File: `display.launch.py`

Purpose:
Used for debugging and manual joint testing using GUI sliders.

Launches:
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

Run:
```bash
ros2 launch robot_bringup display.launch.py
```

---

### Teleoperation Mode (Vision Control)
File: `teleop.launch.py`

Purpose:
Full vision-based teleoperation pipeline using MediaPipe.

Launches:
- robot_state_publisher
- robot_vision (MediaPipe node)
- robot_teleop (teleop node)
- rviz2

Run:
```bash
ros2 launch robot_bringup teleop.launch.py
```

---

## Robot Model

URDF: `robot_description/urdf/robot_hand.urdf`

Features:
- 5-finger humanoid hand
- 15 revolute joints
- Realistic joint limits
- Simplified geometry for RViz visualization

---

## Build Instructions

### 1. Clone repository
```bash
git clone https://github.com/kimsniper/ros2_vision_hand_teleop.git
cd ros2_vision_hand_teleop
```

### 2. Install dependencies
```bash
sudo apt install ros-$ROS_DISTRO-rviz2 \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui
```

Python dependencies:
```bash
pip install mediapipe opencv-python numpy
```

### 3. Build workspace
```bash
colcon build
```

### 4. Source setup
```bash
source install/setup.bash
```

---

## Topic Architecture

| Topic | Type | Description |
|------|------|-------------|
| /hand/landmarks | Float64MultiArray | MediaPipe hand landmarks |
| /joint_states | JointState | Robot joint commands |

---

## Package Structure

robot_bringup - Launch files  
robot_control - Control logic  
robot_description - URDF, RViz config  
robot_teleop - Landmark, joint mapping  
robot_vision - MediaPipe vision node  

---

## How It Works

1. Webcam captures hand movement  
2. MediaPipe extracts 21 landmarks  
3. Finger curl estimation  
4. Mapping to joint angles  
5. Publish to ROS2 `/joint_states`  
6. RViz renders humanoid hand motion  
