# ROS 2 Vision Hand Teleoperation

A vision-based ROS 2 teleoperation application for controlling a simulated KUKA iiwa 14 arm and Schunk SVH right hand from real-time MediaPipe hand landmarks.

The application supports independent finger flexion and lateral motion, configurable Cartesian arm control, MoveIt 2 inverse kinematics, `ros2_control`, and RViz visualization.

## Features

- Real-time hand tracking with MediaPipe
- KUKA iiwa 14 arm with Schunk SVH right hand
- Independent finger flexion control
- Independent index, ring, and pinky lateral control
- Configurable Cartesian and Cartesian-delta arm teleoperation
- Screen-centered Cartesian-delta motion
- Calibrated Cartesian Y/Z positioning
- MoveIt 2 inverse kinematics
- `ros2_control` trajectory and hand position controllers
- RViz visualization

## Installation

The commands below clone the application first, then use the existing root-level `dex-urdf.repos` manifest to import `dex-urdf` into `robot_description`.

If `vcs` or `rosdep` is not installed:

```bash
sudo apt update
sudo apt install -y python3-vcstool python3-rosdep
```

Initialize `rosdep` only if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

Clone the application:

```bash
mkdir -p ~/ros2_vision_hand_teleop_ws/src
cd ~/ros2_vision_hand_teleop_ws/src

git clone https://github.com/kimsniper/ros2_vision_hand_teleop
```

Import `dex-urdf` using the cloned application's existing root-level VCS manifest:

```bash
cd ~/ros2_vision_hand_teleop_ws

vcs import src < \
  src/ros2_vision_hand_teleop/dex-urdf.repos
```

Install dependencies with the cloned application's setup script:

```bash
source /opt/ros/humble/setup.bash

chmod +x \
  src/ros2_vision_hand_teleop/setup.sh

src/ros2_vision_hand_teleop/setup.sh
```

Build and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

For an existing workspace, update the application and `dex-urdf` with:

```bash
cd ~/ros2_vision_hand_teleop_ws
vcs pull src
```

Install ROS package dependencies manually when needed:

```bash
cd ~/ros2_vision_hand_teleop_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## System Architecture

### Vision

`robot_vision` captures webcam frames, runs MediaPipe hand tracking, and publishes 21 landmarks for the right hand.

```text
/hand/landmarks
```

The timestamped message contains 63 `float64` landmark values and an explicit detection flag:

```text
right hand: 21 landmarks × x/y/z = 63 values
```

### Hand Teleoperation

`robot_teleop` maps MediaPipe landmarks to the Schunk SVH hand controller.

The hand controller commands:

1. Thumb flexion
2. Thumb opposition
3. Index distal flexion
4. Index proximal flexion
5. Middle proximal flexion
6. Middle distal flexion
7. Ring flexion
8. Pinky flexion
9. Index lateral spread
10. Ring lateral spread
11. Pinky lateral spread

The middle finger remains laterally fixed because the Schunk model does not provide a middle-finger lateral degree of freedom.

### Arm Teleoperation

The application provides two selectable arm control modes.

#### Cartesian

```text
arm_control_mode:=cartesian
```

The first stable palm detection becomes the screen reference. Palm motion is mapped to absolute robot Y/Z targets while robot X and end-effector orientation remain fixed.

```text
/arm/cartesian_target
```

Nodes:

```text
/arm_cartesian_teleop_node
/cartesian_control_node
```

#### Cartesian Delta

```text
arm_control_mode:=cartesian_delta
```

The center of the camera image is the neutral point. Palm displacement from the screen center produces bounded Y/Z Cartesian increments. The latest command replaces stale waiting commands, and returning to the center cancels retained movement.

```text
/arm/cartesian_delta
```

Nodes:

```text
/arm_cartesian_delta_teleop_node
/cartesian_delta_control_node
```

Robot X remains fixed in both modes.

## Running the Application

Source ROS 2 and the workspace in every terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_vision_hand_teleop_ws/install/setup.bash
```

### Cartesian Mode

```bash
ros2 launch robot_bringup application.launch.py \
  arm_control_mode:=cartesian
```

### Cartesian-Delta Mode

```bash
ros2 launch robot_bringup application.launch.py \
  arm_control_mode:=cartesian_delta
```

`arm_control_mode` accepts only:

```text
cartesian
cartesian_delta
```

## Main Topics

| Topic | Type | Description |
|---|---|---|
| `/hand/landmarks` | `robot_vision_msgs/msg/HandLandmarks` | Timestamped MediaPipe landmarks for the right hand |
| `/arm/cartesian_target` | `geometry_msgs/msg/PoseStamped` | Absolute Cartesian arm target |
| `/arm/cartesian_delta` | `geometry_msgs/msg/TwistStamped` | Cartesian arm increment |
| `/arm_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Arm trajectory action |
| `/hand_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Schunk SVH joint commands |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot joint feedback |
| `/tf` | `tf2_msgs/msg/TFMessage` | Robot transforms |

## Control Behavior

### Cartesian

- Calibrates from the first stable palm detection
- Maps screen X to robot Y
- Maps screen Y to robot Z
- Keeps robot X fixed
- Keeps the configured end-effector orientation fixed
- Publishes only when the target changes beyond the configured threshold

### Cartesian Delta

- Uses the screen center as zero motion
- Maps screen X displacement to robot Y increments
- Maps screen Y displacement to robot Z increments
- Keeps robot X fixed
- Retains only the latest waiting command
- Cancels retained motion when the palm returns to the center
- Limits each accepted Cartesian increment for smoother motion

## Working Demo

![MediaPipe ROS 2 teleoperation demo](./images/mediapipe-ros2-demo.gif)

[Vision Hand Teleoperation Demo](https://www.linkedin.com/feed/update/urn:li:activity:7461981548058144768/)
