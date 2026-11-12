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

### Hand State Estimation

MediaPipe landmark measurements can contain frame-to-frame jitter, irregular timing, and short tracking interruptions. The `robot_state_estimation` package provides a dedicated `state_estimator_node` between vision and teleoperation so robot commands are generated from an estimated hand state instead of directly from raw landmarks.

```text
Camera frame
    |
MediaPipe right-hand landmarks
    |  /hand/landmarks
state_estimator_node
    |  /hand/landmarks/filtered
Cartesian, Cartesian-delta, and hand teleoperation
```

The estimator runs 63 independent constant-velocity Kalman filters, one for every `x`, `y`, and `z` coordinate across the 21 MediaPipe hand landmarks. Each coordinate uses the state:

```text
state = [position, velocity]
measurement = position
```

The measurement timestamp determines the update interval `dt`. The estimator preserves the input timestamp and `frame_id` in the filtered output.

Estimator behavior:

- The first valid detection initializes all 63 filters directly from the measured landmarks.
- Valid timestamped measurements perform Kalman prediction and correction for every coordinate.
- A missing, non-increasing, or excessively delayed timestamp reinitializes the filters from the current measurement.
- `hand_detected=false` immediately resets all filters and publishes 63 zeros with detection disabled.
- The first valid measurement after tracking loss initializes a new estimate without carrying stale velocity forward.

This immediate reset behavior preserves the teleoperation safety path: Cartesian-delta control receives a zero stop command, while hand teleoperation suppresses new commands until the right hand is detected again.

#### Kalman Filter Parameters

| Parameter | Default | Purpose |
|---|---:|---|
| `process_noise` | `0.1` | Allows estimated landmark velocity to change |
| `measurement_noise` | `0.01` | Controls trust in MediaPipe measurements |
| `initial_position_variance` | `1.0` | Initial landmark-position uncertainty |
| `initial_velocity_variance` | `1.0` | Initial landmark-velocity uncertainty |
| `minimum_dt` | `0.001 s` | Prevents updates with an excessively small interval |
| `maximum_dt` | `0.5 s` | Reinitializes after stale or delayed measurements |

Higher `measurement_noise` produces smoother estimates but can introduce more motion lag. Higher `process_noise` follows rapid hand motion more closely but can pass more measurement jitter. The current values are initial operating defaults and should be tuned using measured raw-versus-filtered jitter and motion latency for the target camera and lighting conditions.

The raw and filtered landmark streams can be inspected with:

```bash
ros2 topic echo /hand/landmarks
ros2 topic echo /hand/landmarks/filtered
ros2 topic hz /hand/landmarks
ros2 topic hz /hand/landmarks/filtered
```

The timestamped message contains 63 `float64` landmark values and an explicit detection flag:

```text
right hand: 21 landmarks × x/y/z = 63 values
```

#### Depth Estimation

The state estimator derives camera-depth motion from apparent palm scale.
MediaPipe landmark `z` is not used directly as robot depth because it is
relative to the hand geometry and is not a stable metric camera distance.

`PalmStateExtractor` calculates:

- palm center from landmarks `0`, `5`, `9`, `13`, and `17`;
- robust palm scale from four palm distances;
- normalized palm normal;
- orientation-gated depth validity.

The depth measurement is relative to the first valid palm scale:

```text
relative_depth = log(current_palm_scale / reference_palm_scale)
```

Positive relative depth means the hand moved toward the camera.
Negative relative depth means it moved away.

```text
MediaPipe landmarks
    |
    v
63 coordinate Kalman filters
    |
    v
PalmStateExtractor
    |-- palm center
    |-- palm scale
    |-- palm normal
    `-- depth validity
    |
    v
Relative-depth Kalman filter
    |-- relative depth
    `-- depth velocity
    |
    v
/hand/state
    |-- primary Cartesian pose control
    `-- secondary Cartesian-delta control
```

The estimator publishes `robot_vision_msgs/msg/HandState` on
`/hand/state`. It contains filtered landmarks, palm geometry, relative
depth, depth velocity, and depth validity.

Depth is disabled when the palm is strongly edge-on. Detection loss,
invalid geometry, or stale timestamps reset the depth reference and filter.

##### State-estimation parameters

| Parameter | Default | Purpose |
|---|---:|---|
| `depth_scale_deadband` | `0.03` | Ignores palm-scale changes of 3% or less |
| `minimum_palm_normal_z` | `0.70` | Rejects strongly edge-on palm orientations |
| `process_noise` | `0.1` | Allows estimated position and velocity to change |
| `measurement_noise` | `0.01` | Controls trust in measurements |
| `minimum_dt` | `0.001 s` | Protects against very small update intervals |
| `maximum_dt` | `0.5 s` | Reinitializes after stale measurements |

##### Primary Cartesian pose control

```text
robot x = reference x + relative depth * workspace_x_scale
robot y = reference y - horizontal palm offset * workspace_y_scale
robot z = reference z - vertical palm offset * workspace_z_scale
```

| Parameter | Default | Purpose |
|---|---:|---|
| `workspace_x_scale` | `0.30` | Converts relative depth to robot x displacement |
| `maximum_x_offset` | `0.15 m` | Limits forward and backward displacement |
| `reacquisition_samples` | `5` | Valid states required for calibration |

Initial calibration and reacquisition average five consecutive valid palm
centers. No Cartesian target is published during this calibration window.
Detection loss or invalid depth resets the pending calibration.

##### Secondary Cartesian-delta control

```text
linear.x = depth_velocity * depth_velocity_scale
```

| Parameter | Default | Purpose |
|---|---:|---|
| `depth_velocity_scale` | `0.30` | Converts depth velocity to robot x motion |
| `maximum_depth_velocity` | `0.10 m/s` | Limits depth velocity |

Invalid depth or tracking loss immediately produces a zero stop.

Inspect the estimated state with:

```bash
ros2 topic echo /hand/state
ros2 topic hz /hand/state
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
| `/hand/landmarks` | `robot_vision_msgs/msg/HandLandmarks` | Raw timestamped MediaPipe landmarks for the right hand |
| `/hand/landmarks/filtered` | `robot_vision_msgs/msg/HandLandmarks` | Kalman-filtered landmarks used by teleoperation |
| `/hand/state` | `robot_vision_msgs/msg/HandState` | Estimated palm geometry, relative depth, and depth velocity |
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
