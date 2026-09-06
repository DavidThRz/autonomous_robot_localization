# Autonomous Robot Localization (Visual Odometry & Streaming)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/Language-C%2B%2B17-green)

## Overview
This ROS2 package provides a real-time Visual Odometry (VIO) pipeline and camera streaming utilities for 2D mobile robots. It is designed to capture, process, and stream visual data while extracting motion estimation (odometry) from the video feed.

## System Architecture and Nodes

The package is modularized into specific C++ nodes handling different stages of the visual pipeline:

* **`visual_node`**: The core perception node. Processes the incoming image stream, extracts visual features, and publishes visual odometry.
* **`streaming_node`**: Handles real-time video streaming over HTTP (using `cpp-httplib`), allowing remote monitoring of the robot's camera feed with minimal latency.
* **`photographer_node`**: Captures images from the hardware camera.
* **`mapper_node`**: Records the fused trajectory and generates a 2D map on shutdown.

## Build Instructions

### Prerequisites
* ROS2 (Humble / Iron)
* C++17 Compiler
* OpenCV (for visual odometry feature extraction)

### Building the Package
Clone the repository into your ROS2 workspace `src` directory and build using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_robot_localization_pkg --symlink-install
source install/setup.bash
```

## Launchers

The launchers load node parameters exclusively from the YAML files in `config/`.

### `localization_core.launch.py`

Starts the localization pipeline without hardware-specific drivers:

- `visual_node`
- `ekf_node`
- `mapper_node`
- `streaming_node`

All nodes load [config/localization_core.yaml](config/localization_core.yaml), which configures visual odometry, the EKF, mapper output, streaming, and `use_sim_time: false`.

```bash
ros2 launch autonomous_robot_localization_pkg localization_core.launch.py
```

### `localization_hardware.launch.py`

Starts the complete pipeline for the physical robot:

- `photographer_node` for the camera hardware
- `imu_node` for the SPI IMU
- all nodes from `localization_core.launch.py`

It loads [config/localization_core.yaml](config/localization_core.yaml). In particular, configure the SPI device, IMU rate, and frame ID in this file before using the launcher.

```bash
ros2 launch autonomous_robot_localization_pkg localization_hardware.launch.py
```

### `localization_replay.launch.py`

Replays a rosbag and starts `visual_node`, `ekf_node`, `mapper_node`, and `streaming_node`. It loads the common configuration first, then [config/localization_replay.yaml](config/localization_replay.yaml), which sets `use_sim_time: true` for those nodes.

```bash
ros2 launch autonomous_robot_localization_pkg localization_replay.launch.py \
  bag_path:=/path/to/rosbag2_directory
```

`bag_path` is required and selects the input rosbag; it is not a node configuration parameter. Playback uses the rosbag clock, runs once, and has a fixed rate of `1.0`.

## Configuration

- [config/localization_core.yaml](config/localization_core.yaml): common node parameters for hardware and core operation.
- [config/localization_replay.yaml](config/localization_replay.yaml): replay overrides (`use_sim_time`).
