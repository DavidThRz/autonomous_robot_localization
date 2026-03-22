# Autonomous Robot Localization (Visual Odometry & Streaming)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/Language-C%2B%2B17-green)

## 📌 Overview
This ROS2 package provides a real-time Visual Odometry (VIO) pipeline and camera streaming utilities for 2D mobile robots. It is designed to capture, process, and stream visual data while extracting motion estimation (odometry) from the video feed.

## ⚙️ System Architecture & Nodes

The package is modularized into specific C++ nodes handling different stages of the visual pipeline:

* **`visual_odometry`**: The core perception node. Processes the incoming image stream, extracts visual features, and computes the 2D motion estimation, publishing the resulting `nav_msgs/msg/Odometry`.
* **`streaming_node`**: Handles real-time video streaming over HTTP (using `cpp-httplib`), allowing remote monitoring of the robot's camera feed with minimal latency.
* **`photographer`**: A utility node responsible for capturing high-resolution still frames from the continuous video stream based on service calls or specific triggers.
* **`mapper`**: Processes visual data for environmental mapping/logging purposes (currently focused on 2D space representation).

## 🚀 Build Instructions

### Prerequisites
* ROS2 (Humble / Iron)
* C++17 Compiler
* OpenCV (for visual odometry feature extraction)

### Building the Package
Clone the repository into your ROS2 workspace `src` directory and build using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_robot_localization --symlink-install
source install/setup.bash
