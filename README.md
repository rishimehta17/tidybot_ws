# TidyBot Home-Tidying Simulation

## Overview

This project implements a complete ROS 2 + Gazebo simulation of a home-tidying robot for the Drift internship assignment. The system includes:

- a custom robot model with a mobile base, torso, 3-DoF dual-arm, camera, and additional lidar sensor,
- a two-room home world with furniture, pickup objects, and a collection area,
- autonomous waypoint-based navigation launched with a single command,
- basic arm-action sequencing at selected waypoints,
- camera-based color-object detection for simulation testing.

The goal of the project is not advanced perception or optimal control. The focus is a stable, runnable, integrated simulation that launches cleanly and demonstrates meaningful robot behavior, which matches the assignment emphasis.

## Assignment Coverage

This repository addresses the main requirements from the assignment:

- custom ROS 2 robot model with a four-wheel base, torso, visible dual-arm structure, and 2 functional 3-DOF arm,
- home environment with two connected rooms, doorway, walls, furniture, scattered objects, and a collection box,
- autonomous navigation through both rooms without manual teleoperation,
- sensor publishing during runtime,
- measurable runtime output through odometry/path logging in the terminal,
- optional manipulation behavior through waypoint-triggered arm actions.

## Repository Structure

A typical package layout for this project is:

```text
src/
└── drift_tidybot/
    ├── launch/
    │   └── tidybot.launch.py
    ├── urdf/
    │   └── fin.urdf
    ├── worlds/
    │   └── tidybot_home.world
    ├── drift_tidybot/
    │   ├── arm.py
    │   ├── camera.py
    │   ├── navigate.py
    ├── rviz/
    │   └── config.rviz
    ├── package.xml
    ├── CMakeLists.txt
```

## Requirements

- Ubuntu 22.04 or 24.04
- ROS 2 Humble
- Gazebo Classic 11
- colcon build system
- Python packages used in the nodes:
  - `rclpy`
  - `opencv-python`
  - `numpy`
  - `cv_bridge`
  - `tf_transformations`

#!/bin/bash

# ROS dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-tf-transformations \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-image-transport \
  ros-humble-std-msgs
```

# Python dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
## Build Instructions

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

Python scripts, make sure they are executable:

```bash
chmod +x src/drift_tidybot/drift_tidybot/arm.py
chmod +x src/drift_tidybot/drift_tidybot/camera.py
chmod +x src/drift_tidybot/drift_tidybot/navigate.py
```

## Launch Instructions

To run the full simulation stack from a single command:

```bash
ros2 launch drift_tidybot tidybot.launch.py
```

This launch file is designed to:

1. start Gazebo with the custom home world,
2. publish the robot description,
3. spawn the robot into the world,
4. start the required controllers,
5. launch RViz,
6. launch the navigation node after the robot and controllers are active,
7. launch the camera test node after the simulation is running.


## Main Runtime Nodes

### `navigate.py`

Autonomous waypoint navigator.

- subscribes to `/odom`
- publishes `/cmd_vel`
- publishes arm commands to `/arm_controller/commands`
- prints elapsed time, position, yaw, distance to waypoint, total path length, and visited rooms
- supports waypoint-triggered arm actions

### `camera.py`

Color-based vision test node.

- subscribes to `/camera/image_raw`
- detects blue, green, and yellow objects in HSV space
- displays annotated image and combined mask for debugging

### `arm.py`

Manual GUI controller for robot movement and arm testing.

- publishes drive commands to `/cmd_vel`
- publishes arm joint commands to `/arm_controller/commands`
- useful for controller verification and debugging outside the autonomous run

## Sensors and Topics

The simulation publishes sensor and motion-related data during runtime. Topics include:

- `/camera/image_raw`
- `/odom`
- `/cmd_vel`
- `/arm_controller/commands`



## Navigation and Task Behavior

The robot follows a hard-coded waypoint path through both rooms. The navigation strategy intentionally prioritizes robustness and repeatability over algorithmic sophistication. 

Selected waypoints can trigger arm actions. For example, a waypoint can execute a sequence such as:

- pick pose,
- lift pose,
- stow pose.


## Known Limitations

- Navigation is waypoint-based, not map-based.
- Perception is not fused into navigation.
- Manipulation is scripted rather than contact-rich grasp planning.
- Some waypoint tuning is specific to the observed motion characteristics in simulation.


## Debugging Notes

During development, the main issues were:

- waypoint orbiting and oscillation near goals,
- controller startup timing,
- mismatch between expected and observed robot motion behavior,
- URDF parsing and joint/frame consistency issues.


## Contact

Author: Rishi Mehta
Email: rishi17@umd.edu
