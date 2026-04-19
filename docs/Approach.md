# Home Tidying Robot Simulation

## Overview

I implemented a complete **ROS 2 + Gazebo simulation** of a home-tidying robot. The system includes a custom robot model designed from scratch in Solidworks, a two-room home environment, autonomous navigation, and waypoint-triggered manipulation.

My primary goal was to build a **stable, fully integrated simulation pipeline** that launches reliably and performs a meaningful task end-to-end.

---

## 1. Robot Model Design

### Structure

The robot was built in Solidworksd and exported as a URDF model with the following components:

- **4-wheel skid-steer base**
  - All wheels are actuated
  - Sized to pass through doorways

- **Torso**
  - Vertical T-shaped structure mounted on rectangular robot base

- **Arms**
  - Left arm: 3 DOF (shoulder, elbow, wrist)
  - Right arm: similar as the left one

- **End Effector**
  - Simple gripper

- **Head Panel**
  - Flat rectangular panel representing LED face

---

### Sensors

- **Camera**
  - Mounted on torso
  - Publishes `/camera/image_raw`


- **Lidar**
  - Mounted on top of the head panel
  - Publishes on `/scan`

---

### Physical Properties

Significant effort was spent ensuring simulation stability:

- Realistic **mass and inertia values**
- Proper **joint limits, damping, and friction**
- Valid **collision geometries**

---

## 2. Home Environment Design

The environment is implemented as a Gazebo world file.

### Layout

- Two rooms connected by a doorway
- Fully enclosed walls with collision

---

### Furniture and Objects

Each room contains:

- Table
- Chairs as cubes
- Additional structure (shelf/cabinet) 
- couch

Objects:

- Multiple small items scattered across both rooms
- Represent pickup targets

---

### Collection Box

- Located near robot start position
- Serves as drop-off location

---

### Design Considerations

- Furniture placement constrains navigation paths
- Waypoints must account for obstacles

---

## 3. Navigation Strategy

### Approach

Navigation is implemented using:

- Odometry → position (x, y)
- IMU → orientation (yaw)

The robot follows a **waypoint-based navigation system**.

---

### Controller Design

- Linear PID → distance to waypoint
- Angular PID → heading error

---

### Key Challenges and Fixes

#### Orbiting Around Waypoints

- Problem: robot circled goals without converging
- Fix: disabled forward motion when close and misaligned

---

#### Oscillation Near Goal

- Problem: forward/backward oscillation near waypoint
- Fix:
  - Introduced deadband
  - Increased tolerance from 0.2 → 0.3

---

#### Control Mapping Issue

- Robot had non-standard mapping because of design issue and urdf export settings:
  - `angular.z` → forward motion
  - `linear.x` → turning

- Identified using manual `/cmd_vel` testing and corrected

---

#### Drift Compensation

- Robot drifted right during motion
- Waypoints adjusted to be slightly left-biased

---

## 4. Pick-and-Place Behavior

Pick-and-place actions are triggered at specific waypoints.

Example:

```python
(1.0, 1.0, "Room1 – west side", "PICK_UP_STOW")
```

At this waypoint:

1. Arm moves down (PICK)
2. Arm lifts (UP)
3. Arm returns to safe pose (STOW)

This approach uses **predefined positions** rather than perception.

---

## 5. System Integration

A single launch file is used to start the entire system:

1. Gazebo world
2. Robot spawn
3. Controllers
4. Navigation node
5. Camera node

---

### Synchronization

- Delayed startup ensures controllers are active before navigation begins
- Prevents command loss and unstable behavior

---

## 6. Debugging Challenges

### URDF Issues

- Duplicate link names → fixed
- joint angles orientation issues were faced.

---

### Gazebo Issues

- Robot did not spwan because of errors in urdf files. 
- Joints were defined with 0 0 0 origins as revolute joints causing this failure.

---

### Navigation Issues

- Orbiting → fixed with turn-only zone
- Oscillation → fixed with deadband
- Unreachable waypoints → adjusted based on actual robot path

---

### Control Debugging

- Verified motion using manual `/cmd_vel` commands
- Identified non-standard control mapping

---

## 7. Tradeoffs

- Used waypoint navigation instead of SLAM
- Used known object positions instead of perception
- Simplified gripper design

These choices prioritized **system reliability and simplicity**.

---

## 8. External Assets Used

- Gazebo default models (tables, chairs)
- No external robot URDFs were copied

---
## My Build vs Drift’s Build

To evaluate AI-generated simulation quality, I used the Drift CLI to generate a robot and environment from the same specification.

---

### Drift Output (Observed Behavior)

Drift successfully generated an initial ROS 2 package structure, including:

* A URDF/Xacro robot model with a 4-wheel base and 3-DOF arm
* A launch file and configuration files that prints hello world the talker script.
* Img of generated URDF is attached in the docs folder.
* Img of Drift output is also attached.
However, several critical components were incomplete or missing:
I had to build the workspace manually.

#### 1. Missing Environment

* The home world (two rooms, furniture, objects) was not generated at all
* This makes the system incomplete for the assignment task

#### 2. No End-to-End Execution

* The workspace was not built (`colcon build` not executed)
* The simulation was never launched
* No verification that the robot actually runs in Gazebo

#### 3. Lack of Integration

* Controllers were not fully configured or validated
* No navigation or behavior pipeline was implemented
* No pick-and-place logic was present

#### 4. Unverified Robot Model

* URDF was generated but:

  * No confirmation of physical stability
  * No testing for collisions or dynamics
  * No debugging of TF or joint issues

---

### My Build (Improvements)

In contrast, my implementation focuses on full system functionality and robustness:

#### 1. Complete Environment

Designed a two-room home with:

* Walls, doorway, and constrained paths
* Furniture (table, chairs, structures)
* Scattered objects
* Collection box

#### 2. Fully Working Simulation

Successfully built and launched using:

```bash
ros2 launch drift_tidybot tidybot.launch.py
```

* Robot spawns correctly and interacts with the environment

#### 3. Stable Robot Model

Fixed URDF issues such as:

* duplicate link names

Additionally:

* Tuned damping and friction for stability

#### 4. Autonomous Navigation

Implemented waypoint-based navigation and solved:

* orbiting around waypoints
* oscillation near goals
* drift in motion

#### 5. Integrated System Pipeline

A single launch file initializes:

* Gazebo
* robot spawn
* controllers
* navigation
* camera node

#### 6. Functional Task Execution

The robot:

* navigates both rooms
* avoids obstacles
* executes pick-and-place actions at waypoints
* produces measurable logs (odometry, distance, rooms visited)

---

### Key Insight

Drift generates a structural starting point, but it does not produce a working system.

The missing elements include:

* environment construction
* simulation validation
* controller integration
* debugging and refinement

These require engineering judgment, iteration, and testing, which were essential in building a complete solution.

---

### Conclusion

While Drift accelerates initial setup, my implementation delivers a fully functional, stable, and integrated simulation, meeting all assignment requirements and exceeding the capabilities of the current AI-generated output.
