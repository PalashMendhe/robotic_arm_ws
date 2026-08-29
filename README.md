# Robotic 6-DOF Arm (UR5 Replica)

## Summary
This workspace contains a custom 6-Degree-of-Freedom (DOF) robotic arm, structurally inspired by the industrial UR5 robot. The goal of this project is to build an accessible, simulated manipulator that captures the core kinematics and aesthetic of a UR5-style robot while being streamlined for easier simulation, control, and motion planning. It serves as an excellent testbed for experimenting with pick-and-place tasks, obstacle avoidance, and robotic manipulation using ROS 2 and MoveIt 2.

## Demo Video and GIF
![Demo Video](Media/ur5_replica_demo_gif.gif)

## System Overview
The system is built on **ROS 2** and uses **MoveIt 2** for motion planning and execution. The physical attributes, visual meshes, and collision models of the arm are defined using URDF/Xacro. The robot is simulated in Gazebo, which provides realistic physics and joint feedback. 

MoveIt 2 handles the complex tasks of inverse kinematics (IK), collision checking, and generating valid trajectories avoiding obstacles. High-level commands and environmental setups (like adding tables and collision boxes) are managed through custom Python ROS 2 nodes that interface with MoveIt's planning scene.

## Packages and Directories

| Directory / Package | Description |
| :--- | :--- |
| `robotic_4dof_arm` | The core ROS 2 package. Contains the URDF/Xacro robot description, Gazebo simulation launch files, custom Python nodes (pick-and-place, scene management), and 3D meshes. |
| `arm_moveit_config` | The MoveIt 2 configuration package. Contains the SRDF (`arm.srdf`), joint limits, kinematics settings, and MoveIt-specific launch files. |
| `.../robotic_4dof_arm/urdf/` | Contains the `.urdf.xacro` files that assemble the robot's links, joints, and visual/collision geometries. |
| `.../robotic_4dof_arm/scripts/` | Python executables like `pick_and_place.py` and `planning_scene_manager.py` that use `rclpy` to command the arm and manipulate the environment. |
| `.../arm_moveit_config/config/` | YAML configuration files for the controllers and the SRDF file which defines the planning groups and the Allowed Collision Matrix (ACM). |

## Tech Stack
- **ROS 2** (Middleware and Node architecture)
- **MoveIt 2** (Motion planning and collision checking)
- **Gazebo** (Physics simulation)
- **Python / rclpy** (Control scripts)
- **URDF / Xacro** (Robot modeling)
- **CMake & Colcon** (Build system)

## Quick Start
*Note: Make sure your ROS 2 environment is sourced (e.g., `source /opt/ros/humble/setup.bash`).*

1. **Build the workspace:**
   ```bash
   cd ~/robotic_arm
   colcon build
   source install/setup.bash
   ```
2. **Clone the Repo**
   ```bash
   git clone https://github.com/PalashMendhe/robotic_arm_ws
   ```
3. **Launch the Simulation**
   ```bash
   ros2 launch arm_moveit_config moveit.launch.py
   ```
4. **Run the pick_and_pace node**
   ```bash
   ros2 run robotic_4dof_arm pick_and_place.py
   ```

## Bug Fixed
- **Self-Collisions:** Corrected the Allowed Collision Matrix (ACM) in `arm.srdf` to disable collision checking between adjacent links, which was causing immediate planning failures.
- **Planning Scene Updates:** Resolved issues with publishing collision objects (like the table and obstacles) correctly to the MoveIt planning scene via Python scripts.
- **URDF/SRDF Synchronization:** Fixed discrepancies between the URDF joint definitions and the SRDF planning groups to ensure stable trajectory generation.
- **Message Types & Imports:** Fixed ROS 2 Python imports and adapted ROS 1 `moveit_commander` logic to use ROS 2 `rclpy` compatible interfaces for scene management.

