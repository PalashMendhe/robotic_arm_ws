# Robotic 6-DOF Arm (UR5 Replica)

![CI Status](https://github.com/PalashMendhe/robotic_arm_ws/actions/workflows/ci.yml/badge.svg)

## Summary
This workspace contains a custom 6-Degree-of-Freedom (DOF) robotic arm, structurally inspired by the industrial UR5 robot. 

This project serves as a **production-grade testbed** for experimenting with pick-and-place tasks, obstacle avoidance, and robotic manipulation using ROS 2 Lyrical, MoveIt 2, and Gazebo Harmonic. It features a fully dockerized environment, dynamic YAML parameterization, robust error handling, and automated CI/CD testing.

## Demo Video and GIF
![Demo Video](Media/ur5_replica_demo_gif.gif)

## Key Features
- **Production-Ready Docker Environment**: Launch the entire simulation stack instantly with zero ROS installation required, eliminating "works on my machine" bugs.
- **Robust Error Handling**: Real-time preflight checks, workspace bounds validation, and emergency abort routines ensure safe operation.
- **Dynamic Parameterization**: A single source of truth (`robot_params.yaml`) governs Cartesian coordinates, joint limits, velocities, and dimensions for rapid iteration without code changes.
- **Automated Testing & CI/CD**: A comprehensive `pytest` suite runs automatically on GitHub Actions on every push to guarantee motion reliability.
- **KDL Inverse Kinematics**: Customized IK solver configured specifically for this 6-DOF architecture.

## Quick Start (Docker - Recommended)

The easiest way to run the simulation is using our pre-configured Docker environment.

1. **Build the Environment:**
   ```bash
   make docker-build
   ```
2. **Launch the Simulation:**
   ```bash
   make docker-sim
   ```
   *(Gazebo and MoveIt will launch. Wait for the arm and table to spawn).*

3. **Run the Autonomous Routine:**
   Open a second terminal and run:
   ```bash
   make docker-run
   ```

## Quick Start (Local)

If you have ROS 2 Lyrical installed locally:

1. **Build the workspace:**
   ```bash
   make build
   ```
2. **Launch the Simulation:**
   ```bash
   make sim
   ```
3. **Run the Autonomous Routine:**
   Open a second terminal and run:
   ```bash
   make run
   ```

## Packages and Architecture

| Directory / Package | Description |
| :--- | :--- |
| `robotic_4dof_arm` | The core ROS 2 package containing URDF/Xacro models, Python control scripts, and the `pytest` suite. |
| `arm_moveit_config` | MoveIt 2 configuration, including SRDF, kinematics settings, and the master `robot_params.yaml`. |
| `Dockerfile` | Builds an isolated Ubuntu environment with all required dependencies and control libraries. |
| `docker-compose.yml` | Maps display sockets and bridges isolated networks for seamless Gazebo UI rendering. |

## Testing

To run the automated test suite locally:
```bash
make test
```
The suite verifies parameter consistency, planning scene integrity, and environment dimensions.
