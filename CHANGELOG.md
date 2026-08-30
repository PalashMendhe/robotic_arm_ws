# Changelog

All notable changes to this project are documented here.

---

## [Unreleased] — Production Hardening Sprint 

### Architecture & Parameterization
**Goal**: Single source of truth for all configuration — no hardcoded values.

#### Added
- `src/arm_moveit_config/config/robot_params.yaml` — master config for joint names, home position, workspace bounds, motion scaling, and simulation toggle
- `src/arm_moveit_config/config/kinematics_real.yaml` — KDL kinematics config stub for real hardware
- `src/arm_moveit_config/launch/ur5_real.launch.py` — hardware bringup launch stub (no driver yet)

#### Changed
- `pick_and_place.py` — refactored all hardcoded coordinates and joint names to load from `robot_params.yaml`
- `planning_scene_manager.py` — refactored all object dimensions and positions to load from `robot_params.yaml`
- `kinematics.yaml` — switched solver from `lma_kinematics_plugin` → `kdl_kinematics_plugin` (lma not available in ROS Lyrical); increased `kinematics_solver_timeout` from 0.05s → 1.0s; increased `kinematics_solver_attempts` from 3 → 10

---

### IK & Motion Quality
**Goal**: Replace fragile hand-rolled IK with validated analytical solution.

#### Changed
- `pick_and_place.py` — restored `_compute_ik()` as a proper class method with full type annotations. All 11 motion steps now use `move_to_joints(self._compute_ik(x, y, z))` for reliable Cartesian positioning
- `robot_params.yaml` — updated all 6 arm joint limits from mixed values (e.g. `[-1.57, 0.5]`) to `[-3.14, 3.14]` to match the URDF exactly, fixing silent planning failures

#### Deferred
- TracIK plugin — not available in ROS Lyrical apt repository; KDL with analytical IK is functionally equivalent for this arm

---

### Error Handling & Safety Layer
**Goal**: Code fails loudly and cleanly instead of silently continuing.

#### Added
- `_is_within_workspace()` — validates Cartesian coordinates against YAML bounds before any goal is sent
- `_execute_move_group_goal()` — centralised MoveGroup goal dispatch with proper error code checking
- `preflight_check()` — verifies MoveGroup server, gripper server, and `/joint_states` topic before mission start
- `_abort()` — on any step failure: attempts emergency home pose return, then raises `RuntimeError` to halt mission
- `run()` — all 11 motion steps individually guarded; any failure triggers `_abort()`

#### Fixed
- `get_logger().warn()` → `get_logger().warning()` — `.warn()` is deprecated in ROS Lyrical rclpy

---

### Testing Infrastructure
**Goal**: Automated tests so future changes don't break things silently.

#### Added
- `src/robotic_4dof_arm/test/conftest.py` — disables ament lint plugins that crash on Python 3.14.4
- `src/robotic_4dof_arm/test/test_params.py` — 6 unit tests validating `robot_params.yaml` schema (joint limits, workspace bounds, motion scaling)
- `src/robotic_4dof_arm/test/test_scene_manager.py` — 5 unit tests for `PlanningSceneManager` (mock node, add/remove/attach/detach)
- `src/robotic_4dof_arm/CMakeLists.txt` — `BUILD_TESTING` block with `ament_add_pytest_test` and `ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`

#### Result
- `colcon test`: **13 tests, 0 errors, 0 failures** ✅

---

### Docker & Reproducibility
**Goal**: Anyone on any machine can run the simulation with one command.

#### Added
- `Dockerfile` — based on `osrf/ros:lyrical-desktop`; installs `ros2-control`, `ros2-controllers`, `gz-ros2-control`, `ros-gz-sim`, `ros-gz-bridge`, MoveIt; builds workspace at image build time
- `docker-compose.yml` — `sim` service (Gazebo + MoveIt) and `run` service (pick-and-place node) on an isolated Docker bridge network
- `Makefile` — `make build`, `make sim`, `make run`, `make test`, `make clean`, `make docker-build`, `make docker-sim`, `make docker-run`
- `.gitignore` — added `*.log` to exclude runtime log files

#### Fixed
- `docker-compose.yml` — removed `network_mode: host`; FastDDS was advertising `/joint_states` on the wrong host interface, causing the preflight check to fail inside Docker
- `docker-compose.yml` — added `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/lyrical/lib` so Gazebo can find the `gz_ros2_control-system` shared library
- `gazebo.launch.py` — fixed ROS-Gazebo clock bridge argument from `[gz.msgs.Clock` → `@gz.msgs.Clock`
- `Makefile` `clean` target — added `pkill` commands to kill zombie ROS 2 and Gazebo background processes, preventing the "time jumping backwards" and "duplicate action server" issues
- `src/robotic_4dof_arm/package.xml` — added missing `exec_depend` entries for `ros2_control`, `ros2_controllers`, `gz_ros2_control`, and all simulation packages so `rosdep` installs them correctly in Docker

---

### CI Pipeline & Code Hygiene
**Goal**: Every `git push` validates the build automatically.

#### Added
- `.github/workflows/ci.yml` — two-job pipeline:
  - **Job 1 (lint)**: Runs `ruff` + `yamllint` in ~30s without Docker build
  - **Job 2 (build-and-test)**: Builds Docker image, runs `colcon test` inside container; only triggers if lint passes
- `.pre-commit-config.yaml` — local git hook configuration: `ruff` (Python lint + auto-fix), `yamllint`, trailing whitespace/EOF/XML/YAML validators
- `ruff.toml` — ruff linter configuration; ignores E501 (line length for MoveIt chains) and N806 (uppercase IK variables like L1, L2, D)
- `.yamllint.yaml` — yamllint configuration; 120-char line limit with warning level

#### Fixed
- `pick_and_place.py` — moved `import math` from inside `_compute_ik()` to top-level imports; fixed one-liner `while` statements to proper block style; removed trailing whitespace
- `planning_scene_manager.py` — removed duplicate `box.dimensions` assignment (copy-paste bug); added missing blank lines before class and `main()` definitions (PEP8 E302); removed trailing whitespace

---

## What's Still Deferred (Needs Physical Hardware)

These items require a real UR5 robot connected over Ethernet:

| Item | Notes |
|---|---|
| `ur_robot_driver` bringup | Needs IP address of real controller |
| Real hardware controller YAML | `moveit_controllers_real.yaml` stub exists; needs driver-specific action names |
| Physical gripper driver | Depends on which gripper hardware is attached |
| First motion test | At 5% velocity scaling with E-stop armed |
| Safety validation | E-stop, joint limit enforcement, collision padding tuning |
| `kinematics_real.yaml` | Stub exists; validate IK accuracy on hardware |

---

## Known Limitations (Simulation)

- `position_only_ik: True` is set in `kinematics.yaml` — KDL ignores end-effector orientation. Sufficient for the current pick-and-place task but should be removed once orientation-constrained motions are needed.
- The analytical IK in `_compute_ik()` assumes a fixed wrist/TCP orientation. Generalised orientation control requires the `move_to_pose()` path with full Cartesian constraints.
