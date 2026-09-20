# Changelog

All notable changes to this project are documented here.

---

## [Unreleased] — Vision Sorting Bugfix Sprint

### Fixed — all three vision-sorting failure modes + TF chain
**Goal**: Every spawned object pickable, every pick at the right pose, every object sorted, no wasted motion.

#### Fixed — Bug 1: picks always go to the same stretched pose
- `pick_and_place.py` — `_compute_ik()` no longer clamps `acos`/`asin` on unreachable targets; it returns `None` instead. Added `is_reachable(x, y, z, margin)` envelope check (wrist-offset + `L1+L2` bounds), `_ik_or_abort()` wrapper that aborts loudly, and a `None` guard in `move_to_joints()`. All 7 demo call sites switched to `_ik_or_abort`.
- `sorting_controller.py` — `_object_reachable()` verifies grasp, pre-pick AND lift waypoints (with `REACH_MARGIN_M = 0.02`) before any object becomes a candidate; unreachable detections are skipped with a warning, never picked.
- `camera_tf_broadcaster.py` — publishes the full TF chain `world → d415_camera_link → d415_color_optical_frame` (standard `rpy = [-90°, 0, -90°]` optical rotation). Previously only the mount pose was published, so back-projected world coordinates were systematically offset.
- `camera_params.yaml` — documents the two-step TF chain and its SDF-pose constraint.
- `vision_sim.launch.py` — spawner constrained to the arm's true reach envelope: `x` sampled inside `[-sqrt(R_PICK_MAX² − y²), -0.63]` with `R_PICK_MAX = 0.78 m` (2 cm inside the IK limit at grasp height); deterministic reachable fallback layout replaces the old `RuntimeError` when random sampling cannot fit all objects.
- `robot_params.yaml` — sort zones trimmed from 5 to 3 slots per class (matches `N_CUBES = N_CYLINDERS = 3` spawner capacity and `MAX_SLOTS = 3`); place-height comments now show the derived `arm_z` values.

#### Fixed — Bug 2: bot sorts 2 of 3+3 (assumed object count)
- `sorting_controller.py` — perception-driven mission loop: `home → [scan → pick → lift → traverse → place → retreat] × N → home`. The count is never assumed; the mission ends only after `EMPTY_SCANS_TO_FINISH = 2` consecutive empty scans.
- Added cross-scan object registry (`_update_registry`, `MATCH_RADIUS_M = 0.07`) that tracks unknown objects across scans, refreshes positions on re-detection, returns candidates nearest-first, and excludes `done`/`blacklisted` entries.
- Added Table 2 pick-region filter (`_on_pick_table`) so already-placed sort-zone objects on Table 1 are never re-picked during re-scans.
- Zone slots assigned by per-class placement counter (`_next_zone` / `_mark_zone_used` / `_slot_index`), not detection order; retry limit (`MAX_ATTEMPTS_PER_OBJECT = 2`) with blacklist prevents infinite loops on failing objects.

#### Fixed — Bug 3: wasteful home round-trips between pick and place
- `sorting_controller.py` — `_pick_and_place(entry, zone_x, zone_y, label)` transport split into lift → traverse legs (pre-pick → lift → traverse → pre-place → place → release → retreat) so the planner cannot fold back through home; failures retry from the lifted pose via `_set_down_at` / `_retreat` instead of homing mid-swing. Home is visited exactly twice per mission (start + end).

#### Added — tests
- `test/test_vision_sorting.py` — 27 unit tests for the pure mission logic (IK reachability incl. loud `None` on the old far corner, pick-region filter, grasp+lift waypoint check, registry nearest-first/refresh/dedup/skip done+blacklisted, zone-slot ordering/exhaustion, spawner reach envelope + `MIN_SEP` + fallback layout).
- `CMakeLists.txt` — registered `test_vision_sorting` with `ament_add_pytest_test` (same `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` env as existing tests).

#### Fixed — live-run hotfix: every motion answered FAILURE(99999)
First live run after the sprint aborted with `Motion failed [code 99999]: Joint-space` on every goal (including the trivial "returning home") plus `Ignoring unexpected result response … more than one action server for '/move_action'` — two stacked root causes:
- `pick_and_place.py` — IK base joint: the `d4` wrist offset gives TWO mirrored branches (`a0 ± asin(d4/R)`); the code always took `+`, and the pick at `(-0.676, 0.105)` produced `theta1 = 3.14817`, above the URDF `arm_base_joint` limit (±3.14) — an unplannable goal. `_compute_ik()` now wraps every angle into `[-π, π)`, evaluates both branches, and picks the valid one with the smaller base swing (2.8276 rad here); targets with no in-limit branch are refused loudly.
- `pick_and_place.py` — duplicate MoveGroup detection: `_move_group_server_count()` counts publishers on `/move_action/_action/feedback` (one per live move_group); preflight refuses to start with a `make vision-clean` hint when more than one server owns the action name — the exact cause of the stale-server FAILURE(99999) storm and the "unexpected result response" warnings.
- `pick_and_place.py` — preflight also waits for `/arm_controller/follow_joint_trajectory` (a missing arm controller previously surfaced only as mid-mission motion failures); MoveGroup results get a 60 s timeout with goal cancel instead of hanging forever.
- `pick_and_place.py` — motion failures log the symbolic MoveIt error name (`FAILURE(99999)`, `PLANNING_FAILED(10001)`, `CONTROL_FAILED(10004)`, …) plus a diagnostic hint for `FAILURE`; `move_gripper()` now checks goal acceptance and result code instead of failing silently.
- `sorting_controller.py` — circuit breaker: 3 consecutive failed sort cycles abort the mission early (healthy MoveIt + healthy perception never produce that pattern) instead of scanning to the end and blacklisting every object; the final home move reports failure instead of swallowing it.
- `Makefile` — new `vision-clean` target kills stale `move_group` / Gazebo / controller processes without wiping the build (the "duplicate action server" mode was seen before — see the Docker sprint notes in `make clean`).
- `test/test_vision_sorting.py` — 6 new regression tests (the live-failure coordinates, mirrored-branch selection, angle wrap, spawn-band limit sweep, duplicate-server hint, MoveIt error-code names) → 33 total.

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
