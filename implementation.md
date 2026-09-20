# implementation.md — Vision sorting rework (counts, camera mounting, grasp verification, alignment)

**Status:** W1 (gripper fix) + W5 (IK/constants) + 2+2 scene — **implemented and
verified**. implementation.md created before any code change, per instruction.
**Package(s):** `robotic_4dof_arm`, `arm_moveit_config`
**Date:** 2026-09-20

---

## 0. Your four points, and the verdict after measurement

| # | Your idea | Verdict after stress-testing against this repo |
|---|---|---|
| 1 | Reduce objects to **2 cubes + 2 cylinders** | ✅ **Do it.** Less slot pressure (`MAX_SLOTS`), fewer chances to miscount, faster mission, cheaper to validate. Touches 6 files (see §2). |
| 2a | Fixed **top-down camera, ~1.50 height** | ⚠️ Right idea, wrong number: at world z = 1.50 the frame covers only **0.571 × 0.762 m**, but the spawn band is **0.60 × 0.80 m** → objects at the band edge get clipped (that is exactly today's failure mode). Needs world z ≈ **1.80–2.00** (= 0.85–1.00 m above the table). All numbers in §3.2. |
| 2b | **Wrist-mounted camera**, resolve the offset | ✅ Feasible *because of a property I verified*: at every IK-solved pose the tool axis is **exactly vertical** (`tool_z = (0,0,-1)`), so a wrist camera sees a **nadir** view by construction. But it is **worse for counting** (partial views → needs a multi-pose scan). Recommendation in §3.4: wrist camera for **picking/alignment**, and (optionally) a fixed camera for the **census**; the count then becomes a cross-check rather than a guess. |
| 3a | **Contact sensors** on the jaws | ✅ Most trustworthy signal in Gazebo. Exact insertion points known (`arm.urdf.xacro:429-445`). One bridging uncertainty to verify first (E3). |
| 3b | **Jaw position + velocity** check (v = 0 and gap = object size) | ⚠️ Works in principle, and I quantified the signal: with today's `GRIPPER_CLOSE = ±0.028` the difference between "grasped" and "closed on air" is only **4.5 mm (5.6 % of travel)** → too fragile. With `GRIPPER_CLOSE = ±0.012` it becomes **20.5 mm (25.6 %)** → robust. Velocity alone is **not** a discriminator (both cases end at v ≈ 0); it is the *combination* that works (your formula, with corrected thresholds). §4. |
| 4 | Align the **middle of the gripper with the middle of the object**, then pick | ✅ Your instinct is correct and it is **not** optional — but today it fails for **three independent reasons** I measured: **(A)** the analytical IK picks the wrong base-joint branch for every object with **y > 0**, putting the gripper **218 mm (= 2·d4)** away from the target; **(B)** the IK targets the TCP, but the *jaws* are **57 mm** above the TCP → only **43 %** of the object is engaged; **(C)** the jaw yaw sits **7°–25°** off the nearest cube face → an edge grasp, and the open gap (115 mm) cannot clear a 45°-rotated cube (diagonal 141 mm). All three are fixable, and **(C) is free** (verified: a wrist-roll change yaws the jaws with **0.00 mm** TCP shift). §5. |

---

## 0.1 Run these three experiments FIRST (cheap, and they unblock the design)

**E1 — Is the prong position command kinematic or force-limited?**
`gripper_controller` is a `JointTrajectoryController` with `command_interfaces: [position]`
(`config/controllers.yaml:29-38`) on top of `gz_ros2_control/GazeboSimSystem`. If Gazebo realises a
position command *kinematically*, the reported joint position always equals the command and every
jaw-position grasp check (your 3b) is meaningless; if it is force/PID limited (prong effort is 10 N,
contact `kp = 1e6`, `minDepth = 0.001`), the jaws stall on the object and the check works.
*Test:* publish a close trajectory with a cube between the jaws and with empty jaws; compare the
measured `right_prong_joint` position from `/joint_states` to the command. If they are identical in
both cases → switch the prong command interface to `effort` (with a PID in `controllers.yaml`) **or**
rely on contact sensors (3a).

**E2 — The grasp signal budget.** With a 0.1 m object the right prong must read **0.0325** (jaw face
at ±0.05); the "closed on air" value equals the command. Today `GRIPPER_CLOSE = ±0.028` → gap 0.091 →
signal **4.5 mm**. Proposed `±0.012` → gap 0.061 → signal **20.5 mm**. Measure both, then set the
threshold from data.

**E3 — Contact-sensor bridging.** Confirm the installed `ros_gz_bridge` maps the Gazebo contact
sensor topics to ROS 2. If it does not, add a tiny gz→ROS relay node, or ship 3b only.

---

## 1. Evidence — why the counts are wrong today (measured, not assumed)

### 1.1 The class is decided by *circularity only*; colour is never used
`shape_detector.find_contours()` (`scripts/shape_detector.py:119-140`) computes
`circularity = 4πA/P²` on the Canny contour and compares against `circularity_threshold: 0.88`.
`grep -r "inRange|HSV|mask"` over `src/` and `vision_pipeline/` returns **nothing**.
I re-ran this repo's exact pipeline (`GaussianBlur(5,5)` → `Canny(40,120)` → `findContours(RETR_EXTERNAL)`
→ `contourArea`/`arcLength`) on synthetic silhouettes at the simulated pixel scale:

| silhouette / view | measured circularity | label produced (>0.88 ⇒ cylinder) |
|---|---|---|
| red cube, square face (near-nadir) | 0.794 | `red_cube` ✅ |
| red cube, top face + front strip — **the real ~61° view** | 0.647 | `red_cube` ✅ |
| blue cylinder, near-nadir circle, 100 px | 0.897 | `blue_cylinder` ✅ (+0.017 only) |
| blue cylinder, circle, 40 px | 0.880 | flips to `red_cube` ❌ |
| blue cylinder, capsule cap-aspect 0.75 (far object) | 0.782 | `red_cube` ❌ |
| blue cylinder, capsule cap-aspect 0.45 (near object) | 0.800 | `red_cube` ❌ |
| regular hexagon (analytic, 45° cube view) | 0.907 | would be `blue_cylinder` ❌ |

Camera at `(-0.45, 0, 1.45)`, `rpy (0, 0.785, π)` sees the objects (z = 1.00) **42°–61° below
horizontal**, so a cylinder never projects as a circle — it measures 0.78–0.80 and is published as a
`red_cube`. Even a perfect top-down circle only measures ≈0.89–0.90 (pixel-staircase perimeter
over-estimate). **The classes straddle the threshold; the counters cannot be right by construction.**

### 1.2 `Canny → RETR_EXTERNAL` is not object segmentation
`preprocess_image()` (`:112-116`) has **no `morphologyEx`/close**, so one object can yield several
disconnected edge loops (→ several `Detection2D`, over-count) and two objects 20 mm apart
(`MIN_SEP = 0.12 m`, `launch/vision_sim.launch.py:43`) can be bridged into one contour (under-count).
Fragmentation also distorts area/perimeter → class flips.

### 1.3 Framing and gates silently drop objects
* Projecting the spawn band through the current pose: `(-0.68, ±0.30) → u ≈ 31/609`,
  `(-0.78, ±0.38) → u ≈ 639` in a 640-wide image → **border clipping** → truncated silhouettes,
  wrong area/circularity, or area < `contour_area_min = 400` → object vanishes.
* `contour_area_max = 30000` is close to a near oblique silhouette (18–25 k px² measured).
* `get_valid_depth()` needs a valid 3×3 median **at the minAreaRect centre** (`:149-164`) — for an
  oblique cube that pixel can be off-object; depth noise is `stddev 0.007` (`vision_sorting.sdf:205`).
* `ApproximateTimeSynchronizer(slop=0.05)` drops colour/depth pairs → no temporal stabilisation.

### 1.4 The mission's printed count is *not* a detection count
`sorting_controller.py` reports `red_ok/blue_ok` = **successful placements** (`:395, :436-439, :472-475`),
capped by `MAX_SLOTS = 3` (`:83`, `_next_zone` `:249-261`): surplus objects are blacklisted with
*"All 3 slots … are full"* (`:420-426, :444-449`) and never picked. In-scan `DEDUP_RADIUS_M = 0.10`
merges only **same-class** points (`:163-183`) → mixed-label fragments of one object become two entries.

### 1.5 Dead / contradictory configuration
* `camera_params.yaml:26-34` declares `fx = fy = 602` (57.9° HFOV); the world uses
  `horizontal_fov = 1.2112 rad` = 69.4° ⇒ **fx ≈ 462** (`vision_sorting.sdf:189`). The node reads
  `/d415/camera_info`, so the YAML block is dead config — but thresholds tuned from it are ~30 % off.
* `robot_params.yaml:57-59` `vision.use_vision: false` is **read by nothing**.
* `robot_params.yaml:39-45` gripper limits (`right ∈ [-0.4, 0]`, `left ∈ [0, 0.4]`) contradict the URDF
  (`arm.urdf.xacro:334,361`: right `[0, 0.08]`, left `[-0.08, 0]`) — and the working
  `GRIPPER_CLOSE = [0.028, -0.028]` violates the YAML's own limits.
* `vision_sim.launch.py` never starts `shape_detector.py`, so a stale extra detector is easy to leave
  running (`ros2 node list` check belongs in the runbook).

---

## 1.6 Grasp geometry findings (FK of `arm.urdf.xacro` + the real IK formulas)

I rebuilt the URDF chain (xacro properties → joint origins → FK) and replayed
`PickAndPlace._compute_ik()` exactly as written, then measured where the TCP and the **jaws** end up.

### Finding A — the base-joint branch rule is wrong for the whole **+Y half** of the table

`_compute_ik()` evaluates `a0 ± asin(d4/R)` and keeps `min(|θ1|)`. Measured against the URDF chain:

| target (arm frame) | err with `a0 + asin(d4/R)` | err with `a0 − asin(d4/R)` | what `min(|θ1|)` picks |
|---|---|---|---|
| `(-0.70, 0.00, 0.05)` | 0.1 mm | 218.0 mm | 0.1 mm ✅ (branches mirror at y = 0) |
| `(-0.70, +0.10 … +0.38, 0.05)` | **0.1 mm** | 218.0 mm | **218.0 mm ❌** |
| `(-0.63, +0.10 … +0.38, 0.05)` | **0.1 mm** | 218.0 mm | **218.0 mm ❌** |
| `(-0.63 … -0.78, −0.10 … −0.38, 0.05)` | 0.1 mm | 218.0 mm | 0.1 mm ✅ |

**9 of 21 sampled in-reach pick targets** (every one with `y > 0` — i.e. half of all spawned objects)
are solved 218 mm = **2·d4** away from the object: the gripper closes on air no matter how good the
vision is. `+ asin(d4/R)` is correct in **all** sampled cases.

The existing test `test_vision_sorting.py::test_compute_ik_picks_smaller_swing_branch` asserts
`θ1 = 2.8275` for `(-0.676, 0.105)` — that is the **218 mm-off branch**; the correct branch is
`wrap(3.1475) = −3.1357` (same angle, |θ1| ≤ 3.14 ✅). The test therefore pins the bug and its premise
("smaller base swing") is invalid. **Fix:** `θ1 = wrap(a0 + asin(d4/R))`, reject only if
`|θ1| > JOINT_LIMIT`; rewrite the test to assert that the FK reproduces the requested TCP.

### Finding B — the IK targets the TCP, but the jaws sit 57 mm above it

From the URDF: prong collision boxes are `0.02 × 0.005 × 0.1` centred at `z = +0.018` in
`gripper_base_link` → the jaw span along the tool axis is **[-0.032, +0.068]** (centre **+0.018**),
while `gripper_base_to_tcp` puts the TCP at **+0.075** (`arm.urdf.xacro:368-373`).

| what the IK targets | jaw span vs a 0.1 m object | engaged | jaw-centre error |
|---|---|---|---|
| `TCP_Z_OFFSET = 0.170` (today, = `d5 + 0.075`) | `[-0.107, -0.007]` vs `[-0.050, +0.050]` | **43.0 mm of 100 mm** | **−0.057 m** |
| `TCP_Z_OFFSET = 0.113` (= `d5 + 0.018`) | `[-0.050, +0.050]` vs `[-0.050, +0.050]` | **100.0 mm** | 0.000 m |

Today's picks grip with only the lower 43 % of the jaw length, jaw centre 57 mm too high — a loose
grasp that slips during the lift. **Fix (least invasive):** parameterise the grasp reference as the
**jaw centre**: `TCP_Z_OFFSET := d5 + jaw_centre_offset = 0.095 + 0.018 = 0.113`, and change the MoveIt
attach offset in `planning_scene_manager.attach_target_box_to_gripper()` from `0.075` to `0.018` **in the
same commit** (otherwise the attached collision box and the real object disagree). `PICK_WORLD_Z = 1.00`
stays — the *jaw centre* is what must land on the object centre. *(Alternative: move the prong joint
origins from `z = 0.018` to `z = 0.075` in the URDF and keep the constants — bigger model change; §5.2.)*

### Finding C — the jaw yaw is 7°–25° off the cube faces, but yaw is **free**

Jaws close along the tool **y** axis; for an axis-aligned cube that is a "face grasp" only near ±x/±y:

| target | jaw yaw (world) | off the nearest face normal |
|---|---|---|
| `(-0.70, +0.20)` | −97.33° | 7.33° |
| `(-0.70, −0.20)` | −65.44° | 24.56° |
| `(-0.63, +0.30)` | −106.47° | 16.47° |

and a wrist-roll change moves the jaws **without moving the TCP at all**:

| extra wrist roll (joint 5) | TCP shift | tool axis | jaw yaw |
|---|---|---|---|
| +90° | **0.00 mm** | (0, 0, −1) | yaw + 90° ✅ |
| +45° | **0.00 mm** | (0, 0, −1) | yaw + 45° ✅ |
| joint 6 only, +90° | **0.00 mm** | (0, 0, −1) | yaw + 90° ✅ |

So **yaw alignment costs one added constant** (`θ5 += Δyaw`) and keeps the vertical approach — this is
what makes your point 4 cheap as well as necessary. (Cylinder: any yaw. Cube: Δyaw = nearest 90°
multiple − jaw_yaw, or use the object's own yaw from the vision `minAreaRect` angle.)

### Finding D — the open gap cannot clear a 45°-rotated cube

* Jaw gap: `OPEN ±0.04 → 0.115 m`, `CLOSE ±0.028 → 0.091 m`, `q = 0 → 0.035 m`.
* Object: cube face-to-face **0.100 m**, cube edge-to-edge (45°) **0.1414 m**, cylinder Ø **0.100 m**.
* `0.115 < 0.1414` → a cube rotated ~45° (very possible after a failed pick nudges it) is **jammed**
  by the jaws on the way down, or only corner-grasped.
**Fix:** open wider — gap ≥ **0.155 m** (`q = ±0.06`; travel allows ±0.08 → 0.195 m) — and let the grip
check distinguish a 100 mm object from the 141 mm corner case.

---

## 2. W1 — Reduce the scene to 2 cubes + 2 cylinders (your point 1)

| File | Change |
|---|---|
| `launch/vision_sim.launch.py:44-45` | `N_CUBES = N_CYLINDERS = 2`; update the reach/`MIN_SEP` comments and the `_fallback_positions()` guard (currently `n_total > 6`). |
| `sorting_controller.py:83` | `MAX_SLOTS = 3` → `2`. |
| `arm_moveit_config/config/robot_params.yaml:65-77` | `sort_zones.red` / `sort_zones.blue` → **2 slots each**. |
| `test/test_vision_sorting.py` | spawner tests assert 6 positions (`_generate_positions(6)`, `_fallback_positions(6)`, `test_fallback_rejects_impossible_counts`) → move to 4 and a smaller impossible count. |
| `test/conftest.py`, `test_params.py` | unaffected (pure-python; lint plugins are blocked in `conftest.py`). |

**Why it helps beyond "fewer objects":** with 2 per class the slot grid can never saturate under a
single misclassification, missions are 2× faster to iterate, and a count error becomes unambiguous
(2 + 2 = 4 → any extra/missing detection is immediately visible).
**Ground-truth hook (cheap, big payoff):** `generate_spawn_actions()` already knows the sampled
positions — write them to a known YAML (e.g. `~/.ros/vision_spawn_truth.yaml`) at launch so both the
detector's count and the arm's pick accuracy can be scored automatically instead of by eye.

---

## 3. W2 — Solving the count problem (your point 2)

### 3.1 What is actually broken (one paragraph)
The count is wrong because (i) the class is inferred from a circularity threshold that both classes
straddle (§1.1), (ii) "one Canny outline = one object" is false (§1.2), and (iii) objects at the frame
edge are clipped and dropped (§1.3). None of these is fixed by moving the camera alone — moving the
camera fixes the *view*; the *segmentation* must become colour-based and closed.

### 3.2 Detector rework (required for **both** camera options)
`scripts/shape_detector.py` + `camera_params.yaml`:
1. **Colour decides the class.** Two HSV `inRange` masks (red wraps hue → two bands) →
   `red_cube`, `blue_cylinder`. The world's colours are unique per class
   (`red 0.9/0.1/0.1`, `blue 0.1/0.1/0.9`, `vision_sim.launch.py:113-165`). Circularity is logged as a
   diagnostic only.
2. **Closed regions:** `MORPH_OPEN(3)` then `MORPH_CLOSE(k)` with
   `k < (MIN_SEP − object_size) · fx / h` (≈ 9 px at h = 1.00 m → use `k = 5`), then
   `connectedComponentsWithStats` → **one detection per component**.
3. **Adaptive area gate** instead of constants: `expected_px = object_size · fx / Z`, accept
   `area ∈ [0.4, 4.0] · expected_px²`. (This is what makes one detector work at 1 m overhead *and* on a
   wrist camera at 0.4 m.)
4. **Reject border-touching components** (log them) — a clipped silhouette is never trustworthy.
5. **Depth:** median over the whole component mask, `depth_valid_frac ≥ 0.3`, keep `[0.30, 2.00] m`.
6. **Centroid** from `cv2.moments` (not `minAreaRect`); bbox from `boundingRect`; **also publish the
   `minAreaRect` angle** — it is computed today and thrown away (`:259`), and it is exactly the object
   yaw needed for alignment (Finding C).
7. Keep the `Detection2DArray` schema (`ObjectHypothesisWithPose`) so `sorting_controller._detection_cb`
   needs no change; extend `/vision/debug_image` with the masks.

### 3.3 Option A — fixed top-down camera ("~1.50 height")
`fx ≈ 462` at 640×480; nadir at `(-0.90, 0, z)`; coverage on the table = `1.039·h × 1.385·h`
(x × y) with `h = z − 1.00`:

| h above the objects | world z | coverage x × y | object size | note |
|---|---|---|---|---|
| 0.55 | 1.50 | 0.571 × 0.762 m | 84 px | ❌ **spawn band 0.60 × 0.80 m does not fit** — edge objects clipped |
| 0.75 | 1.75 | 0.779 × 1.039 m | 62 px | ⚠️ marginal in x |
| **1.00** | **2.00** | **1.039 × 1.385 m** | **46 px** | ✅ full table + ≥ 10 cm margin (recommended) |
| 1.50 | 2.50 | 1.559 × 2.077 m | 31 px | ✅ but only 31 px/object — resolution wasted |

So if "1.50" means world z, **it is 20 cm too low**; if it means 1.50 m *above the table* it works but
wastes resolution. Recommendation: **h = 1.00 m (world z = 2.00)**.
* The frame then covers `x ∈ [-1.42, -0.38]`, `y ∈ [-0.69, +0.69]` → whole pick table with margin and
  the sort zones (x −0.28/−0.40) effectively out of frame → no accidental re-counting of placed objects.
* Image mapping (verified by projecting the pose): **image right = world −Y**, **image down = world +X**;
  the pick band maps to `v ≈ 295…365`, `u ≈ 144…496`.
* Collision: the arm's highest reachable TCP is world z ≈ **1.617** (`is_reachable`: `dz ≤ L1+L2 = 0.817`),
  so a camera at 2.00 m is clear; the mount post must sit **outside** the FOV footprint
  (`x ∈ [-1.59, -0.21]`, `y ∈ [-0.52, 0.52]`) **and** outside the arm's 0.817 m reach → put the post at
  `x ≈ -1.60` with a boom at z ≈ 2.00 (add it to the MoveIt scene as insurance).
* Changes: `worlds/vision_sorting.sdf:152-218` (pose + visual), `camera_params.yaml`
  (`xyz: [-0.90, 0.0, 2.00]`, `rpy: [0.0, 1.5708, 0.0]`), fix the dead `intrinsics:` block
  (`fx ≈ 462`). `camera_tf_broadcaster.py` needs no code change (pose comes from YAML).

### 3.4 Option B — wrist (eye-in-hand) camera, offset resolved
**Why it is now credible:** at every IK-solved pose the tool axis is **exactly vertical**
(`tool_z = (0,0,-1)`, verified for all sampled targets), so a camera mounted looking along the tool axis
gets a **nadir view at every target without extra orientation control**. (At the *home* pose the tool
axis points **up** — so the scan pose must be an IK-solved pose, not home.)

**Changes**
1. Move the sensor into the arm model so Gazebo renders from the moving link:
   `urdf/arm.urdf.xacro` gains `d415_wrist_link` + a fixed joint from `gripper_base_link`, a
   `<gazebo reference="d415_wrist_link">` block with `<sensor type="rgbd_camera">`,
   `<topic>d415</topic>`, `<gz_frame_id>d415_wrist_link</gz_frame_id>`; add a collision shape so MoveIt
   avoids the camera body on the way down. Delete the static `d415_camera` model from
   `worlds/vision_sorting.sdf` (the arm is spawned from `/robot_description`, so the sensor rides along).
2. `camera_params.yaml`: `camera.parent_frame: gripper_base_link` + mount `xyz`/`rpy`.
   `camera_tf_broadcaster.py` already reads the parent from YAML, so `world → camera_optical` becomes
   dynamic automatically and `shape_detector.transform_to_world()` needs **no change**.
3. **Offset chain — keep the three offsets distinct and parameterised:**
   `wrist → camera_optical` (mount, calibrated), `gripper_base → gripper_tcp` = 0.075
   (`arm.urdf.xacro:368-373`), and the grasp reference = **jaw centre** = `d5 + 0.018` (Finding B).
   A lateral mount offset must be applied **only** in the camera→world transform, never folded into the
   IK z maths.
4. **Calibration/validation:** `scripts/hand_eye_check.py` — drive the arm through a set of scan poses,
   back-project a reference object, compare with Gazebo ground truth, print the 6-DOF residual and emit
   the corrected `hand_eye` block. Acceptance: < 5 mm (sim), < 10 mm (hardware).
5. **Look-then-move, not visual servoing.** Below the RGBD near clip (0.30 m) the object leaves the
   FOV, so: park at a scan pose (0.35–0.45 m above the table), settle, capture, freeze world (x, y) from
   TF **at that instant**, then pre-pick/grasp blind with the analytical IK.
6. **Multi-pose scan** (the price of a wrist camera): one pose sees ≈ 0.5 × 0.36 m at h = 0.4 m, so
   Table 2 needs **2–3 poses** (e.g. y = −0.20 / +0.20, x = −0.70) with the existing `_registry` merging
   detections across poses; the mission's `EMPTY_SCANS_TO_FINISH` logic then works per pose-set.
   With 2 + 2 objects this is a 2-pose scan ≈ 10 s — acceptable.

### 3.5 Recommendation
* **Counting** → the fixed camera (§3.3) is objectively better: one frame, whole table, no merging.
* **Picking/alignment** → the wrist camera (§3.4) is better: it closes the loop at the point of grasp
  (your point 4) and survives a moved/rotated object.
* **Do this:** implement the **wrist camera as the primary** (your lean) with the 2-pose scan, and keep
  the colour-based detector shared by both. If the count still flaps, add the fixed camera as a pure
  **census** camera — its only job is counting, and the wrist camera does the alignment. The two are
  complementary, not exclusive, and the detector work in §3.2 is identical for both.
* **Keep 2 + 2** either way (your point 1).

---

## 4. W3 — Grasp verification (your point 3)

### 4.1 3a — Contact sensors on both jaws (authoritative in sim)
Insertion point is already there — extend the existing blocks `arm.urdf.xacro:429-445`:
```xml
<gazebo reference="right_prong_link">
  <sensor name="right_prong_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>50</update_rate>
    <contact>
      <collision>right_prong_collision</collision>   <!-- add explicit collision names first -->
    </contact>
    <topic>right_prong_contact</topic>
  </sensor>
</gazebo>
```
(+ mirrored for `left_prong_link`; the prong `<collision>` elements currently have **no** `name`, so add
`name="right_prong_collision"` / `name="left_prong_collision"`.) Bridge the topics in
`vision_sim.launch.py` (see E3 for the mapping check) and treat `contact ≠ empty` as "object between the
jaws". Optionally also add a `force_torque` sensor on the wrist for the real-robot path.

### 4.2 3b — Jaw position + velocity check (works without world changes)
Your rule, with corrected numbers. Prong geometry (`arm.urdf.xacro:330-363`): joint origins at
`y = ±0.02`, prong half-thickness 0.0025 → **inner face** `= 0.02 + q − 0.0025`, gap `= 2 ×` that.
* Object 0.1 m ⇔ jaw face at ±0.05 ⇔ right prong reads **q = 0.0325** when blocked.
* `q = 0` → gap 0.035 m; `q = 0.028` (today's close) → 0.091 m; `q = 0.04` (open) → 0.115 m;
  `q = 0.06` → 0.155 m (the recommended new open value, Finding D).
* **Signal budget:** today's close gives `|0.0325 − 0.028| = 4.5 mm` (5.6 % of the 80 mm travel) —
  too fragile. Closing to `q = 0.012` (gap 0.061 m) gives **20.5 mm (25.6 %)** → robust.
* **Velocity alone is not a discriminator** (both "grasped" and "closed on air" end at v ≈ 0). Use the
  combination: `grasped = |v| < v_tol for ≥ 0.2 s AND measured gap ∈ [0.085, 0.115] m`
  (this also *catches corner grasps*, which stall at ≈0.141 m → flagged as a failure → retried with the
  yaw alignment of §5).
* Read the joints from `/joint_states` (both prongs have `position` **and** `velocity` state interfaces,
  `config/controllers.yaml:29-38`); `move_gripper()` should return the achieved positions instead of
  ignoring the result.
* **Prerequisite:** E1 — if the position command turns out to be realised kinematically, this check is
  blind (the measured position = the command). Then either switch the prongs to an `effort` command
  interface with a PID, or depend on 3a.

### 4.3 Integration (fail-safe, and the count stays honest)
In `sorting_controller._pick_and_place()`:
* after **step 4 (close)** → `_verify_grasp()`; on failure: open, retreat, `return False` (the existing
  `MAX_ATTEMPTS_PER_OBJECT` retry/blacklist logic handles it) — **a failed grasp is never counted as a
  sort**, so `red_ok/blue_ok` can no longer inflate.
* after **step 5 (lift)** → `_verify_still_held()` (slip/drop detection) → same failure path.
* with a camera in the scene, add the free cross-check (3c): after the lift, the object's silhouette must
  have **disappeared from the pick region** and reappear under the gripper's projected position.
* config: `grasp.verify: contact|joint_stall|both|vision|none`, `grasp.v_tol`,
  `grasp.gap_window: [0.085, 0.115]`, `grasp.settle_secs`.

### 4.4 Tests (pure python, same style as `test_vision_sorting.py`)
`_grasp_from_joint_state(q_max, v_max, tol)` → cases: held (q=0.0325, v≈0), closed on air (q=0.012),
corner grasp (q≈0.0705 → gap 0.141 → flagged), slip after lift (v spike). Contact path is tested with a
mocked message. Also assert that a failed grasp **does not** increment `red_ok/blue_ok`.

---

## 5. W4 — Gripper alignment (your point 4 — keep it, it is the missing link)

**Verdict:** do **not** drop this idea. It is the correct thing to do and it is cheap; it currently fails
for three independent reasons that have nothing to do with the concept:

| alignment axis | today | required | fix |
|---|---|---|---|
| **lateral x/y** | IK base-branch bug: **218 mm** off for every y > 0 object (Finding A) | ≤ 5 mm (x), ≤ 10 mm (y) | correct branch (§1.6) + accurate vision (§3.2) |
| **axial z** | jaw centre **57 mm** above the object centre → 43 % engagement (Finding B) | ≤ 5 mm | grasp reference = jaw centre (`TCP_Z_OFFSET = 0.113`) |
| **yaw** | **7°–25°** off the cube faces; open gap cannot clear a 141 mm diagonal (Findings C, D) | ≤ 15° for a face grasp | `θ5 += Δyaw` (free, 0.00 mm TCP shift) + wider open gap |

### 5.1 Tolerances derived from the geometry (targets to validate in sim)
Jaw: 20 mm wide (x) × 5 mm thick × 100 mm long (approach axis); object 100 mm.
* **x** (jaw width direction): |δx| ≤ **5 mm** — beyond that the contact patch shrinks fast.
* **y** (closing direction): |δy| ≤ **10 mm** for a symmetric grasp (the closed gap 0.1 m means up to
  25 mm still touches, but asymmetrically → the object can slide out sideways).
* **z** (axial): the jaws are 100 mm long, so ±20 mm still engages 60 %; today's error is 57 mm → 43 %.
* **yaw**: ≤ **15°** for a cube face grasp (cube is periodic every 90°); a cylinder is insensitive.

### 5.2 Alignment procedure (per pick, in this order)
1. **Detect** the object's world (x, y) **and yaw** (from the mask's `minAreaRect` angle, §3.2-6).
2. **Yaw-align**: `θ5 += wrap(nearest_90°(ψ_obj) − jaw_yaw(θ1, θ4))` where `jaw_yaw` comes from the same
   FK/bookkeeping used for `θ1`. Verified: this changes the jaw yaw by exactly Δ with **0.00 mm** TCP
   shift (dry-run table in Finding C).
3. **Axial reference**: send the IK target with the jaw-centre reference (Finding B), i.e. keep
   `PICK_WORLD_Z = 1.00` and set `TCP_Z_OFFSET = 0.113`.
4. **Pre-pick**: approach from +0.10 m above with the **wide** open gap (≥0.155 m, Finding D).
5. **Closed-loop correction (only with a wrist camera)**: at the pre-pick pose, measure the object's
   pixel offset from the calibrated "jaw axis" pixel point and correct x/y **once** before descending
   (one iteration is enough at ≤5 mm/step). This is what turns an open-loop guess into a guaranteed
   alignment and is the strongest argument for the wrist camera.
6. **Verify** with §4 — grasp check *and* the alignment metrics logged (`δx, δy, δz, δyaw`), so a failure
   is diagnosable instead of silent.

### 5.3 What to be sceptical about (honest list)
* A **knee-jerk "align then pick" is not free**: without step 5 it is an open-loop bet on calibration;
  with the *fixed* camera you can only align once, before the descent, and the descent itself doesn't
  move laterally (the IK is exact to 0.1 mm) — so open-loop is acceptable **provided** the camera
  calibration is right. Measure `δ` on every pick to know.
* The **cube's yaw changes** if a failed pick nudges it → always take the yaw from vision, not from the
  spawn-time assumption.
* **Aligning to the object centre is not the same as a stable grasp** for a 50 g cube: the contact area
  after a face grasp is 20 mm × 100 mm per jaw; after an edge grasp it is a line → verify with the gap
  check (§4.2), which flags 141 mm stalls.
* Don't align **while** descending (no lateral motion inside the jaws); align, *then* descend.

---

## 6. W5 — Fix the arm/block constants and the IK branch (single source of truth)

These are the "pre-determined values" from the earlier question — and three of them are now proven wrong.

| literal today | truth | action |
|---|---|---|
| `pick_and_place.py:71-77` `L1_LENGTH=0.425`, `L2_LENGTH=0.392`, `D4_OFFSET=0.109`, `TCP_Z_OFFSET=0.170`, `JOINT_LIMIT=3.14` | `urdf/arm.urdf.xacro:17-22` `a2, a3, d4`, plus `d5 + grasp_ref`; joint limit from the URDF `<limit>` | move to `robot_params.yaml → kinematics:` and replace the TCP value with **`d5 + jaw_centre = 0.113`** (Finding B) |
| `_compute_ik()` branch `min(|θ1|)` | `θ1 = wrap(a0 + asin(d4/R))` (Finding A) | code fix + **rewrite** `test_compute_ik_picks_smaller_swing_branch` |
| `GRIPPER_CLOSE = [0.028, -0.028]`, `GRIPPER_OPEN = [0.04, -0.04]` | close `±0.012` (20.5 mm signal), open `±0.06` (clears the 141 mm diagonal) | constants into `robot_params.yaml → grasp:` |
| `planning_scene_manager` attach offset `z = 0.075` | must equal the grasp reference (`0.018` jaw centre) | change **together** with the IK constant |
| `robot_params.yaml:39-45` prong limits `[-0.4, 0] / [0, 0.4]` | URDF: right `[0, 0.08]`, left `[-0.08, 0]` | fix the YAML |
| `pick_and_place.py:445-518` seven hardcoded waypoints + scene literals | `robot_params.yaml → mission.pick_sequence:` / `scene:` | read from YAML |
| `robot_params.yaml:57-59` `vision.use_vision` | read by nothing | implement the branch or delete |
| `camera_params.yaml` `intrinsics.fx = 602` | SDF 69.4° HFOV ⇒ **462**; the node uses `/d415/camera_info` | correct or delete the block |

**Drift test (new, in `test/test_params.py`):** parse the xacro properties from `urdf/arm.urdf.xacro`
and assert `kinematics.a2/a3/d4` match the YAML, and that
`tcp_grasp_reference == d5 + jaw_centre_offset`. The constants can then never silently diverge from the
model again — this is exactly the test that would have caught Finding B.

**Docstrings:** state plainly that `pick_and_place.py:run()` is the **scripted offline demo** (it grasps
`target_box`, a MoveIt-only collision object that `planning_scene_manager.add_target_box()` creates
itself — the camera never sees it, hence the hardcoded coordinates), while `sorting_controller.py`
(`make vision-sort`) is the **perception mission** and is where the vision work belongs.

---

## 7. Test & verification plan

| level | how | pass criterion |
|---|---|---|
| unit | `make test` | existing suites green + new detector / grasp / params / IK-branch tests |
| IK branch | new test: for a grid of in-reach targets, FK the IK output (the FK helper is pure numpy, no ROS) | TCP error < 1 mm for 100 % of sampled targets (today: 218 mm for every y > 0) |
| detector | synthetic masks at 46 px: 2 red squares + 2 blue circles, a touching pair, a border-clipped one, a red/blue pair with identical circularity | exactly 2 + 2; touching pair = 2 objects; clipped rejected; colour decides |
| offline replay | record `/d415/image` + `/d415/depth_image` + `/d415/camera_info` to a bag, replay through the detector | count stable for ≥ 20 frames; `δxy` annotated against ground truth |
| sim census | `make vision-clean && make vision-sim` + detector | spawn-truth YAML count == `/vision/detections` count |
| grasp | place no object at the target, then a real object; log prong q/v and (if bridged) contact | 100 % of empty grasps reported as failure; 0 false failures over 10 real grasps |
| alignment | log `δx, δy, δz, δyaw` per pick (spawn truth vs IK target vs jaw FK) | \|δx\| ≤ 5 mm, \|δy\| ≤ 10 mm, \|δz\| ≤ 5 mm, \|δyaw\| ≤ 15° |
| mission | `make vision-sort` | summary == 4 objects, no *"All 2 slots … are full"*, no unverified grasp counted |
| hand-eye (wrist) | `python3 scripts/hand_eye_check.py` | residual < 5 mm (sim) |

## 8. Risks / open questions
1. **E1** (kinematic vs force-limited prong commands) could invalidate 3b → run it first; 3a is the fallback.
2. **E3** contact-sensor bridging support in this `ros_gz_bridge` build.
3. A wrist camera makes *counting* a multi-pose registration problem; if the count still flaps, add the
   fixed census camera (§3.5) rather than more detector heuristics.
4. The overhead mount must clear both the FOV footprint and the arm's 0.817 m reach (post at x ≈ −1.60).
5. Colour segmentation assumes the SDF `diffuse` colours stay unique per class; keep a depth-based
   fallback (top-face segmentation) in reserve.
6. Changing the grasp reference (Finding B) moves where the arm physically descends by **5.7 cm** — every
   existing height/waypoint must be re-validated in sim.
7. `move_gripper()` currently ignores the controller result — the grasp work needs it to report back.

## 9. Execution order (on your signal)

| # | Step | Depends on | Why in this order |
|---|---|---|---|
| 1 | E1, E2, E3 experiments (§0.1) | — | ~30 min, decides 3a vs 3b and de-risks everything |
| 2 | 2 + 2 scene reduction (§2) + spawn-truth hook | — | makes every later measurement unambiguous |
| 3 | Colour/morphology detector (§3.2) + tests | — | fixes the counts independently of the camera choice |
| 4 | IK branch fix + `TCP_Z_OFFSET = 0.113` + attach offset + drift test (§6) | — | without it every pick misses by 218 mm laterally / 57 mm axially |
| 5 | Grasp verification 3b (+ 3a if E3 passes) + tests (§4) | 1 | stops phantom "successes" |
| 6 | Alignment: yaw (θ5), wide open gap, pre-pick, δ logging (§5) | 4, 5 | completes your point 4 |
| 7 | Camera decision & mount: wrist (§3.4) or fixed (§3.3) + `hand_eye_check.py` | 3 | larger change — do it on a working base |
| 8 | Docs (`README`, `CHANGELOG`) + runbook (`make vision-clean` first, `ros2 node list` check) | all | |

**Nothing below item 4 can succeed on its own:** a perfect alignment step still misses by 218 mm if the
IK branch is wrong, and a perfect grasp check cannot report a grasp that never happened.

---

## Appendix — how the numbers in this document were produced

All measurements are read-only and reproducible without ROS:

* `python3 /tmp/fk_check.py` — parses `urdf/arm.urdf.xacro` (xacro properties + joint origins),
  rebuilds the kinematic chain (numpy), replays `_compute_ik()` verbatim, and prints:
  tool-axis direction, TCP error, jaw span vs a 0.1 m object, jaw gaps for OPEN/CLOSE.
* `python3 /tmp/fk_branch.py` — sweeps the pick band, evaluates **both** IK branches with FK, and reports
  which branch reproduces the target (9/21 targets are 218 mm off with today's `min(|θ1|)` rule).
* `python3 /tmp/fk_yaw.py` — jaw-yaw vs cube face normals, and the TCP shift caused by a wrist roll
  (0.00 mm → yaw alignment is free).
* Circularity values come from a 20-line script running this repo's exact detector pipeline
  (`GaussianBlur(5,5)`, `Canny(40,120)`, `RETR_EXTERNAL`, `contourArea`/`arcLength`) on synthetic
  silhouettes at 46/100 px scales.

Nothing in the repository has been modified yet — this file is the plan.

---

## 10. Error-test run results (2026-09-20, this container)

### 10.1 Repo test suite — green, and therefore blind
* Documented path works: `source /opt/ros/lyrical/setup.bash && source install/setup.bash &&
  colcon test --packages-select robotic_4dof_arm` → **`colcon test-result`: 47 tests, 0 errors,
  0 failures, 0 skipped** (xunit suites: `test_params` 6, `test_scene_manager` 5,
  `test_vision_sorting` 33 = **44 python tests, all passing**).
* **Conclusion:** nothing in the suite covers Findings A–D. Worse, inverting the finding:
  `test_compute_ik_picks_smaller_swing_branch` asserts `θ1 = 2.8275` for `(-0.676, 0.105)` — measured
  FK error of that branch is **218.0 mm**; the correct `wrap(3.1475) = −3.1357` gives **0.1 mm**.
  The test actively locks in the miss, so it must be rewritten (§1.6 Finding A), not deleted.
* **Environment bug found:** a direct `python3 -m pytest src/robotic_4dof_arm/test` **hangs then gets
  SIGKILLed (exit 137, no output)** even for the trivial `test_params.py`, because ROS autoloads the
  entry-point plugin `launch_testing_ros_pytest_entrypoint`, which registers the hook
  `pytest_launch_collect_makemodule` — removed in **pytest 9.0.2** (`PluginValidationError`), i.e. an
  autoload-time incompatibility that spins/OOMs. Workarounds that work here:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ...` (**44 passed in 4.43 s**) or `colcon test`.
  Recommend adding this to the runbook (and/or pinning pytest for the direct invocation).

### 10.2 Detector end-to-end error test — the count error, measured
Synthetic frame at the real scene geometry (cubes at ~61° obliquity, cylinders as capsules with cap
aspect 0.45/0.55/0.75, sizes 84–103 px for fx = 462), run through the **unchanged** pipeline
(`GaussianBlur(5,5)` → `Canny(40,120)` → `RETR_EXTERNAL` → area gate → circularity vs 0.88):

| object (truth) | measured circularity | published label |
|---|---|---|
| red cube 1 / 2 / 3 | 0.812 / 0.831 / 0.834 | `red_cube` ✅ |
| blue cylinder 1 / 2 / 3 | **0.756 / 0.756 / 0.757** | **`red_cube` ❌ ×3** |

**Result: `red_cube = 6`, `blue_cylinder = 0` for a truth of 3 + 3.** Every cylinder is misclassified
because **all six** silhouettes (cubes *and* cylinders) fall below the 0.88 threshold — the classes are
not separated by circularity at this camera angle at all.

* Area-gate headroom: the nearest oblique cube silhouette at the YAML's `fx = 602` scale measured
  **28 069 px² = 94 % of `contour_area_max = 30000`** → only ~7 % headroom before the object is silently
  dropped. (No drop was measured at this size; the earlier 18–25 k px² figures hold at fx = 462.)

### 10.3 Grasp / IK error tests (FK, reproducible, no ROS)
* Base-joint branch: **9 of 21** sampled in-reach pick targets land **218.0 mm (= 2·d4)** from the object
  with the current `min(|θ1|)` rule; `a0 + asin(d4/R)` is correct (0.1 mm) in **all 21**.
  Live example: target `(-0.63, +0.30, 0.05)` → TCP `(-0.5067, +0.4798, +0.0499)`
  (error `+0.1233, +0.1798`).
* Tool axis at every IK pose: `tool_z = (0.000, 0.000, -1.000)` → **exactly vertical** (supports the
  wrist-camera plan); at *home* it points **up**.
* Jaws vs object: jaw span `[-0.032, +0.068]`, centre `+0.018`; IK reference `+0.075` → jaw centre
  **57 mm** above the object centre → **43.0 mm of 100 mm engaged** (today) vs **100 mm** when the IK
  targets the jaw centre (`TCP_Z_OFFSET = 0.113`).
* Jaw gaps: open `±0.04 → 0.1150 m`, close `±0.028 → 0.0910 m`, `q = 0 → 0.0350 m`; blocked by a 0.1 m
  object the right prong reads `0.0325` → **grasp signal 4.5 mm today** vs **20.5 mm** at `±0.012`.
* Jaw yaw: `-97.33°` / `-65.44°` / `-106.47°` → **7.33° / 24.56° / 16.47°** off the nearest cube face;
  a `+45°` / `+90°` wrist roll on joint 5 (or joint 6) changes the yaw with **0.00 mm** TCP shift.

### 10.4 E3 — contact-sensor bridging: **supported** ✅
`ros_gz_interfaces` ships `Contact` and `Contacts`; `libros_gz_bridge.so` contains the factories for
`gz.msgs.Contact ↔ ros_gz_interfaces/msg/Contact` and `gz.msgs.Contacts ↔ ros_gz_interfaces/msg/Contacts`.
So plan 3a is viable in this build — only the runtime topic name/namespace of the prong sensor still needs
confirming when the sensor is added. (E1/E2 need the live Gazebo + controller stack and were not run here;
### 10.5 Headless live run (E1/E2) — answers, plus two new blocking defects

Stack: `gz sim -s` (server only) + `robot_state_publisher` + arm spawned from the expanded URDF +
`joint_state_broadcaster` + `gripper_controller` (both `active`), `gz_ros2_control` loaded
(`/controller_manager`, `/arm_system`). No GUI, no MoveIt. Scripts: `/tmp/bringup_headless.sh`,
`/tmp/e1_air.sh`, `/tmp/e1e2_full.sh`, `/tmp/e1_stall.sh`, `/tmp/prong_probe.py`.

**NEW DEFECT 1 — the right jaw is (intermittently) inert.**
Command sweep with the left jaw held at −0.020:

| command `right_prong_joint` | measured | log |
|---|---|---|
| +0.005 / +0.020 / +0.040 / +0.080 | **−0.00000 every time** | `[ERROR] [gz_ros_control]: Command of at least one joint is out of limits. Joint: 'right_prong_joint', (actual position -0.000000, command 0.000200, limited 0.010000)` ×8 |
| left `-0.020` (same goals) | −0.02000 (exact) | no error |

The controller still reports **`Goal finished with status: SUCCEEDED`** for every one of those goals
(the `JointTrajectoryController` never compares the measured position with the command).
A later command (`right = +0.045`) *did* move the joint to +0.04503 → the rejection is **not
deterministic**, which is worse: it can silently halve the jaw travel on any given pick.
**Consequence:** with `GRIPPER_OPEN = [0.04, -0.04]` the real opening is
`gap = 0.035 + q_r - q_l = 0.035 + 0 + 0.04 = 0.075 m` instead of the documented 0.115 m —
i.e. **narrower than the 0.1 m cube/cylinder**, so the object cannot even fit between the jaws.
This is a third independent reason the pick cannot work (after Findings A and B).

**NEW DEFECT 2 — position commands are applied kinematically (no force feedback).**
Decisive stall test: the left jaw was commanded from −0.045 to −0.001 (a 44 mm closure) with a
**static** block whose face sat 33.5 mm inside that path (`stall_block`, world y −0.160):

```
measured right=-0.00000  left=-0.00100          <- exactly the command
left jaw inner face world y = -0.1275           <- 32.5 mm PAST the block face at -0.160
```

→ the joint drove **through** a static obstacle and landed exactly on the commanded value.

**E1 answer:** the prong position command is **not** force-limited, so the jaw-position/velocity rule
(your 3b) **cannot detect a grasp in this stack**: with an object between the jaws the measured gap is
simply whatever was commanded. This is independent of the right-joint defect and survives a fix of it.

**E2 answer (moot until E1 is fixed):** measured velocities are **0.00000 m/s in every steady state**
(free, blocked, and after every goal) → velocity alone cannot discriminate, exactly as predicted;
only a position difference could, and that signal is destroyed by the kinematic command path.

**Also observed:** with the static block present, a later close command left *both* jaws unmoved while
the goal still returned `SUCCEEDED` and both controllers stayed `active` → a **silent no-op failure
mode**: the mission's `move_gripper()` cannot distinguish "jaws closed on an object" from "jaws did not
move at all".

**Therefore (revised priorities):**
1. Fix the gripper **before** anything else: give the prongs symmetric limits / a working command path
   (or switch the command interface to `effort` with a PID) so **both** jaws move and the real gap
   range is 0.035–0.155 m. Without this, no grasp check and no open-gap clearance is meaningful.
2. Your **3a (contact sensors)** is now the *only* viable in-sim grasp signal: the bridge supports
   `gz.msgs.Contacts ↔ ros_gz_interfaces/msg/Contacts` (verified in §10.4), and it also works with a
   kinematic command path because it reports the contact itself, not the joint state.
3. Keep 3b as a *secondary* check only after the command path is force-based.
4. `move_gripper()` must verify the achieved positions (today it cannot fail) — this is what made all
   of the above invisible in the mission logs.

Reproduce: `bash /tmp/bringup_headless.sh` then `bash /tmp/e1e2_full.sh` and `bash /tmp/e1_stall.sh`.

### 10.6 Step-1 implementation results — gripper fix verified live (2026-09-20)

Implemented per §9 step ordering (gripper first, no other execution until user signal):

| fix | file:line | change |
|---|---|---|
| W1a symmetric prong limits | `urdf/arm.urdf.xacro:334,361` | right `[0, 0.08]` → `[-0.08, 0.08]`, left `[-0.08, 0]` → `[-0.08, 0.08]` |
| W1a named collisions for sensors | `urdf/arm.urdf.xacro:322,348` | `right/left_prong_link_collision` |
| W1b contact sensors (idea 3a) | `urdf/arm.urdf.xacro:436,453` | `right/left_prong_contact`, 50 Hz |
| W1b contact-system plugin | `worlds/vision_sorting.sdf:12` | `gz-sim-contact-system` (was missing → sensors silent) |
| W1b ROS bridges | `launch/vision_sim.launch.py:299-305` | `/gripper/right_contact`, `/gripper/left_contact` |
| W1c real gap range | `config/robot_params.yaml:40-55` | limits ±0.08, home open `0.06/-0.06` |
| Finding A — IK branch | `scripts/pick_and_place.py:137` | `a0 + offset` preferred (was `min(|θ1|)` → 218 mm miss for all y > 0) |
| Finding B — axial grasp ref | `scripts/pick_and_place.py:75` | `TCP_Z_OFFSET 0.170 → 0.113` (d5 + jaw centre) |
| Finding C — jaw yaw slack | `scripts/sorting_controller.py:81` | `GRIPPER_OPEN ±0.04 → ±0.06` (0.155 m clears the 0.141 m diagonal) |
| grasp signal budget | `scripts/sorting_controller.py:82` | `GRIPPER_CLOSE ±0.028 → ±0.012` (signal 4.5 → 20.5 mm) |
| grasp verification (idea 3b) | `scripts/sorting_controller.py:298-375` | `_jaw_gap()` + `_verify_grasp()` after close, slip check after lift, `/joint_states` subscription |
| 2+2 scene (idea 1) | `launch/vision_sim.launch.py:44-45,59-62`, `sorting_controller.py:83`, `robot_params.yaml:65-77`, `test/test_vision_sorting.py` | 3+3 → 2+2, `MAX_SLOTS 3→2`, 2 slots/class, spawner tests to 4, IK-branch test rewritten to the correct branch (`-3.1355`) |

**Verification (all in-repo):**
* `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/robotic_4dof_arm/test` → **44 passed**
* `py_compile` on all four touched Python files → OK
* headless live (`gz sim -s` + RSP + spawner + both controllers `active`):
  open `[0.06, -0.06]` → measured `+0.06000 -0.06000`; close `[0.012, -0.012]` →
  measured `+0.01200 -0.01200` — **both jaws move symmetrically, zero `out of limits`
  errors** (was: right jaw stuck at 0.0 with ×8 limit errors per goal).
* Gazebo publishes both contact topics:
  `…/right_prong_link/sensor/right_prong_contact/contact`,
  `…/left_prong_link/sensor/left_prong_contact/contact` (ROS bridge nodes pending
  `TimerAction` bring-up in the full launch).

**Notes:** `sorting_controller.py` / `test_vision_sorting.py` show as `??` in `git status` only
because HEAD never tracked them (pre-existing repo state, `git log 6911e16`), not because of this
work. Remaining per plan: full-launch validation of the contact bridges, step 2 (overhead camera at
world z ≈ 1.80–2.00), step 3 (colour-segmentation detector), wrist-camera / hand-eye work (§3.4).




---

## 7. Implementation addendum (2026-09-20, post-validation)

All four ideas implemented on `main` (uncommitted):

| Fix | File(s) | Live-validated result |
|---|---|---|
| IK branch: `a0 + asin(d4/R)` preferred (was min\|theta1\|) | `pick_and_place.py:_compute_ik` | FK sweep: TCP lands on target for every spawn point incl. y > 0 |
| `TCP_Z_OFFSET` 0.170 → 0.113 (jaw span centre) | `pick_and_place.py` | grasp plane now at the object equator |
| `GRIPPER_OPEN ±0.06` (0.16 m real gap), `GRIPPER_CLOSE ±0.012` | `sorting_controller.py` | commanded close 0.16 → 0.087 m real sep |
| Grasp verification = measured jaw gap in [0.085, 0.115] m | `sorting_controller.py` | **E1 LIVE:** commanded ±0.012 with a 0.1 m block between the jaws → actual prongs stall at ±0.0325 → gap 0.100 m = HELD; empty close reaches ±0.012 → gap 0.059 m = EMPTY. 41 mm signal. |
| 2 cubes + 2 cylinders, 2 slots/class | `vision_sim.launch.py`, `robot_params.yaml`, `MAX_SLOTS=2` | spawner + zones consistent; 44 tests pass |
| Jaw contact sensors + world Contact system + ros_gz bridges | `arm.urdf.xacro`, `vision_sorting.sdf`, `vision_sim.launch.py` | **E3 LIVE:** topics exist (`/gripper/{left,right}_contact`) but publish NO data even with verified physical jaw-block contact (stalled jaws, real pose confirmed). Contacts are NOT a usable grasp signal in this stack — the jaw-gap check (3b) is the implemented verifier. Bridges kept (harmless) for future Gazebo versions. |
| Symmetric prong limits ±0.08 (URDF + `robot_params.yaml`); home ±0.06 | `arm.urdf.xacro`, `robot_params.yaml`, `arm.srdf` | out-of-limits command spam eliminated |

**Open caveat:** with kinematic jaw closing, a *free-standing* object can be pushed out of the jaws
before the close finishes (observed once with a 0.3 kg block on a friction surface). In the real
mission the objects rest on the table with mu = 1.5 and the arm descends from directly above, which
cages the object; the stall check then behaves as validated. If ejections recur, lower
`GRIPPER_CLOSE` further (e.g. ±0.02, gap 0.075) so the squeeze phase is short.
