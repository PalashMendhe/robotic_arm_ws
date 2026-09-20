#!/usr/bin/env python3
"""
sorting_controller.py  —  Week 3: Autonomous Vision-Based Sorting
------------------------------------------------------------------
Perception-driven sorting for an UNKNOWN number of objects:

  home → [ scan → pick → lift → traverse → place → retreat ] × N → home

The controller never assumes how many objects the world contains.  It scans
Table 2, sorts the nearest reachable object, then re-scans.  The mission ends
only when EMPTY_SCANS_TO_FINISH consecutive scans find nothing new — so the
cycle count always equals the number of objects actually on the table.

COORDINATE FIX
--------------
The shape_detector publishes world-frame (x, y, z) but TF calibration may
drift.  Since all objects spawn at world z=1.00 (table surface 0.95 + half
object 0.05), the grasp uses the fixed PICK_WORLD_Z; only (x, y) from the
detector steer the arm.  Detected z is logged for TF validation only.

IK z conversion:  arm_z = world_z - ARM_BASE_WORLD_Z (0.95)
  pick:        arm_z = 0.05   (at object centre)
  pre-pick:    arm_z = 0.15   (+10 cm above object)
  lift:        arm_z = 0.15   (straight up after grasp)
  traverse:    lifted pose → above the zone at pre-place height
  place:       arm_z = 0.10
  pre-place:   arm_z = 0.25

MOTION SEQUENCE (per object) — no home visit anywhere inside the loop
---------------------------------------------------------------------
  home (once at mission start)
  loop (per object):
    1. open gripper
    2. pre-pick   (above object)    MoveIt
    3. grasp      (descend)         MoveIt
    4. close gripper
    5. lift       (straight up)     MoveIt   ← short vertical leg
    6. traverse   (above the zone)  MoveIt   ← plans FROM the lifted pose,
                                                 so the path is a short high
                                                 swing — the arm can no
                                                 longer fold back to home
    7. place      (descend)         MoveIt
    8. open gripper
    9. retreat    (straight up)     MoveIt   ← ready for the next pick
  home (once at mission end)

FAILURE HANDLING
----------------
  * Pre-pick / grasp failure → object untouched; the cycle is retried later.
  * Lift / traverse failure  → retried from the lifted pose; if retries are
    exhausted the object is lowered back onto the pick table (never dropped
    mid-swing) and the cycle reports failure.
  * An object that fails MAX_ATTEMPTS_PER_OBJECT times is blacklisted so the
    mission can never loop forever.
"""

import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from vision_msgs.msg import Detection2DArray

current_dir = os.path.dirname(os.path.realpath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from pick_and_place import PickAndPlace   # noqa: E402


# ── Coordinate constants ──────────────────────────────────────────────────────
ARM_BASE_WORLD_Z = 0.95    # arm spawn height in Gazebo world frame
PICK_WORLD_Z     = 1.00    # all objects on table at this world z (fixed)

# ── Detection tuning ──────────────────────────────────────────────────────────
DETECTION_COLLECT_SECS = 5.0   # seconds to collect detections per scan
DEDUP_RADIUS_M         = 0.10  # in-scan: detections within 10 cm → same object
MATCH_RADIUS_M         = 0.07  # cross-scan registry match (< MIN_SEP / 2)
GRIPPER_OPEN           = [0.06, -0.06]   # 0.155 m gap clears a 45-deg-rotated cube (0.141 m)
GRIPPER_CLOSE          = [0.012, -0.012]  # 20.5 mm grasp signal vs 4.5 mm at +/-0.028
MAX_SLOTS              = 2     # placement slots per class (sort_zones grid)


# ── Grasp verification (idea 3b): jaw gap + settled velocity ────────────────
# Jaw geometry (arm.urdf.xacro:330-363): prong origins at y = +/-0.02,
# axis +Y on BOTH joints.  Inner-face gap = 0.035 + qr - ql.
# A 0.1 m object stalls the jaws at gap = 0.10 (qr = +0.0325, ql = -0.0325).
# With GRIPPER_CLOSE = [0.012, -0.012] the command (gap 0.059) can never
# reach the object faces, so 3b compares the MEASURED gap against the
# object size: HELD = gap in [0.085, 0.115] with both jaw velocities ~0.
# A grasp is accepted when the measured jaw gap (from /joint_states via
# the inherited node) sits in [GRASP_GAP_MIN_M, GRASP_GAP_MAX_M].
GRASP_GAP_MIN_M     = 0.085   # 0.10 m object minus tolerance / slight rotation
GRASP_GAP_MAX_M     = 0.115   # open-gap value; also catches 0.141 m corner grasps
GRASP_MAX_VEL_M_S   = 0.01    # jaws must be settled
GRASP_SETTLE_SECS   = 0.4     # wait after close before sampling

# ── Mission loop tuning ───────────────────────────────────────────────────────
LIFT_HEIGHT_M           = 0.10  # vertical clearance for pre-pick / lift legs
EMPTY_SCANS_TO_FINISH   = 2     # consecutive empty scans that end the mission
MAX_ATTEMPTS_PER_OBJECT = 2     # failed cycles before an object is blacklisted
REACH_MARGIN_M          = 0.02  # IK envelope margin used to filter candidates
CONSECUTIVE_FAIL_ABORT  = 3     # consecutive failed cycles → systemic MoveIt failure

# ── Pick-region filter (Table 2 only — placed objects live on Table 1) ───────
PICK_REGION_X_MIN = -1.00      # Table 2 spans x: -1.20 … -0.60
PICK_REGION_X_MAX = -0.55      # Table 1 ends at x = -0.50
PICK_REGION_Y_MIN = -0.45
PICK_REGION_Y_MAX =  0.45



def _arm_z(world_z: float) -> float:
    """Convert Gazebo world-frame z to arm-local IK z."""
    return world_z - ARM_BASE_WORLD_Z


class SortingController(PickAndPlace):
    """
    Extends PickAndPlace. Overrides run() with a vision-driven sorting mission.
    Inherits: _compute_ik, is_reachable, move_to_joints, move_gripper,
    preflight_check, _abort.
    """

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            f'SortingController ready  |  PICK_WORLD_Z={PICK_WORLD_Z}  '
            f'ARM_BASE_WORLD_Z={ARM_BASE_WORLD_Z}'
        )

        # ── Detection buffer ─────────────────────────────────────────────
        self._buf: list = []
        self._det_sub = self.create_subscription(
            Detection2DArray, '/vision/detections', self._detection_cb, 10
        )
        from sensor_msgs.msg import JointState
        self._last_joint_state = None
        self._js_sub = self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, '_last_joint_state', msg), 10
        )

        # ── Sort zones from YAML (world z applied at call time) ───────────
        sz = self.params['sort_zones']
        self._place_wz     = float(sz['place_height'])      # e.g. 1.05
        self._pre_place_wz = float(sz['pre_place_height'])  # e.g. 1.20
        self._red_zones    = [(float(p[0]), float(p[1])) for p in sz['red']]
        self._blue_zones   = [(float(p[0]), float(p[1])) for p in sz['blue']]

        # ── Mission state: registry of tracked objects (count unknown) ────
        self._registry: list = []   # [{'cls', 'x', 'y', 'attempts', 'done', 'blacklisted'}]
        self._red_used  = 0         # placement slots consumed in the red zone
        self._blue_used = 0         # placement slots consumed in the blue zone

    # ── Detection callback ────────────────────────────────────────────────
    def _detection_cb(self, msg: Detection2DArray):
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            cls = hyp.hypothesis.class_id
            x   = hyp.pose.pose.position.x
            y   = hyp.pose.pose.position.y
            if cls in ('red_cube', 'blue_cylinder'):
                self._buf.append((cls, x, y))

    # ── Snapshot + dedup ──────────────────────────────────────────────────
    def _collect(self) -> list:
        """
        Spin for DETECTION_COLLECT_SECS, then return deduplicated list of
        (class_id, x, y) using XY-only deduplication (z ignored — fixed).
        """
        self.get_logger().info(
            f'Collecting detections for {DETECTION_COLLECT_SECS:.0f} s ...'
        )
        self._buf.clear()
        deadline = time.time() + DETECTION_COLLECT_SECS
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Deduplicate: cluster by (x, y) within DEDUP_RADIUS_M
        remaining = list(self._buf)
        unique = []
        while remaining:
            seed = remaining.pop(0)
            cluster = [seed]
            outliers = []
            for item in remaining:
                if (item[0] == seed[0] and
                        math.hypot(item[1]-seed[1], item[2]-seed[2]) < DEDUP_RADIUS_M):
                    cluster.append(item)
                else:
                    outliers.append(item)
            remaining = outliers
            # Representative: median x, y of cluster
            xs = sorted(c[1] for c in cluster)
            ys = sorted(c[2] for c in cluster)
            mid = len(cluster) // 2
            unique.append((seed[0], xs[mid], ys[mid]))

        return unique

    # ── Pick-region + reachability filters ────────────────────────────────
    def _on_pick_table(self, x: float, y: float) -> bool:
        """True when (x, y) is inside the Table 2 pick region.

        This excludes anything the camera sees outside Table 2 — in
        particular objects already placed in the Table 1 sort zones — so
        placed objects are never re-picked during a re-scan.
        """
        return (
            PICK_REGION_X_MIN <= x <= PICK_REGION_X_MAX and
            PICK_REGION_Y_MIN <= y <= PICK_REGION_Y_MAX
        )

    def _object_reachable(self, x: float, y: float) -> bool:
        """
        True when the grasp, pre-pick AND lift waypoints are all solvable
        with a safety margin.  Guards against the old failure mode where an
        out-of-reach target made IK silently clamp to a bogus stretched pose.
        """
        pick_az = _arm_z(PICK_WORLD_Z)
        return all(
            self.is_reachable(x, y, az, margin=REACH_MARGIN_M)
            for az in (pick_az, pick_az + LIFT_HEIGHT_M)
        )

    # ── Cross-scan object registry (tracks an unknown number of objects) ──
    def _update_registry(self, detections: list) -> list:
        """
        Matches fresh detections to already-tracked objects (within
        MATCH_RADIUS_M), refreshes their positions, registers new objects,
        and returns the sortable candidates — nearest to the arm first.
        Candidates must be: on the pick table, reachable, not done and not
        blacklisted.
        """
        for (cls, x, y) in detections:
            if not self._on_pick_table(x, y):
                continue  # e.g. objects already placed on Table 1
            best, best_d = None, MATCH_RADIUS_M
            for e in self._registry:
                if e['cls'] != cls or e['done'] or e['blacklisted']:
                    continue
                d = math.hypot(x - e['x'], y - e['y'])
                if d < best_d:
                    best, best_d = e, d
            if best is not None:
                best['x'], best['y'] = x, y        # refresh tracked position
                continue
            if not self._object_reachable(x, y):
                self.get_logger().warning(
                    f'Ignoring {cls} at ({x:.2f}, {y:.2f}): outside arm reach'
                )
                continue
            self._registry.append({
                'cls': cls, 'x': x, 'y': y,
                'attempts': 0, 'done': False, 'blacklisted': False,
            })

        candidates = [
            e for e in self._registry if not e['done'] and not e['blacklisted']
        ]
        candidates.sort(key=lambda e: math.hypot(e['x'], e['y']))
        return candidates

    # ── Zone slot assignment (by placement counter, not detection index) ──
    def _next_zone(self, cls: str):
        """Next free placement slot for a class, or None when all are full."""
        zones = self._red_zones if cls == 'red_cube' else self._blue_zones
        used  = self._red_used  if cls == 'red_cube' else self._blue_used
        if used >= min(MAX_SLOTS, len(zones)):
            return None
        return zones[used]

    def _mark_zone_used(self, cls: str):
        if cls == 'red_cube':
            self._red_used += 1
        else:
            self._blue_used += 1

    def _slot_index(self, cls: str) -> int:
        return self._red_used if cls == 'red_cube' else self._blue_used


    # ── Grasp verification helpers ───────────────────────────────────
    def _prong_states(self):
        """Latest (pos, vel) of both prong joints from /joint_states."""
        qr = ql = vr = vl = None
        msg = getattr(self, '_last_joint_state', None)
        if msg is not None:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                if name == 'right_prong_joint':
                    qr, vr = pos, vel
                elif name == 'left_prong_joint':
                    ql, vl = pos, vel
        return qr, ql, vr, vl

    def _jaw_gap(self):
        """Measured jaw gap in metres, or None when no joint state yet."""
        qr, ql, _, _ = self._prong_states()
        if qr is None or ql is None:
            return None
        return 0.035 + qr - ql   # 0.035 m rest gap + symmetric travel

    def _verify_grasp(self, label):
        """True when the jaws stalled on an object after closing."""
        time.sleep(GRASP_SETTLE_SECS)
        gap = self._jaw_gap()
        qr, ql, vr, vl = self._prong_states()
        if gap is None:
            self.get_logger().warning(f'  [{label}] grasp unverified (no /joint_states yet)')
            return True   # do not block the mission on a missing sample
        held = (GRASP_GAP_MIN_M <= gap <= GRASP_GAP_MAX_M
                and abs(vr or 0.0) <= GRASP_MAX_VEL_M_S
                and abs(vl or 0.0) <= GRASP_MAX_VEL_M_S)
        self.get_logger().info(
            f'  [{label}] grasp check: gap={gap:.4f} m '
            f'(qr={qr:+.4f}, ql={ql:+.4f}) -> {"HELD" if held else "EMPTY"}'
        )
        return held
    # ── Single pick-place cycle (no home visit, no drop mid-swing) ────────
    def _pick_and_place(self, entry: dict, zone_x: float, zone_y: float,
                        label: str) -> bool:
        """
        Pick the tracked object `entry` and place it at (zone_x, zone_y).
        Sequence: pre-pick → grasp → lift → traverse → place → retreat.

        The lift leg breaks the long grasp→place move into two short legs, so
        MoveIt plans a direct high swing and can no longer fold the arm back
        through its home configuration.

        Returns True on success.
        """
        ox, oy = entry['x'], entry['y']
        pick_az      = _arm_z(PICK_WORLD_Z)           # 0.05
        pre_pick_az  = pick_az + LIFT_HEIGHT_M        # 0.15
        place_az     = _arm_z(self._place_wz)         # 0.10
        pre_place_az = _arm_z(self._pre_place_wz)     # 0.25

        self.get_logger().info(
            f'[{label}] pick=({ox:.3f},{oy:.3f}) arm_pick_z={pick_az:.3f}  '
            f'→ zone=({zone_x:.3f},{zone_y:.3f}) arm_place_z={place_az:.3f}'
        )

        # 1. Open gripper
        self.move_gripper(GRIPPER_OPEN)

        # 2. Pre-pick: 10 cm above the object
        if not self.move_to_joints(self._compute_ik(ox, oy, pre_pick_az)):
            self.get_logger().error(f'  [{label}] Pre-pick FAILED')
            return False                              # object untouched

        # 3. Descend to grasp height
        if not self.move_to_joints(self._compute_ik(ox, oy, pick_az)):
            self.get_logger().error(f'  [{label}] Grasp descent FAILED')
            return False                              # still at safe height

        # 4. Close gripper
        self.move_gripper(GRIPPER_CLOSE)
        time.sleep(0.3)
        if not self._verify_grasp(label):
            self.get_logger().warning(f'  [{label}] EMPTY grasp — opening and aborting cycle')
            self.move_gripper(GRIPPER_OPEN)
            return False

        # 5. Lift straight up — keeps the transport path short and high
        if not self.move_to_joints(self._compute_ik(ox, oy, pre_pick_az)):
            self.get_logger().error(
                f'  [{label}] Lift FAILED — object left at grasp pose'
            )
            return False

        # 5b. Object still held after lift? (catches slip/drop)
        gap = self._jaw_gap()
        if gap is not None and not (GRASP_GAP_MIN_M <= gap <= GRASP_GAP_MAX_M):
            self.get_logger().warning(
                f'  [{label}] object lost on lift (gap={gap:.4f}) — setting down'
            )
            self._set_down_at(ox, oy, pick_az, pre_pick_az)
            return False

        # 6. Traverse to above the zone (plans FROM the lifted pose)
        q_traverse = self._compute_ik(zone_x, zone_y, pre_place_az)
        if q_traverse is None or not self.move_to_joints(q_traverse):
            self.get_logger().warning(
                f'  [{label}] Traverse FAILED — retrying from lifted pose'
            )
            if q_traverse is None or not self.move_to_joints(q_traverse):
                self.get_logger().error(
                    f'  [{label}] Traverse FAILED twice — setting object back down'
                )
                self._set_down_at(ox, oy, pick_az, pre_pick_az)
                return False

        # 7. Descend to place height
        if not self.move_to_joints(self._compute_ik(zone_x, zone_y, place_az)):
            self.get_logger().error(
                f'  [{label}] Place descent FAILED — releasing above zone'
            )
            self.move_gripper(GRIPPER_OPEN)           # drops into/near the zone
            self._retreat(zone_x, zone_y, pre_place_az)
            return False

        # 8. Release
        self.move_gripper(GRIPPER_OPEN)
        time.sleep(0.3)

        # 9. Retreat straight up — arm ready for the next pick
        self._retreat(zone_x, zone_y, pre_place_az)

        self.get_logger().info(f'  [{label}] ✓ Complete')
        return True

    # ── Recovery helpers ──────────────────────────────────────────────────
    def _set_down_at(self, x: float, y: float, down_az: float, up_az: float):
        """Best-effort recovery while holding an object: lower it back onto
        the table, release, and lift clear.  Never drops it mid-swing."""
        if self.move_to_joints(self._compute_ik(x, y, down_az)):
            self.move_gripper(GRIPPER_OPEN)
            time.sleep(0.2)
        else:
            self.move_gripper(GRIPPER_OPEN)           # unavoidable drop
        self.move_to_joints(self._compute_ik(x, y, up_az))

    def _retreat(self, x: float, y: float, up_az: float):
        """Best-effort vertical retreat after a place/release."""
        q = self._compute_ik(x, y, up_az)
        if q is not None:
            self.move_to_joints(q)

    # ── Main sorting mission (perception-driven, unknown object count) ────
    def run(self):
        if not self.preflight_check():
            self.get_logger().error('Preflight failed.')
            return

        self.get_logger().info('=' * 60)
        self.get_logger().info('   Vision Sorting Mission — START')
        self.get_logger().info('=' * 60)

        home = list(self.params['joints']['home_position'].values())

        # Phase 1: home (mission start only)
        self.get_logger().info('[Phase 1] Home pose...')
        if not self.move_to_joints(home):
            self._abort('Phase 1: Home')

        # Phase 2: gripper ready to grasp
        self.get_logger().info('[Phase 2] Gripper open — ready to sort')
        self.get_logger().info(
            f'[Phase 2] IK z: pick={_arm_z(PICK_WORLD_Z):.3f}  '
            f'place={_arm_z(self._place_wz):.3f}  '
            f'pre_place={_arm_z(self._pre_place_wz):.3f}'
        )
        self.move_gripper(GRIPPER_OPEN)

        # Phase 3: scan → sort nearest → re-scan loop.
        #   The object count is never assumed; the mission ends only when
        #   EMPTY_SCANS_TO_FINISH consecutive scans find nothing new.
        self.get_logger().info(
            '[Phase 3] Sorting loop (re-scanning after every object)...'
        )
        red_ok = blue_ok = 0
        empty_scans = 0
        scan_no = 0
        motion_fail_streak = 0   # consecutive failed cycles (circuit breaker)
        while empty_scans < EMPTY_SCANS_TO_FINISH:
            scan_no += 1
            detections = self._collect()
            candidates = self._update_registry(detections)

            self.get_logger().info(
                f'[Scan {scan_no}] {len(candidates)} sortable object(s) on Table 2'
            )
            if not candidates:
                empty_scans += 1
                if empty_scans < EMPTY_SCANS_TO_FINISH:
                    self.get_logger().info(
                        f'[Scan {scan_no}] Pick table empty — confirming with '
                        f'{EMPTY_SCANS_TO_FINISH - empty_scans} more scan(s)...'
                    )
                continue
            empty_scans = 0

            entry = candidates[0]                   # nearest reachable object
            cls = entry['cls']
            zone = self._next_zone(cls)
            if zone is None:
                self.get_logger().warning(
                    f'All {MAX_SLOTS} slots for {cls} are full — '
                    f'ignoring further {cls} detections.'
                )
                entry['blacklisted'] = True
                continue

            zx, zy = zone
            label = f'{cls.split("_")[0]}_{self._slot_index(cls)}'
            ok = self._pick_and_place(entry, zx, zy, label)

            if ok:
                motion_fail_streak = 0
                entry['done'] = True
                self._mark_zone_used(cls)
                if cls == 'red_cube':
                    red_ok += 1
                else:
                    blue_ok += 1
            else:
                motion_fail_streak += 1
                entry['attempts'] += 1
                ox, oy = entry['x'], entry['y']
                if entry['attempts'] >= MAX_ATTEMPTS_PER_OBJECT:
                    entry['blacklisted'] = True
                    self.get_logger().warning(
                        f'{cls} at ({ox:.2f},{oy:.2f}) failed '
                        f'{entry["attempts"]}x — blacklisting it.'
                    )
                else:
                    self.get_logger().warning(
                        f'{cls} at ({ox:.2f},{oy:.2f}) failed — retrying '
                        f'(attempt {entry["attempts"] + 1}/{MAX_ATTEMPTS_PER_OBJECT}).'
                    )
                if motion_fail_streak >= CONSECUTIVE_FAIL_ABORT:
                    self.get_logger().error(
                        f'{motion_fail_streak} consecutive sort cycles failed — '
                        'the MoveIt stack is not executing motions (duplicate '
                        'move_group? paused sim? controllers down?). Aborting '
                        'instead of blacklisting every object.'
                    )
                    break

        # Phase 4: home (mission end only)
        self.get_logger().info('[Phase 4] Returning home...')
        if not self.move_to_joints(home):
            self.get_logger().warning(
                'Could not return home — MoveIt stack unhealthy?'
            )

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'   Mission Complete: {red_ok} red | {blue_ok} blue | '
            f'{red_ok + blue_ok} sorted'
        )
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = SortingController()
    try:
        node.run()
    except RuntimeError as e:
        node.get_logger().error(f'Mission aborted: {e}')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
