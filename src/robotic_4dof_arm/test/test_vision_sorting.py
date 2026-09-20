# src/robotic_4dof_arm/test/test_vision_sorting.py
"""Unit tests for the vision-based sorting mission pure logic.

Covers the ROS-free helper methods (no executor / action servers needed):
  - PickAndPlace.is_reachable / _compute_ik  — reachability envelope guard
  - PickAndPlace._normalize_angle / IK branch wrap / _duplicate_server_hint —
    live-run regressions (URDF ±3.14 base limit, duplicate move_group, errors)
  - SortingController._on_pick_table         — Table 2 pick-region filter
  - SortingController._object_reachable      — grasp+lift waypoint check
  - SortingController._update_registry       — cross-scan object registry
  - SortingController._next_zone / _mark_zone_used / _slot_index — slot logic
  - vision_sim spawner                       — reach-constrained sampling
"""
import sys
import os
import math
import random
import importlib.util
import pytest
from unittest.mock import MagicMock

# Point Python at the scripts folder BEFORE importing from it
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from pick_and_place import PickAndPlace  # noqa: E402
from sorting_controller import (  # noqa: E402
    SortingController,
    PICK_WORLD_Z,
    ARM_BASE_WORLD_Z,
    MATCH_RADIUS_M,
    LIFT_HEIGHT_M,
    MAX_SLOTS,
    REACH_MARGIN_M,
    PICK_REGION_X_MIN,
    PICK_REGION_X_MAX,
    PICK_REGION_Y_MIN,
    PICK_REGION_Y_MAX,
)

# Load the launch file as a module (filename has a dot, so use importlib).
LAUNCH_PATH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'vision_sim.launch.py')


@pytest.fixture(scope='module')
def spawner():
    spec = importlib.util.spec_from_file_location('vision_sim_launch', LAUNCH_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def arm():
    """PickAndPlace without __init__ (no ROS graph needed for pure IK logic)."""
    return PickAndPlace.__new__(PickAndPlace)


@pytest.fixture
def controller():
    """SortingController without __init__ (pure logic only)."""
    node = SortingController.__new__(SortingController)
    node.get_logger = MagicMock()
    node._registry = []
    node._red_zones = [(-0.28, -0.12), (-0.28, -0.24), (-0.28, -0.36)]
    node._blue_zones = [(-0.28, 0.12), (-0.28, 0.24), (-0.28, 0.36)]
    node._red_used = 0
    node._blue_used = 0
    return node


def _arm_z(world_z: float) -> float:
    return world_z - ARM_BASE_WORLD_Z


# ── IK reachability ─────────────────────────────────────────────────────────

def test_compute_ik_returns_six_joints_for_reachable(arm):
    q = arm._compute_ik(-0.70, 0.10, _arm_z(PICK_WORLD_Z))
    assert q is not None
    assert len(q) == 6
    assert all(isinstance(v, float) for v in q)


def test_compute_ik_returns_none_for_far_corner(arm):
    """Old spawn corner (R ~= 0.87 m) must fail LOUDLY — no clamping."""
    q = arm._compute_ik(-0.78, 0.38, _arm_z(PICK_WORLD_Z))
    assert q is None


def test_is_reachable_boundary(arm):
    z = _arm_z(PICK_WORLD_Z)
    assert arm.is_reachable(-0.70, 0.10, z) is True
    assert arm.is_reachable(-0.78, 0.38, z) is False


def test_is_reachable_below_wrist_offset(arm):
    """Targets closer than the d4 wrist offset are unsolvable (asin domain)."""
    assert arm.is_reachable(0.0, 0.0, 0.05) is False


def test_is_reachable_comfortable_target_keeps_margin(arm):
    z = _arm_z(PICK_WORLD_Z)
    assert arm.is_reachable(-0.70, 0.10, z, margin=REACH_MARGIN_M) is True


# ── Pick-region filter ──────────────────────────────────────────────────────

def test_on_pick_table_inside(controller):
    assert controller._on_pick_table(-0.70, 0.00) is True
    assert controller._on_pick_table(-0.65, -0.30) is True


def test_on_pick_table_excludes_sort_zones(controller):
    """Placed objects on Table 1 must never be re-picked."""
    assert controller._on_pick_table(-0.28, -0.12) is False
    assert controller._on_pick_table(-0.28, 0.24) is False


def test_on_pick_table_excludes_outside(controller):
    assert controller._on_pick_table(-0.30, 0.00) is False
    assert controller._on_pick_table(-0.70, 0.60) is False
    assert controller._on_pick_table(-1.10, 0.00) is False


def test_pick_region_bounds_are_sane():
    assert PICK_REGION_X_MIN < PICK_REGION_X_MAX
    assert PICK_REGION_Y_MIN < PICK_REGION_Y_MAX


# ── Object reachability (grasp + lift waypoints) ────────────────────────────

def test_object_reachable_for_typical_object(controller):
    assert controller._object_reachable(-0.70, 0.10) is True


def test_object_unreachable_for_far_corner(controller):
    assert controller._object_reachable(-0.78, 0.38) is False


def test_object_reachable_checks_lift_waypoint_too(controller):
    """is_reachable is consulted for BOTH pick and pick+LIFT heights."""
    seen = []

    def fake_is_reachable(x, y, z, margin=0.0):
        seen.append(z)
        return True

    controller.is_reachable = fake_is_reachable
    assert controller._object_reachable(-0.70, 0.10) is True
    pick_z = _arm_z(PICK_WORLD_Z)
    assert pick_z in seen
    assert pick_z + LIFT_HEIGHT_M in seen


# ── Cross-scan registry ─────────────────────────────────────────────────────

def test_registry_registers_and_returns_nearest_first(controller):
    dets = [('red_cube', -0.68, -0.15), ('red_cube', -0.60, 0.30)]
    candidates = controller._update_registry(dets)
    assert len(candidates) == 2
    d0 = math.hypot(candidates[0]['x'], candidates[0]['y'])
    d1 = math.hypot(candidates[1]['x'], candidates[1]['y'])
    assert d0 <= d1
    # (−0.60, 0.30) at R≈0.671 is closer to the base than (−0.68, −0.15)
    # at R≈0.696, so it sorts first
    assert (candidates[0]['x'], candidates[0]['y']) == (-0.60, 0.30)


def test_registry_dedups_within_match_radius(controller):
    """Two detections of the same object collapse to one registry entry."""
    controller._update_registry([('red_cube', -0.70, 0.20)])
    controller._update_registry([('red_cube', -0.70 + 0.03, 0.20)])
    assert len(controller._registry) == 1


def test_registry_refreshes_position_on_rescan(controller):
    controller._update_registry([('red_cube', -0.70, 0.20)])
    assert len(controller._registry) == 1
    # Same object re-detected slightly shifted: position refreshed, no duplicate
    controller._update_registry([('red_cube', -0.702, 0.198)])
    assert len(controller._registry) == 1
    assert abs(controller._registry[0]['x'] + 0.702) < 1e-9


def test_registry_ignores_match_radius_boundary(controller):
    controller._update_registry([('red_cube', -0.70, 0.20)])
    far = (-0.70 + MATCH_RADIUS_M + 0.05, 0.20)
    controller._update_registry([('red_cube', *far)])
    assert len(controller._registry) == 2


def test_registry_skips_sort_zone_detections(controller):
    """Re-scans see placed objects on Table 1 — they must not become candidates."""
    candidates = controller._update_registry([('red_cube', -0.28, -0.12)])
    assert candidates == []
    assert controller._registry == []


def test_registry_skips_unreachable_and_warns(controller):
    candidates = controller._update_registry([('red_cube', -0.80, 0.30)])
    assert candidates == []
    assert controller._registry == []
    controller.get_logger.return_value.warning.assert_called()


def test_registry_excludes_done_and_blacklisted(controller):
    controller._update_registry([('red_cube', -0.70, 0.20),
                                 ('blue_cylinder', -0.65, -0.10)])
    assert len(controller._registry) == 2
    controller._registry[0]['done'] = True
    controller._registry[1]['blacklisted'] = True
    assert controller._update_registry([]) == []


# ── Zone slot assignment ────────────────────────────────────────────────────

def test_zone_slots_assigned_in_order(controller):
    assert controller._next_zone('red_cube') == (-0.28, -0.12)
    controller._mark_zone_used('red_cube')
    assert controller._next_zone('red_cube') == (-0.28, -0.24)
    assert controller._slot_index('red_cube') == 1
    # Blue side is independent
    assert controller._next_zone('blue_cylinder') == (-0.28, 0.12)
    assert controller._slot_index('blue_cylinder') == 0


def test_zone_slots_exhausted(controller):
    for _ in range(MAX_SLOTS):
        assert controller._next_zone('blue_cylinder') is not None
        controller._mark_zone_used('blue_cylinder')
    assert controller._next_zone('blue_cylinder') is None
    assert controller._slot_index('blue_cylinder') == MAX_SLOTS


# ── Spawner reach constraint ────────────────────────────────────────────────

def test_spawner_positions_within_reach_envelope(spawner):
    """Every sampled position must satisfy the arm pick-radius envelope."""
    random.seed(42)
    positions = spawner._generate_positions(4)
    assert len(positions) == 4
    for (x, y) in positions:
        assert x <= spawner.X_SPAWN_INNER
        assert x >= spawner._reach_constrained_x_max(y)
        assert math.hypot(x, y) <= spawner.R_PICK_MAX + 1e-9
        assert spawner.SPAWN_Y_MIN <= y <= spawner.SPAWN_Y_MAX


def test_spawner_respects_min_separation(spawner):
    random.seed(7)
    positions = spawner._generate_positions(4)
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dist = math.hypot(positions[i][0] - positions[j][0],
                              positions[i][1] - positions[j][1])
            assert dist >= spawner.MIN_SEP


def test_spawner_reach_helper_matches_pick_radius(spawner):
    """Sanity: at y=0 the outer spawn edge must be exactly -R_PICK_MAX."""
    assert spawner._reach_constrained_x_max(0.0) == pytest.approx(-spawner.R_PICK_MAX)


def test_spawner_old_corner_is_outside_envelope(spawner):
    """The legacy (-0.78, +-0.38) corner that caused stretched poses is excluded."""
    assert math.hypot(-0.78, 0.38) > spawner.R_PICK_MAX


def test_fallback_positions_are_all_reachable(spawner):
    positions = spawner._fallback_positions(4)
    assert len(positions) == 4
    for (x, y) in positions:
        assert math.hypot(x, y) <= spawner.R_PICK_MAX + 1e-9


def test_fallback_rejects_impossible_counts(spawner):
    with pytest.raises(RuntimeError):
        spawner._fallback_positions(6)


# ── Live-run regression: IK base joint must respect the URDF ±3.14 limit ────
# Coordinates from the 20:35 failure log: pick=(-0.676, 0.105) produced
# theta1 = 3.14817 — above the URDF arm_base_joint limit (3.14) — and every
# MoveIt goal answered FAILURE(99999).

def test_normalize_angle_wraps_into_pi_band(arm):
    assert arm._normalize_angle(3.14817) == pytest.approx(3.14817 - 2 * math.pi, abs=1e-9)
    assert arm._normalize_angle(-3.20) == pytest.approx(-3.20 + 2 * math.pi, abs=1e-9)
    assert arm._normalize_angle(math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert -math.pi <= arm._normalize_angle(17.0) < math.pi


def test_compute_ik_live_failure_target_now_within_limits(arm):
    q = arm._compute_ik(-0.676, 0.105, 0.05)
    assert q is not None
    assert all(abs(v) <= 3.14 + 1e-9 for v in q)


def test_compute_ik_picks_correct_d4_branch(arm):
    """atan2(0.105, -0.676) + asin(d4/R) = 3.1477 wraps to -3.1355.
    The old min(|theta1|) rule picked the mirror branch (2.8276), which
    FK-verification showed lands the TCP 218 mm (= 2*d4) from the target
    for every object with y > 0."""
    q = arm._compute_ik(-0.676, 0.105, 0.05)
    assert q is not None
    assert q[0] == pytest.approx(-3.1355, abs=1e-3)


def test_compute_ik_spawn_band_sweep_within_limits(arm):
    """Every pickable spawn position must yield in-limit joints or loud None."""
    for y in (-0.38, -0.20, 0.0, 0.20, 0.38):
        for x in (-0.66, -0.75):
            q = arm._compute_ik(x, y, 0.05)
            if q is None:
                continue  # unreachable envelope corner — covered by None tests
            assert all(abs(v) <= 3.14 + 1e-9 for v in q)


def test_duplicate_server_hint(arm):
    assert arm._duplicate_server_hint(1) is None
    hint = arm._duplicate_server_hint(2)
    assert hint is not None and 'vision-clean' in hint


def test_moveit_error_code_names(arm):
    assert arm._ERROR_NAMES[1] == 'SUCCESS'
    assert arm._ERROR_NAMES[99999] == 'FAILURE'
    assert arm._ERROR_NAMES[10001] == 'PLANNING_FAILED'
    assert arm._ERROR_NAMES[10004] == 'CONTROL_FAILED'
