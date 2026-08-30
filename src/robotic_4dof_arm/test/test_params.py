import os
import yaml
import pytest


PARAMS_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'arm_moveit_config', 'config', 'robot_params.yaml')

@pytest.fixture
def params():
    with open(PARAMS_PATH, 'r') as f:
        return yaml.safe_load(f)

def test_params_load(params):
    assert params is not None

def test_robot_section(params):
    assert 'use_sim_time' in params['robot']
    assert isinstance(params['robot']['use_sim_time'], bool)

def test_motion_section(params):
    required = ['max_velocity_scaling_factor', 'max_acceleration_scaling_factor', 'allowed_planning_time', 'num_planning_attempts', 'replan_attempts', 'max_retries', 'position_tolerance']

    for key in required:
        assert key in params['motion'], f"Missing key '{key}' in motion section"
    assert 0.0 < params['motion']['max_velocity_scaling_factor'] <= 1.0
    assert params['motion']['max_retries'] >= 1

def test_workspace_section(params):
    ws = params['workspace']
    assert len(ws['min_corner']) == 3
    assert len(ws['max_corner']) == 3
    for i in range(3):
        assert ws['min_corner'][i] < ws['max_corner'][i], f"min_corner[{i}] should be less than max_corner[{i}]"

def test_joint_names(params):
    arm_joints = list(params['joints']['arm'].keys())
    assert len(arm_joints) == 6, "Expected 6 arm joints"
    gripper_joints = list(params['joints']['gripper'].keys())
    assert len(gripper_joints) == 2, "Expected 2 gripper joints"

def test_home_position(params):
    home = params['joints']['home_position']
    arm_joints = list(params['joints']['arm'].keys())
    gripper_joints = list(params['joints']['gripper'].keys())
    for joint in arm_joints + gripper_joints:
        assert joint in home, f"Home position missing for joint '{joint}'"
