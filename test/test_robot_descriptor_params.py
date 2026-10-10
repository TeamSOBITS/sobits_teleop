import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'include'))
import robot_descriptor_params as rdp  # noqa: E402,I100

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'sobit_home')


@pytest.fixture(scope='module')
def desc():
    d = rdp.try_load('sobit_home')
    if d is None:
        pytest.skip('sobit_home descriptor not installed')
    return d


def _params(name):
    return rdp.read_params(os.path.join(CONFIG, f'{name}.yaml'))


def test_descriptor_params_sobit_home(desc):
    p = rdp.descriptor_params(desc)
    assert p['robot_topic_name.base_frame'] == 'base_footprint'
    assert p['robot_topic_name.joint_states_topic'] == 'joint_states'
    assert p['robot_topic_name.cmd_vel_topic'] == 'cmd_vel'
    assert p['robot_topic_name.joint_trajectory_topic.body'] == \
        'body_position_controller/joint_trajectory'
    for g in desc.groups:
        assert f'robot_topic_name.joint_trajectory_topic.{g.name}' in p


@pytest.mark.parametrize('device', ['ps4', 'keyboard', 'quest'])
def test_shipped_device_yaml_valid(desc, device):
    rdp.validate_device(desc, _params(device))


def test_unknown_joint_rejected(desc):
    params = _params('keyboard')
    params['controller_joints']['head']['joints_name'].append('head_bogus_joint')
    with pytest.raises(RuntimeError, match='head_bogus_joint') as e:
        rdp.validate_device(desc, params)
    assert 'head_pan_joint' in str(e.value)


def test_unknown_group_rejected(desc):
    params = _params('keyboard')
    params['controller_joints']['groups_name'].append('tail')
    with pytest.raises(RuntimeError, match="unknown group 'tail'") as e:
        rdp.validate_device(desc, params)
    assert 'arm_left' in str(e.value)


def test_ee_frames(desc):
    rdp.check_ee_frames(desc, 'arm_left', 'hand_left_end_effector_link', 'left_target_link')
    with pytest.raises(RuntimeError, match='arm_left'):
        rdp.check_ee_frames(desc, 'arm_left', 'hand_right_end_effector_link', 'left_target_link')


def test_shadowed_keys(desc):
    common = {'teleop_rate_hz': 100.0, 'robot_topic_name': {'base_frame': 'x'}}
    assert rdp.shadowed_keys(common, rdp.descriptor_params(desc)) == [
        'robot_topic_name.base_frame']


def test_fallback_without_descriptor(tmp_path, capsys):
    common = tmp_path / 'common.yaml'
    common.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_topic_name': {
        'base_frame': 'base_link',
        'joint_trajectory_topic': {'arm': 'arm_controller/joint_trajectory'}}}}}))
    assert rdp.try_load('no_such_robot_xyz') is None
    assert 'no robot descriptor' in capsys.readouterr().out
    topics, base_frame = rdp.backend_topics(None, rdp.read_params(common))
    assert topics == {'arm': 'arm_controller/joint_trajectory'}
    assert base_frame == 'base_link'
