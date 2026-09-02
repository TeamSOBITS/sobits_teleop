"""
Plan-and-replace arm-tracking backend (moveit_arm_controller).

Arm identity (arms, planning groups, target frames, controller topics) is
declared once in quest.yaml / common.yaml and injected here; the backend's own
yaml (arm_backend_plan.yaml) carries tuning only.
"""

import os

from ament_index_python.packages import (
    get_package_share_directory, PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    import yaml as pyyaml

    robot_name = LaunchConfiguration('robot_name').perform(context)
    cfg_dir = os.path.join(
        get_package_share_directory('sobits_teleop'), 'config', robot_name)

    with open(os.path.join(cfg_dir, 'quest.yaml')) as f:
        quest_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    with open(os.path.join(cfg_dir, 'common.yaml')) as f:
        robot_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    traj_topics = robot_params['robot_topic_name']['joint_trajectory_topic']

    # Without robot_description_kinematics computeCartesianPath() returns 0.0 and
    # tracking silently never moves. Node() flattens the nested dict on its own.
    kinematics_params = {}
    try:
        kin_path = os.path.join(
            get_package_share_directory(f'{robot_name}_moveit_config'),
            'config', 'kinematics.yaml')
        with open(kin_path) as f:
            kin_yaml = pyyaml.safe_load(f) or {}
        kinematics_params = {'robot_description_kinematics': kin_yaml}
    except (PackageNotFoundError, FileNotFoundError):
        # Fall back to whatever the node can fetch from move_group.
        pass

    controller_cartesian = quest_params['controller_cartesian']
    arm_params = {}
    arms = []
    # An arm group is one that names an end effector; the group name IS the
    # planning group, matching how the node itself identifies an arm.
    for arm in controller_cartesian.get('groups_name', []):
        block = controller_cartesian.get(arm, {})
        if isinstance(block, dict) and 'end_effector_frame_name' in block:
            arms.append(arm)
            arm_params[f'arm_teleop.{arm}.planning_group'] = arm
            arm_params[f'arm_teleop.{arm}.target_frame'] = block['target_frame_name']
            arm_params[f'arm_teleop.{arm}.base_frame'] = 'base_footprint'
            arm_params[f'arm_teleop.{arm}.trajectory_topic'] = traj_topics[arm]
    arm_params['arm_teleop.arms'] = arms

    return [Node(
        package='sobits_teleop',
        executable='moveit_arm_controller',
        name='moveit_arm_controller',
        output='screen',
        namespace=robot_name,
        parameters=[
            os.path.join(cfg_dir, 'arm_backend_plan.yaml'),
            arm_params,
            kinematics_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='sobit_home'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_make_nodes),
    ])
