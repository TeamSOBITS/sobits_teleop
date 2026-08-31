"""
Plan-and-replace arm-tracking backend (moveit_arm_controller).

Arm identity (arms, planning groups, target frames, controller topics) is
declared once in quest.yaml / robot.yaml and injected here; the backend's own
yaml (arm_backend_plan.yaml) carries tuning only.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _make_nodes(context, *args, **kwargs):
    import yaml as pyyaml

    robot_name = LaunchConfiguration('robot_name').perform(context)
    cfg_dir = os.path.join(
        get_package_share_directory('sobits_teleop'), 'config', robot_name)

    with open(os.path.join(cfg_dir, 'quest.yaml')) as f:
        quest_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    with open(os.path.join(cfg_dir, 'robot.yaml')) as f:
        robot_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    traj_topics = robot_params['robot_topic_name']['joint_trajectory_topic']

    kinematics_file_path = os.path.join(
        get_package_share_directory(f'{robot_name}_moveit_config'),
        'config', 'kinematics.yaml')
    with open(kinematics_file_path) as f:
        kinematics_raw = pyyaml.safe_load(f)
    kinematics_params = {
        f'robot_description_kinematics.{group}.{key}': value
        for group, group_params in kinematics_raw.items()
        for key, value in group_params.items()
    }

    quest_control = quest_params['quest_control']
    arm_params = {}
    arms = []
    for ctrl_name in quest_control.get('controllers', quest_control.get('controller', [])):
        block = quest_control.get(ctrl_name, {})
        if isinstance(block, dict) and 'arm' in block:
            arm = block['arm']
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
            kinematics_params,
            arm_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='sobit_home'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_make_nodes),
    ])
