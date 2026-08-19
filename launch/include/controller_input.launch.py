"""
Controller input drivers only — no teleop control node.

Brings up the device layer that publishes /<robot_name>/joy for the chosen
device (quest via ros_tcp_endpoint, ps4/ps5 via joy_linux + optional ds4drv,
keyboard via keyboard_joy), including the Quest USB network setup.

Shared by two consumers:
  * sobits_teleop.launch.py           (adds the teleop control node on top)
  * sobits_vla_deploy launches        (VLA drives the arm; only the input
                                       devices + gamepad client are needed)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    OrSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_name = LaunchConfiguration('robot_name')
    joystick_device = LaunchConfiguration('joystick_device')
    ros_ip = LaunchConfiguration('ros_ip')

    ds4drv_cmd = ExecuteProcess(
        cmd=['ds4drv'],
        output='screen',
        condition=IfCondition(AndSubstitution(
            EqualsSubstitution(LaunchConfiguration('device'), 'ps4'),
            EqualsSubstitution(LaunchConfiguration('use_ds4drv'), 'true'))),
    )

    # joy_linux node for joy controllers (ps4, ps5)
    joystick_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick_node',
        output='screen',
        namespace=robot_name,
        parameters=[
            {'dev': joystick_device},
            {'deadzone': 0.05},
            {'autorepeat_rate': 100.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', 'fatal'],
        condition=IfCondition(OrSubstitution(
            EqualsSubstitution(LaunchConfiguration('device'), 'ps4'),
            EqualsSubstitution(LaunchConfiguration('device'), 'ps5'))),
    )

    # quest node for meta quest controllers
    quest_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='quest_node',
        output='screen',
        namespace=robot_name,
        parameters=[
            {'ROS_IP': ros_ip},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('device'), 'quest')),
    )

    # keyboard node for keyboard teleop
    keyboard_node = Node(
        package='keyboard_joy',
        executable='joy_node',
        name='keyboard_node',
        output='screen',
        namespace=robot_name,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('device'), 'keyboard')),
    )

    # Quest USB network setup (adb reverse) + stale endpoint cleanup.
    usb_network_setup_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sudo adb reverse tcp:10000 tcp:10000'],
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('device'), 'quest')),
    )
    kill_stale_quest = ExecuteProcess(
        cmd=['bash', '-c', 'fuser -k 10000/tcp 2>/dev/null || true'],
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('device'), 'quest')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='sobit_home',
            description='Namespace the joy topic is published under'),
        DeclareLaunchArgument(
            'device', default_value='ps4',
            description='Input device type: ps4, ps5, quest, keyboard'),
        DeclareLaunchArgument(
            'joystick_device', default_value='/dev/input/js0',
            description='Joystick device path for joy_linux node'),
        DeclareLaunchArgument(
            'ros_ip', default_value='127.0.0.1',
            description='ROS IP for ros_tcp_endpoint (Meta Quest)'),
        DeclareLaunchArgument(
            'use_ds4drv', default_value='false',
            description='Whether to launch ds4drv for PS4 controller support'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        kill_stale_quest,
        ds4drv_cmd,
        joystick_node,
        quest_node,
        keyboard_node,
        usb_network_setup_cmd,
    ])
