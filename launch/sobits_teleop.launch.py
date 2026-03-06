from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = "sobits_teleop"

    # Launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='sobit_home',
        # default_value="sobit_pro",
        # default_value="sobit_edu",
        # default_value="sobit_mini",
        # default_value="sobit_light",
        description='Robot name used to select configuration file'
    )
    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='ps4', 
        # default_value='ps5',
        # default_value='quest',
        # default_value='keyboard',
        description='Input device type: ps4, quest, keyboard'
    )
    declare_joystick_device_cmd = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path for joy_linux node'
    )
    declare_ros_ip_cmd = DeclareLaunchArgument(
        'ros_ip',
        default_value='0.0.0.0',
        description='ROS IP address for ros_tcp_endpoint (Meta Quest controllers)'
    )
    declare_use_ds4drv_cmd = DeclareLaunchArgument(
        'use_ds4drv',
        default_value='True',
        description='Whether to launch ds4drv for PS4 controller support'
    )

    # LaunchConfiguration handles runtime values
    robot_name = LaunchConfiguration('robot_name')
    device = LaunchConfiguration('device')
    joystick_device = LaunchConfiguration('joystick_device')
    ros_ip = LaunchConfiguration('ros_ip')
    use_ds4drv = LaunchConfiguration('use_ds4drv')
    robot_config = PathJoinSubstitution([
        get_package_share_directory(pkg_name), 
        'config',
        robot_name,
        'robot' 
    ])

    controller_config = PathJoinSubstitution([
        get_package_share_directory(pkg_name), 
        'config',
        robot_name,
        device 
    ])

    # Main teleop node (loads parameters from YAML)
    sobits_teleop_node = Node(
        package=pkg_name,
        executable='sobits_teleop',
        name='sobits_teleop',
        output='screen',
        namespace=robot_name,
        parameters=[[robot_config, '.yaml'],
                    [controller_config, '.yaml']],
    )

    ds4drv_cmd = ExecuteProcess(
        cmd=['ds4drv'],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'ps4') and EqualsSubstitution(LaunchConfiguration('use_ds4drv'), 'True'))
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
            {'autorepeat_rate': 20.0}
        ],
        arguments=['--ros-args', '--log-level', 'fatal'],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'ps4') or EqualsSubstitution(LaunchConfiguration('device'), 'ps5'))
    )

    # quest node for meta quest controllers
    quest_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='quest_node',
        output='screen',
        namespace=robot_name,
        parameters=[{'ROS_IP': ros_ip}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
    )

    # keyboard node for keyboard teleop
    keyboard_node = Node(
        package='keyboard_joy',
        executable='joy_node',
        name='keyboard_node',
        output='screen',
        namespace=robot_name,
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'keyboard'))
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_device_cmd,
        declare_joystick_device_cmd,
        declare_ros_ip_cmd,
        declare_use_ds4drv_cmd,
        sobits_teleop_node,
        ds4drv_cmd,
        joystick_node,
        quest_node,
        keyboard_node,
    ])
