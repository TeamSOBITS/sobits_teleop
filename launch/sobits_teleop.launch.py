from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        # default_value="hsr_sim",
        # default_value="hsrb_robot"
        description='Robot name used to select config subfolder'
    )
    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='ps4', # ps5
        # default_value='keyboard',
        # default_value='quest',
        description='Input device type: ps4, quest, keyboard'
    )
    declare_ros_ip_cmd = DeclareLaunchArgument(
        'ros_ip',
        default_value='0.0.0.0',
        description='ROS IP address'
    )

    # LaunchConfiguration handles runtime values
    robot_name = LaunchConfiguration('robot_name')
    device = LaunchConfiguration('device')
    ros_ip = LaunchConfiguration('ros_ip')

    config_file = PathJoinSubstitution([
        get_package_share_directory(pkg_name), 
        'config',
        robot_name,
        device 
    ])
    keyboard_config_file = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'config',
        'keyboard_mappings.yaml'
    ])

    # Main teleop node (uses mapping_yaml parameter)
    sobits_teleop_node = Node(
        package=pkg_name,
        executable='sobits_teleop',
        name='sobits_teleop',
        output='screen',
        parameters=[{'mapping_yaml': [config_file, '.yaml']}],
    )

    # joy_linux node for PS controllers (ps4 / ps5)
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen',
        parameters=[
            {'device_id': 0},
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0}
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'ps4'))
    )

    # quest custom node
    quest_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='quest_joy_node',
        output='screen',
        parameters=[{'ROS_IP': ros_ip}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
    )

    # keyboard: launch an external keyboard teleop node (default: teleop_twist_keyboard)
    keyboard_node = Node(
        package='keyboard_joy',
        executable='joy_node',
        name='keyboard_joy_node',
        output='screen',
        parameters=[{'mapping_yaml': keyboard_config_file}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'keyboard'))
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_device_cmd,
        declare_ros_ip_cmd,
        sobits_teleop_node,
        joy_node,
        quest_node,
        keyboard_node,
    ])
