import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, EqualsSubstitution, LaunchConfiguration, NotSubstitution,
    PathJoinSubstitution, OrSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'sobits_teleop'

    # Launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='sobit_home',
        description='Robot name used to select configuration file'
    )
    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='ps4',
        description='Input device type: ps4, quest, keyboard'
    )
    declare_joystick_device_cmd = DeclareLaunchArgument(
        'joystick_device',
        default_value='/dev/input/js0',
        description='Joystick device path for joy_linux node'
    )
    declare_ros_ip_cmd = DeclareLaunchArgument(
        'ros_ip',
        default_value='127.0.0.1',
        description='ROS IP address for ros_tcp_endpoint (Meta Quest controllers)'
    )
    declare_use_ds4drv_cmd = DeclareLaunchArgument(
        'use_ds4drv',
        default_value='false',
        description='Whether to launch ds4drv for PS4 controller support'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock — set true when running with gz_minimal'
    )
    declare_use_moveit_cmd = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Enable arm backend (Quest plan/servo or PS4/keyboard twist servo)'
    )
    declare_use_servo_cmd = DeclareLaunchArgument(
        'use_servo',
        default_value='false',
        description='Use MoveIt Servo pose tracking instead of the plan-and-replace '
                    'moveit_arm_controller backend (use_moveit must also be true)'
    )

    # LaunchConfiguration handles runtime values
    robot_name = LaunchConfiguration('robot_name')
    device = LaunchConfiguration('device')
    joystick_device = LaunchConfiguration('joystick_device')
    ros_ip = LaunchConfiguration('ros_ip')
    use_ds4drv = LaunchConfiguration('use_ds4drv')
    common_config = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'config',
        robot_name,
        'common'
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
        parameters=[
            [common_config, '.yaml'],
            [controller_config, '.yaml'],
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'servo_twist_only': ParameterValue(AndSubstitution(
                AndSubstitution(LaunchConfiguration('use_moveit'), LaunchConfiguration('use_servo')),
                OrSubstitution(EqualsSubstitution(device, 'ps4'),
                               EqualsSubstitution(device, 'keyboard'))), value_type=bool)},
        ],
    )

    # Shared input-driver include so sobits_vla_deploy can reuse the same controllers.
    controller_input_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(pkg_name),
            'launch', 'include', 'controller_input.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'device': device,
            'joystick_device': joystick_device,
            'ros_ip': ros_ip,
            'use_ds4drv': use_ds4drv,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Arm-tracking backends (device=quest && use_moveit; use_servo picks exactly one).
    launch_include_dir = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'include')
    backend_args = {
        'robot_name': robot_name,
        'device': device,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }.items()

    arm_backend_plan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_include_dir, 'arm_backend_plan.launch.py')),
        launch_arguments=backend_args,
        condition=IfCondition(AndSubstitution(AndSubstitution(
            EqualsSubstitution(LaunchConfiguration('device'), 'quest'),
            LaunchConfiguration('use_moveit')
        ), NotSubstitution(LaunchConfiguration('use_servo'))))
    )

    arm_backend_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_include_dir, 'arm_backend_servo.launch.py')),
        launch_arguments=backend_args,
        condition=IfCondition(AndSubstitution(AndSubstitution(
            OrSubstitution(
                OrSubstitution(EqualsSubstitution(device, 'quest'),
                               EqualsSubstitution(device, 'ps4')),
                EqualsSubstitution(device, 'keyboard')),
            LaunchConfiguration('use_moveit')
        ), LaunchConfiguration('use_servo')))
    )

    # A dedicated twist controller shares the existing Joy topic. Quest keeps
    # its original PoseStamped bridge; PS4/keyboard use TwistStamped.
    twist_teleop_node = Node(
        package=pkg_name,
        executable='servo_twist_teleop',
        name='servo_twist_teleop',
        namespace=robot_name,
        output='screen',
        parameters=[PathJoinSubstitution([
            get_package_share_directory(pkg_name), 'config', robot_name,
            ['servo_twist_', device, '.yaml']]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(AndSubstitution(
            AndSubstitution(LaunchConfiguration('use_moveit'),
                            LaunchConfiguration('use_servo')),
            OrSubstitution(EqualsSubstitution(device, 'ps4'),
                           EqualsSubstitution(device, 'keyboard')))),
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_device_cmd,
        declare_joystick_device_cmd,
        declare_ros_ip_cmd,
        declare_use_ds4drv_cmd,
        declare_use_sim_time_cmd,
        declare_use_moveit_cmd,
        declare_use_servo_cmd,
        sobits_teleop_node,
        arm_backend_plan_launch,
        arm_backend_servo_launch,
        twist_teleop_node,
        controller_input_launch
    ])
