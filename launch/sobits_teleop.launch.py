from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, AndSubstitution, OrSubstitution, NotSubstitution
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
        default_value='127.0.0.1',
        description='ROS IP address for ros_tcp_endpoint (Meta Quest controllers)'
    )
    declare_use_ds4drv_cmd = DeclareLaunchArgument(
        'use_ds4drv',
        default_value='True',
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
        description='Launch MoveIt arm controller for Quest arm teleoperation'
    )
    declare_use_servo_cmd = DeclareLaunchArgument(
        'use_servo',
        default_value='false',
        description='Use MoveIt Servo pose tracking instead of the plan-and-replace '
                    'moveit_arm_controller backend (device=quest && use_moveit must also be true)'
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
        parameters=[
            [robot_config, '.yaml'],
            [controller_config, '.yaml'],
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    ds4drv_cmd = ExecuteProcess(
        cmd=['ds4drv'],
        output='screen',
        condition=IfCondition(AndSubstitution(EqualsSubstitution(LaunchConfiguration('device'), 'ps4'), EqualsSubstitution(LaunchConfiguration('use_ds4drv'), 'True')))
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
        condition=IfCondition(OrSubstitution(EqualsSubstitution(LaunchConfiguration('device'), 'ps4'), EqualsSubstitution(LaunchConfiguration('device'), 'ps5')))
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
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
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
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'keyboard'))
    )

    # Arm-tracking backends (device=quest && use_moveit only; use_servo picks
    # exactly one). Both live in launch/include/ and inject the arm identity
    # from quest.yaml / robot.yaml themselves.
    launch_include_dir = os.path.join(
        get_package_share_directory(pkg_name), 'launch', 'include')
    backend_args = {
        'robot_name': robot_name,
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
            EqualsSubstitution(LaunchConfiguration('device'), 'quest'),
            LaunchConfiguration('use_moveit')
        ), LaunchConfiguration('use_servo')))
    )

    # Launch quest usb network setup script (only needed on Linux, and only if using Quest controllers)
    # User needs to allow debugging access on the Quest and connect it via USB for this to work
    usb_network_setup_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sudo adb reverse tcp:10000 tcp:10000'],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
    )

    # Kill any process still holding port 10000 (stale quest_node from a previous launch)
    kill_stale_quest = ExecuteProcess(
        cmd=['bash', '-c', 'fuser -k 10000/tcp 2>/dev/null || true'],
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
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
        kill_stale_quest,
        sobits_teleop_node,
        arm_backend_plan_launch,
        arm_backend_servo_launch,
        ds4drv_cmd,
        joystick_node,
        quest_node,
        keyboard_node,
        usb_network_setup_cmd
    ])
