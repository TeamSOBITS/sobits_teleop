from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "sobits_teleop"

    # Launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this human control stack'
    )
    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='keyboard',
        description='Input device type: keyboard, ps4, ps5, quest'
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='rcjo2025_arena',
        description='Gazebo world name'
    )
    declare_model_name_cmd = DeclareLaunchArgument(
        'model_name',
        default_value='gz_human',
        description='Human model name in Gazebo'
    )
    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='-2.0',
        description='Human spawn x position'
    )
    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='1.5',
        description='Human spawn y position'
    )
    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Human spawn z position'
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Human spawn yaw angle'
    )

    # LaunchConfiguration handles runtime values
    namespace = LaunchConfiguration('namespace')
    device = LaunchConfiguration('device')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    controller_config = PathJoinSubstitution([
        get_package_share_directory(pkg_name),
        'config',
        'gz_human',
        device,
    ])

    # Main teleop node (uses mapping_yaml parameter)
    sobits_teleop_node = Node(
        package=pkg_name,
        executable='sobits_teleop',
        name='sobits_teleop',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'robot_topic_name.joint_states_topic': ['/', model_name, '/joint_states'],
                'robot_topic_name.cmd_vel_topic': 'cmd_vel',
            },
            [controller_config, '.yaml'],
        ],
    )

    # Human cmd_vel controller
    human_cmd_vel_controller_node = Node(
        package='gz_human_interaction',
        executable='human_cmd_vel_controller.py',
        name='human_cmd_vel_controller',
        namespace=namespace,
        output='screen',
        parameters=[
            {'world_name': world_name},
            {'model_name': model_name},
            {'initial_x': x},
            {'initial_y': y},
            {'initial_z': z},
            {'initial_yaw': yaw},
            {'cmd_vel_topic': 'cmd_vel'},
        ],
    )

    # joy_linux node for joy controllers (ps4, ps5)
    joystick_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joystick_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'dev': '/dev/input/js0'},
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0}
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'ps4') or EqualsSubstitution(LaunchConfiguration('device'), 'ps5'))
    )

    # quest node for meta quest controllers
    quest_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='quest_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'quest'))
    )

    # keyboard node for keyboard teleop
    keyboard_node = Node(
        package='keyboard_joy',
        executable='joy_node',
        name='keyboard_node',
        namespace=namespace,
        output='screen',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('device'), 'keyboard'))
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_device_cmd,
        declare_world_name_cmd,
        declare_model_name_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        sobits_teleop_node,
        human_cmd_vel_controller_node,
        joystick_node,
        quest_node,
        keyboard_node,
    ])
