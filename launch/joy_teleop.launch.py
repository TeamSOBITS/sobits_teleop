from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = "sobits_teleop"
    robot_name = "sobit_home"

    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        "config",
        robot_name,
        "ps4.yaml"
    )

    joy_teleop_node = Node(
        package=pkg_name,
        executable="joy_teleop",
        name="joy_teleop",
        output="screen",
        parameters=[{"mapping_yaml": config_file}],
    )

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[
            {"device_id": 0},
            {"deadzone": 0.05},
            {"autorepeat_rate": 20.0},
        ]
    )
    return LaunchDescription([
        joy_teleop_node,
        joy_node
    ])
