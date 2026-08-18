import os
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import launch
import launch.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start sobits_teleop with robot.yaml + quest.yaml and wait for it to load."""
    share_dir = get_package_share_directory('sobits_teleop')
    robot_yaml = os.path.join(share_dir, 'config', 'sobit_home', 'robot.yaml')
    quest_yaml = os.path.join(share_dir, 'config', 'sobit_home', 'quest.yaml')
    exe = os.path.join(
        get_package_prefix('sobits_teleop'), 'lib', 'sobits_teleop', 'sobits_teleop')

    # Fixed unused domain ID so this test doesn't collide with anything else running.
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = '42'

    sobits_teleop_process = launch.actions.ExecuteProcess(
        cmd=[exe, '--ros-args',
             '--params-file', robot_yaml,
             '--params-file', quest_yaml],
        additional_env=env,
        output='screen',
    )

    return launch.LaunchDescription([
        sobits_teleop_process,
        launch_testing.actions.ReadyToTest(),
    ]), {'sobits_teleop_process': sobits_teleop_process}


class TestNodeStartup(unittest.TestCase):

    def test_loaded_quest_params(self, proc_output):
        # rclcpp logging (RCLCPP_INFO) goes to stderr, not stdout.
        proc_output.assertWaitFor(
            'Loaded 2 quest arm and 2 quest hand parameters',
            timeout=8.0, stream='stderr')

    def test_created_arm_track_publisher(self, proc_output):
        proc_output.assertWaitFor(
            "Created arm track publisher for 'arm_left'",
            timeout=8.0, stream='stderr')
        # Let the constructor finish (hand pose client, timers) before SIGINT — an
        # early signal races node construction and aborts with RCLError.
        proc_output.assertWaitFor(
            "Created hand pose client for 'hand_left'",
            timeout=8.0, stream='stderr')


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_exit_ok(self, proc_info, sobits_teleop_process):
        # keep_alive processes are SIGINT'd by the test framework at shutdown.
        launch_testing.asserts.assertExitCodes(
            proc_info, process=sobits_teleop_process)
