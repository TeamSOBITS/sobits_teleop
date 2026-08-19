import os
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import launch
import launch.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
import yaml


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


def _target_arm_groups():
    """Arm groups the device yaml declares, so the test tracks the config."""
    share_dir = get_package_share_directory('sobits_teleop')
    with open(os.path.join(share_dir, 'config', 'sobit_home', 'quest.yaml')) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    return [g for g in params['controller_cartesian']['groups_name'] if g.startswith('arm_')]


class TestNodeStartup(unittest.TestCase):

    def test_publishes_track_enable_per_configured_arm(self, proc_output):
        """Assert on the graph, not log text: one enable topic per configured arm."""
        expected = {f'/{g}/moveit_track_enabled' for g in _target_arm_groups()}
        self.assertTrue(expected, 'quest.yaml declares no arm groups')

        # Must match the node's domain, set in generate_test_description.
        os.environ['ROS_DOMAIN_ID'] = '42'
        rclpy.init()
        try:
            probe = rclpy.create_node('startup_probe')
            deadline = time.monotonic() + 15.0
            found = set()
            while time.monotonic() < deadline and not expected <= found:
                rclpy.spin_once(probe, timeout_sec=0.2)
                found = {name for name, _ in probe.get_topic_names_and_types()}
            probe.destroy_node()
        finally:
            rclpy.shutdown()

        self.assertLessEqual(
            expected, found, f'missing enable topics: {sorted(expected - found)}')

    def test_constructor_completes(self, proc_output):
        """Last unconditional constructor line; SIGINT before it aborts with RCLError."""
        proc_output.assertWaitFor('Config:', timeout=15.0, stream='stderr')


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_exit_ok(self, proc_info, sobits_teleop_process):
        # keep_alive processes are SIGINT'd by the test framework at shutdown.
        launch_testing.asserts.assertExitCodes(
            proc_info, process=sobits_teleop_process)
