"""Servo launcher.

Starts the full MoveIt Servo stack for both arms plus the
servo_target_bridge, under the /<robot_name> namespace.

servo_node needs robot_description / robot_description_semantic /
robot_description_planning.* as its OWN node parameters at startup (Jazzy
servo 2.12.4 has no URDF/SRDF-path parameters — see SERVO_MIGRATION_PLAN.md).
In this system those parameters live only on <robot_name>/move_group, so this
launch file fetches them at launch time (mirrors the C++ fetch-from-move_group
pattern in moveit_arm_controller.cpp ~lines 185-230) via an OpaqueFunction that
spins a throwaway rclpy node against move_group's parameter services.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MOVE_GROUP_WAIT_TIMEOUT_S = 60.0

# rcl_interfaces/msg/ParameterType enum values.
PARAM_TYPE_NOT_SET = 0
PARAM_TYPE_BOOL = 1
PARAM_TYPE_INTEGER = 2
PARAM_TYPE_DOUBLE = 3
PARAM_TYPE_STRING = 4
PARAM_TYPE_BYTE_ARRAY = 5
PARAM_TYPE_BOOL_ARRAY = 6
PARAM_TYPE_INTEGER_ARRAY = 7
PARAM_TYPE_DOUBLE_ARRAY = 8
PARAM_TYPE_STRING_ARRAY = 9


def _parameter_value_to_python(value):
    """
    Convert an rcl_interfaces/ParameterValue to a plain Python value.

    Matches the field selected by its .type (same mapping rclpy.Parameter uses).
    """
    t = value.type
    if t == PARAM_TYPE_BOOL:
        return value.bool_value
    if t == PARAM_TYPE_INTEGER:
        return value.integer_value
    if t == PARAM_TYPE_DOUBLE:
        return value.double_value
    if t == PARAM_TYPE_STRING:
        return value.string_value
    # Empty arrays become None (caller skips them): launch_ros cannot infer an
    # element type for an empty list and raises when one reaches Node(parameters=).
    if t == PARAM_TYPE_BYTE_ARRAY:
        return list(value.byte_array_value) or None
    if t == PARAM_TYPE_BOOL_ARRAY:
        return list(value.bool_array_value) or None
    if t == PARAM_TYPE_INTEGER_ARRAY:
        return list(value.integer_array_value) or None
    if t == PARAM_TYPE_DOUBLE_ARRAY:
        return list(value.double_array_value) or None
    if t == PARAM_TYPE_STRING_ARRAY:
        return list(value.string_array_value) or None
    return None


def _fetch_move_group_params(context, *args, **kwargs):
    """
    Run at launch time, after 'robot_name' / 'use_sim_time' substitutions are resolvable.

    Fetches robot_description, robot_description_semantic, and the
    full robot_description_planning.* subtree from
    /<robot_name>/move_group, then returns the Servo + bridge launch actions
    parameterised with those values.
    """
    import rclpy
    import rclpy.executors
    from rclpy.node import Node as RclpyNode
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rcl_interfaces.msg import ParameterValue

    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ('true', '1', 'yes')

    move_group_node_name = f'/{robot_name}/move_group'

    # Dedicated rclpy context — launch_ros already owns the default one in-process.
    ctx = rclpy.Context()
    rclpy.init(context=ctx, args=None)
    fetch_node = None
    try:
        fetch_node = RclpyNode('servo_arms_launch_param_fetch', context=ctx)
        executor = rclpy.executors.SingleThreadedExecutor(context=ctx)
        executor.add_node(fetch_node)
        fetch_node.get_logger().info(
            f"Fetching robot_description(_semantic|_planning.*) from '{move_group_node_name}' ...")

        list_params_client = fetch_node.create_client(
            ListParameters, f'{move_group_node_name}/list_parameters')
        get_params_client = fetch_node.create_client(
            GetParameters, f'{move_group_node_name}/get_parameters')

        if not list_params_client.wait_for_service(timeout_sec=MOVE_GROUP_WAIT_TIMEOUT_S) or \
           not get_params_client.wait_for_service(timeout_sec=MOVE_GROUP_WAIT_TIMEOUT_S):
            raise RuntimeError(
                f"arm_backend_servo.launch.py: '{move_group_node_name}' parameter services "
                f'were not available within {MOVE_GROUP_WAIT_TIMEOUT_S:.0f} s — is '
                f"move_group running under namespace '/{robot_name}'? Aborting launch.")

        def call_sync(client, request, what):
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=MOVE_GROUP_WAIT_TIMEOUT_S)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    f"arm_backend_servo.launch.py: {what} call to '{move_group_node_name}' "
                    f'timed out or failed. Aborting launch.')
            return future.result()

        # move_group returns an EMPTY list for a whole GetParameters batch if any
        # name is unset (verified), so fetch one name at a time and skip empties.
        def get_params_chunked(names, what):
            values = []
            for name in names:
                req = GetParameters.Request()
                req.names = [name]
                resp = call_sync(get_params_client, req, f'{what} <{name}>')
                if resp.values:
                    values.append(resp.values[0])
                else:
                    # Unset/undeliverable single param: emit a NOT_SET placeholder
                    # so downstream zip(names, values) stays index-aligned.
                    placeholder = ParameterValue()
                    placeholder.type = PARAM_TYPE_NOT_SET
                    values.append(placeholder)
            return values

        # ── robot_description / robot_description_semantic ────────────────
        base_req = GetParameters.Request()
        base_req.names = ['robot_description', 'robot_description_semantic']
        base_resp = call_sync(get_params_client, base_req, 'get_parameters(robot_description*)')

        if len(base_resp.values) < 1 or not base_resp.values[0].string_value:
            raise RuntimeError(
                'arm_backend_servo.launch.py: robot_description is empty on move_group — '
                'aborting launch.')

        robot_description_value = base_resp.values[0].string_value
        robot_description_semantic_value = (
            base_resp.values[1].string_value if len(base_resp.values) > 1 else '')

        fetch_node.get_logger().info(
            f'robot_description fetched ({len(robot_description_value)} chars)')

        # ── robot_description_planning.* (recursive list, then get) ────────
        list_req = ListParameters.Request()
        list_req.prefixes = ['robot_description_planning']
        list_req.depth = 0  # recursive
        list_resp = call_sync(
            list_params_client, list_req, 'list_parameters(robot_description_planning)')

        planning_params = {}
        names = list(list_resp.result.names)
        if names:
            planning_values = get_params_chunked(
                names, 'get_parameters(robot_description_planning.*)')
            for name, value in zip(names, planning_values):
                if value.type == PARAM_TYPE_NOT_SET:
                    continue
                py_value = _parameter_value_to_python(value)
                if py_value is None:  # unset or empty array — cannot be passed on
                    continue
                planning_params[name] = py_value
            fetch_node.get_logger().info(
                f'robot_description_planning: fetched {len(planning_params)} / '
                f'{len(names)} sub-parameters')
        else:
            fetch_node.get_logger().warn(
                'robot_description_planning namespace empty on move_group — '
                'Servo joint-limit awareness may be degraded')

        # ── robot_description_kinematics.* ── without the IK params servo emits
        # zero commands; fetch from move_group the same way as _planning.*.
        kin_list_req = ListParameters.Request()
        kin_list_req.prefixes = ['robot_description_kinematics']
        kin_list_req.depth = 0  # recursive
        kin_list_resp = call_sync(
            list_params_client, kin_list_req, 'list_parameters(robot_description_kinematics)')

        kinematics_params = {}
        kin_names = list(kin_list_resp.result.names)
        if kin_names:
            kin_values = get_params_chunked(
                kin_names, 'get_parameters(robot_description_kinematics.*)')
            for name, value in zip(kin_names, kin_values):
                if value.type == PARAM_TYPE_NOT_SET:
                    continue
                py_value = _parameter_value_to_python(value)
                if py_value is None:  # unset or empty array — cannot be passed on
                    continue
                kinematics_params[name] = py_value
            fetch_node.get_logger().info(
                f'robot_description_kinematics: fetched {len(kinematics_params)} / '
                f'{len(kin_names)} sub-parameters')
        else:
            fetch_node.get_logger().warn(
                'robot_description_kinematics namespace empty on move_group — '
                'Servo POSE tracking will have no IK solver and emit no commands')
    finally:
        if fetch_node is not None:
            fetch_node.destroy_node()
        rclpy.shutdown(context=ctx)

    # ── Build the Servo + bridge nodes with the fetched parameters ────────
    pkg_share = get_package_share_directory('sobits_teleop')

    common_model_params = {
        'robot_description': robot_description_value,
        'robot_description_semantic': robot_description_semantic_value,
        **planning_params,
        **kinematics_params,
    }

    # Per-arm identity comes from quest.yaml / robot.yaml; this yaml carries only
    # tuning shared by all nodes.
    servo_yaml = f'{pkg_share}/config/{robot_name}/arm_backend_servo.yaml'

    import yaml as pyyaml
    with open(f'{pkg_share}/config/{robot_name}/quest.yaml') as f:
        quest_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    with open(f'{pkg_share}/config/{robot_name}/robot.yaml') as f:
        robot_params = pyyaml.safe_load(f)['/**']['ros__parameters']
    traj_topics = robot_params['robot_topic_name']['joint_trajectory_topic']

    quest_control = quest_params['quest_control']
    arm_entries = []  # (planning_group, quest controller block)
    for ctrl_name in quest_control.get('controllers', quest_control.get('controller', [])):
        block = quest_control.get(ctrl_name, {})
        if isinstance(block, dict) and 'arm' in block:
            arm_entries.append((block['arm'], block))
    if not arm_entries:
        raise RuntimeError(
            "arm_backend_servo.launch.py: no quest_control entries with an 'arm' key "
            f"found in quest.yaml for robot '{robot_name}' — nothing to servo.")

    servo_nodes = []
    bridge_arm_params = {'servo_bridge.arms': [a for a, _ in arm_entries]}
    for arm, block in arm_entries:
        servo_nodes.append(Node(
            package='moveit_servo',
            executable='servo_node',
            name=f'servo_{arm}',
            namespace=robot_name,
            output='screen',
            parameters=[
                servo_yaml,
                common_model_params,
                {
                    'moveit_servo.move_group_name': arm,
                    'moveit_servo.command_out_topic': traj_topics[arm],
                    'use_sim_time': use_sim_time,
                },
            ],
        ))
        # quest.yaml is also the authority for what the target TF means.
        if 'target_frame_name' in block:
            bridge_arm_params[f'servo_bridge.{arm}.target_frame_name'] = \
                block['target_frame_name']
        if 'end_effector_frame_name' in block:
            bridge_arm_params[f'servo_bridge.{arm}.end_effector_frame_name'] = \
                block['end_effector_frame_name']

    servo_target_bridge_node = Node(
        package='sobits_teleop',
        executable='servo_target_bridge',
        name='servo_target_bridge',
        namespace=robot_name,
        output='screen',
        parameters=[
            servo_yaml,
            bridge_arm_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    return servo_nodes + [servo_target_bridge_node]


def generate_launch_description():
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='sobit_home',
        description='Robot name used to select configuration files and namespace'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        OpaqueFunction(function=_fetch_move_group_params),
    ])
