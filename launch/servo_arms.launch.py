"""
Servo launcher — starts the full MoveIt Servo stack for both arms plus the
servo_target_bridge, under the /<robot_name> namespace.

servo_node needs robot_description / robot_description_semantic /
robot_description_planning.* as its OWN node parameters at startup (Jazzy
servo 2.12.4 has no URDF/SRDF-path parameters — see SERVO_MIGRATION_PLAN.md).
In this system those parameters live only on <robot_name>/move_group, so this
launch file fetches them at launch time (mirrors the C++ fetch-from-move_group
pattern in moveit_arm_controller.cpp ~lines 185-230) via an OpaqueFunction that
spins a throwaway rclpy node against move_group's parameter services.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    """Converts an rcl_interfaces/ParameterValue to a plain Python value,
    matching the field selected by its .type (same mapping rclpy.Parameter uses)."""
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
    Runs at launch time (after 'robot_name' / 'use_sim_time' substitutions are
    resolvable). Fetches robot_description, robot_description_semantic, and the
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

    # Use a DEDICATED rclpy context (never the default one): this code runs
    # inside the `ros2 launch` process, and launch_ros uses rclpy's default
    # context in-process — rclpy.init() would raise if already initialized and
    # rclpy.shutdown() would tear down state launch still needs.
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
                f"servo_arms.launch.py: '{move_group_node_name}' parameter services "
                f"were not available within {MOVE_GROUP_WAIT_TIMEOUT_S:.0f} s — is "
                f"move_group running under namespace '/{robot_name}'? Aborting launch.")

        def call_sync(client, request, what):
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=MOVE_GROUP_WAIT_TIMEOUT_S)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    f"servo_arms.launch.py: {what} call to '{move_group_node_name}' "
                    f"timed out or failed. Aborting launch.")
            return future.result()

        # move_group declares some planning params (e.g. *.has_jerk_limits,
        # *.max_position) without ever setting them. GetParameters returns an
        # entry of type NOT_SET for such a name — but if a BATCH request contains
        # even one of them, this move_group's service returns an EMPTY value list
        # for the WHOLE batch (verified: names[10:20] -> 0 values, because
        # arm_left_elbow_joint.has_jerk_limits is unset). Batching is therefore
        # unusable here; fetch one name at a time. Unset names come back empty and
        # are skipped (aligned to `names` by index; missing -> None -> caller skips).
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
                "servo_arms.launch.py: robot_description is empty on move_group — "
                "aborting launch.")

        robot_description_value = base_resp.values[0].string_value
        robot_description_semantic_value = (
            base_resp.values[1].string_value if len(base_resp.values) > 1 else '')

        fetch_node.get_logger().info(
            f"robot_description fetched ({len(robot_description_value)} chars)")

        # ── robot_description_planning.* (recursive list, then get) ────────
        list_req = ListParameters.Request()
        list_req.prefixes = ['robot_description_planning']
        list_req.depth = 0  # recursive
        list_resp = call_sync(list_params_client, list_req, 'list_parameters(robot_description_planning)')

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
                f"robot_description_planning: fetched {len(planning_params)} / {len(names)} sub-parameters")
        else:
            fetch_node.get_logger().warn(
                "robot_description_planning namespace empty on move_group — "
                "Servo joint-limit awareness may be degraded")

        # ── robot_description_kinematics.* (recursive list, then get) ──────
        # Servo POSE tracking solves Cartesian->joint IK every tick; without
        # these params servo_node logs "No IK solver for planning group" and
        # emits zero commands. move_group already holds them (loaded from the
        # moveit_config kinematics.yaml), so fetch the same way as _planning.*.
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
                f"robot_description_kinematics: fetched {len(kinematics_params)} / {len(kin_names)} sub-parameters")
        else:
            fetch_node.get_logger().warn(
                "robot_description_kinematics namespace empty on move_group — "
                "Servo POSE tracking will have no IK solver and emit no commands")
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

    # One consolidated file; each node reads only its own /**/<node_name> section.
    servo_yaml = f'{pkg_share}/config/{robot_name}/servo.yaml'

    servo_arm_right_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_arm_right',
        namespace=robot_name,
        output='screen',
        parameters=[
            servo_yaml,
            common_model_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    servo_arm_left_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_arm_left',
        namespace=robot_name,
        output='screen',
        parameters=[
            servo_yaml,
            common_model_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    servo_target_bridge_node = Node(
        package='sobits_teleop',
        executable='servo_target_bridge',
        name='servo_target_bridge',
        namespace=robot_name,
        output='screen',
        parameters=[
            servo_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [servo_arm_right_node, servo_arm_left_node, servo_target_bridge_node]


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
