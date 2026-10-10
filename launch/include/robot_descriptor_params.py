"""Optional robot-descriptor overrides for the teleop launch files."""

import yaml


def try_load(robot_name, log=print):
    """Return the robot descriptor, or None when the robot has none."""
    try:
        from sobits_robot_descriptor import DescriptorError, load
    except ImportError:
        log('[sobits_teleop] sobits_robot_descriptor not installed; '
            f"using YAML config for '{robot_name}'")
        return None
    try:
        return load(robot_name)
    except DescriptorError as e:
        first = str(e).splitlines()[0]
        log(f"[sobits_teleop] no robot descriptor for '{robot_name}' ({first}); "
            'using YAML config')
        return None


def read_params(path):
    with open(path) as f:
        return (yaml.safe_load(f) or {})['/**']['ros__parameters']


def descriptor_params(desc):
    """Flat node parameters derived from the descriptor (relative topics)."""
    params = {
        'robot_topic_name.base_frame': desc.base_frame,
        'robot_topic_name.joint_states_topic': desc.joint_states_topic,
    }
    if desc.mobile_base is not None:
        params['robot_topic_name.cmd_vel_topic'] = desc.mobile_base.command_topic
    for g in desc.groups:
        if g.command_topic:
            params[f'robot_topic_name.joint_trajectory_topic.{g.name}'] = g.command_topic
    return params


def _flatten(prefix, value, out):
    if isinstance(value, dict):
        for k, v in value.items():
            _flatten(f'{prefix}.{k}' if prefix else str(k), v, out)
    else:
        out[prefix] = value


def shadowed_keys(common_params, overrides):
    """Descriptor-provided keys that the config YAML also sets."""
    flat = {}
    _flatten('', common_params, flat)
    return sorted(k for k in flat if k in overrides)


def _check_joints(desc, where, group, joints, errors):
    g = next((x for x in desc.groups if x.name == group), None)
    if g is None:
        known = ', '.join(x.name for x in desc.groups)
        errors.append(f'{where}: unknown group {group!r} (known groups: {known})')
        return
    known = set(g.joints) | set(g.uncommanded_joints)
    for j in joints or []:
        if j not in known:
            errors.append(
                f'{where}: joint {j!r} is not in group {group!r} '
                f'(known joints: {", ".join(sorted(known))})')


def validate_device(desc, device_params):
    """Raise RuntimeError when device joints/groups are not in the descriptor."""
    errors = []

    def block(where, cfg):
        for group in cfg.get('groups_name') or []:
            sub = cfg.get(group) or {}
            _check_joints(desc, f'{where}.{group}', group,
                          sub.get('joints_name'), errors)

    block('controller_joints', device_params.get('controller_joints') or {})
    block('controller_tracking', device_params.get('controller_tracking') or {})
    poses = (device_params.get('controller_poses') or {}).get('poses') or {}
    for pose in poses.get('poses_name') or []:
        block(f'controller_poses.poses.{pose}', poses.get(pose) or {})
    if errors:
        raise RuntimeError(
            f"Device config disagrees with robot descriptor '{desc.robot_id}':\n  "
            + '\n  '.join(errors))


def backend_topics(desc, common_params):
    """Return (trajectory topics by group, base frame); the descriptor wins."""
    topic_cfg = common_params.get('robot_topic_name') or {}
    topics = dict(topic_cfg.get('joint_trajectory_topic') or {})
    base_frame = topic_cfg.get('base_frame', 'base_footprint')
    if desc is not None:
        topics.update({g.name: g.command_topic for g in desc.groups if g.command_topic})
        base_frame = desc.base_frame
    return topics, base_frame


def check_ee_frames(desc, arm, end_effector_frame_name, target_frame_name):
    """Cross-check a quest.yaml arm block against the descriptor's ee entries."""
    entries = [e for e in desc.ee if e.control and e.control.group == arm]
    if not entries:
        return
    ok = any(e.ee_link == end_effector_frame_name
             and e.control.command_frame == target_frame_name for e in entries)
    if not ok:
        want = ', '.join(
            f'ee_link={e.ee_link} command_frame={e.control.command_frame}'
            for e in entries)
        raise RuntimeError(
            f'quest.yaml controller_cartesian.{arm} (end_effector_frame_name='
            f'{end_effector_frame_name}, target_frame_name={target_frame_name}) '
            f"disagrees with descriptor '{desc.robot_id}': {want}")
