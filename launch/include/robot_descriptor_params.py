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


def backend_topics(desc, common_params):
    """Return (trajectory topics by group, base frame); the descriptor wins."""
    topic_cfg = common_params.get('robot_topic_name') or {}
    topics = dict(topic_cfg.get('joint_trajectory_topic') or {})
    base_frame = topic_cfg.get('base_frame', 'base_footprint')
    if desc is not None:
        topics.update({g.name: g.command_topic for g in desc.groups if g.command_topic})
        base_frame = desc.base_frame
    return topics, base_frame
