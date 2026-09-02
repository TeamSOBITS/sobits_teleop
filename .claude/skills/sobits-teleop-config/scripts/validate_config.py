#!/usr/bin/env python3
"""Static checks on a sobits_teleop robot config, without needing a robot.

The node itself validates thoroughly at startup, so this is not a substitute
for running it — it catches the mistakes that are cheap to find on disk and
annoying to find at runtime: misspelled list keys, groups with no topic,
pose/position length mismatches, and duplicate input bindings.

Usage:
    validate_config.py config/my_robot                # every device file
    validate_config.py config/my_robot quest.yaml     # just one
"""
import sys
from collections import defaultdict
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required: pip install pyyaml")

LIST_KEYS = {
    'controllers_name', 'groups_name', 'joints_name',
    'poses_name', 'blends_name', 'cycles_name',
}
# A bare key that should have been the _name form. Silently inert otherwise.
MISSPELLINGS = {
    'controllers': 'controllers_name', 'groups': 'groups_name',
    'joints': 'joints_name', 'poses': 'poses_name',
    'pose_list': 'poses_name', 'blends': 'blends_name',
    'cycles': 'cycles_name', 'names': 'joints_name',
}
CONTROLLERS = {
    'controller_joints', 'controller_velocity', 'controller_poses',
    'controller_tracking', 'controller_cartesian',
}

problems, notes = [], []


def params(path):
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    return (doc.get('/**') or {}).get('ros__parameters', {}) or {}


def check_misspellings(node, where):
    """A bare list key the loader will not see."""
    if not isinstance(node, dict):
        return
    for key, val in node.items():
        # 'poses'/'blends'/'cycles' are sub-blocks of controller_poses, and
        # a cycle's 'poses:' is its own ordered list — all legitimate.
        legit_block = key in ('poses', 'blends', 'cycles') and (
            where.endswith('controller_poses') or '.cycles.' in where)
        if key in MISSPELLINGS and isinstance(val, list) and not legit_block:
            problems.append(
                f"{where}.{key}: should be '{MISSPELLINGS[key]}' — "
                f"the loader does not read '{key}', so this is inert")
        check_misspellings(val, f"{where}.{key}")


def walk_groups(block, block_name, topics):
    """Every group named in groups_name needs a topic entry."""
    if not isinstance(block, dict):
        return
    for group in block.get('groups_name', []) or []:
        if group not in topics:
            problems.append(
                f"{block_name}.{group}: no "
                f"robot_topic_name.joint_trajectory_topic.{group} in "
                f"common.yaml — the group will be skipped")
        if group not in block:
            problems.append(
                f"{block_name}.groups_name lists '{group}' but no "
                f"'{group}:' block is defined")


def check_pose_lengths(poses):
    for name in poses.get('poses_name', []) or []:
        pose = poses.get(name)
        if not isinstance(pose, dict):
            problems.append(f"poses_name lists '{name}' but it is not defined")
            continue
        for group in pose.get('groups_name', []) or []:
            g = pose.get(group)
            if not isinstance(g, dict):
                continue
            j, p = g.get('joints_name'), g.get('positions')
            if isinstance(j, list) and isinstance(p, list) and len(j) != len(p):
                problems.append(
                    f"pose '{name}' group '{group}': {len(j)} joints but "
                    f"{len(p)} positions — the group will be skipped")


def check_pose_refs(cp):
    """Blends and cycles must name poses that exist."""
    defined = set(cp.get('poses', {}).get('poses_name', []) or [])
    blends = cp.get('blends', {}) or {}
    for b in blends.get('blends_name', []) or []:
        e = blends.get(b, {})
        for side in ('from', 'to'):
            ref = e.get(side)
            if ref and ref not in defined:
                problems.append(
                    f"blend '{b}': {side}: '{ref}' is not in poses_name")
    cycles = cp.get('cycles', {}) or {}
    for c in cycles.get('cycles_name', []) or []:
        e = cycles.get(c, {})
        plist = e.get('poses', []) or []
        if len(plist) < 2:
            problems.append(f"cycle '{c}': needs two or more poses")
        for ref in plist:
            if ref not in defined:
                problems.append(f"cycle '{c}': '{ref}' is not in poses_name")


def collect_bindings(node, where, out):
    """Gather button/axis numbers so duplicates can be reported."""
    if not isinstance(node, dict):
        return
    for key, val in node.items():
        if isinstance(val, int) and val >= 0:
            if key in ('button', 'enable_button', 'fast_button',
                       'to_button', 'from_button', 'pose_button'):
                out[('button', val)].append(f"{where}.{key}")
            elif key in ('axis', 'enable_axis', 'fast_axis', 'x_axis',
                         'y_axis', 'fast_enable_axis'):
                out[('axis', val)].append(f"{where}.{key}")
        collect_bindings(val, f"{where}.{key}", out)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cfg = Path(sys.argv[1])
    common_path = cfg / 'common.yaml'
    if not common_path.exists():
        sys.exit(f"no common.yaml in {cfg}")

    common = params(common_path)
    topics = (common.get('robot_topic_name', {})
              .get('joint_trajectory_topic', {}) or {})
    # An empty topic map is fine for a velocity-only robot; walk_groups
    # reports per-group when something actually needs one.

    devices = ([cfg / a for a in sys.argv[2:]] if len(sys.argv) > 2
               else [p for p in sorted(cfg.glob('*.yaml'))
                     if p.name not in ('common.yaml',)
                     and not p.name.startswith('arm_')])

    for dev in devices:
        p = params(dev)
        tag = dev.name
        check_misspellings(p, tag)

        listed = p.get('controllers_name')
        if listed is not None:
            for c in listed:
                if c not in p:
                    notes.append(
                        f"{tag}: controllers_name lists '{c}' but no "
                        f"'{c}:' block is defined — nothing will load for it")
            for c in CONTROLLERS & set(p):
                if c not in listed:
                    notes.append(
                        f"{tag}: '{c}:' is defined but not in "
                        f"controllers_name — it is disabled")

        for name in ('controller_joints', 'controller_tracking',
                     'controller_cartesian'):
            walk_groups(p.get(name), f"{tag}.{name}", topics)

        cp = p.get('controller_poses')
        if isinstance(cp, dict):
            check_pose_lengths(cp.get('poses', {}) or {})
            check_pose_refs(cp)

        binds = defaultdict(list)
        collect_bindings(p, tag, binds)
        for (kind, num), uses in sorted(binds.items()):
            if len(uses) > 1:
                notes.append(
                    f"{tag}: {kind} {num} bound {len(uses)}x: "
                    + ", ".join(u.split('.', 1)[1] for u in uses))

    for m in problems:
        print(f"  PROBLEM  {m}")
    for m in notes:
        print(f"  note     {m}")
    if not problems and not notes:
        print("  no problems found")
    print(f"\n  {len(problems)} problem(s), {len(notes)} note(s)")
    print("  Static checks only — run the node to validate fully.")
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
