---
name: sobits-teleop-config
description: Create or update sobits_teleop robot configuration YAML — adding a new robot, adding or rebinding a controller block (controller_joints/poses/velocity/tracking/cartesian), remapping buttons and axes, porting an existing robot to a new input device, or debugging why a teleop control does nothing. Use this whenever the user mentions sobits_teleop config, a device yaml (quest/ps4/keyboard), common.yaml, controllers_name, joint groups, pose blends or cycles, or says a button/stick/trigger isn't working on a SOBIT robot — even if they don't name the package explicitly.
---

# sobits_teleop robot configuration

## What this package's config actually is

`sobits_teleop` maps joystick input to robot motion. Every robot gets a
directory under `config/{robot_name}/` containing:

| File | Holds | Loaded |
|---|---|---|
| `common.yaml` | topics, loop rate, base speed limits | always |
| `{device}.yaml` | the controllers, one file per input device | one at a time |

`{device}` is `ps4`, `ps5`, `quest` or `keyboard`. **Exactly one device file
loads per run**, alongside `common.yaml`. That matters: two device files never
conflict, so the same button number can mean different things in `ps4.yaml`
and `keyboard.yaml`.

Launch is always:

```bash
ros2 launch sobits_teleop sobits_teleop.launch.py robot_name:=<robot> device:=<device>
```

## The five controllers

A device yaml lists which controllers to load, then configures them. Only
listed controllers load, so commenting one line disables it:

```yaml
/**:
  ros__parameters:
    controllers_name:
      - controller_joints
      - controller_poses
      # - controller_velocity     # disabled
```

Omitting `controllers_name` entirely enables every block defined below it.

| Controller | Drives | Input |
|---|---|---|
| `controller_joints` | one joint per entry, integrating | axis, held |
| `controller_velocity` | base `cmd_vel` | axes, direct |
| `controller_poses` | named configurations + blends + cycles | buttons |
| `controller_tracking` | joints following a TF frame's motion | frame delta |
| `controller_cartesian` | an arm EE following a controller pose | frame pose |

Read `references/controllers.md` for the full key list of each block — go
there whenever you are writing or editing one, because the key names are
specific and a wrong one is silently inert (see Verify below).

## The naming rules that are easy to get wrong

Every list of names ends in `_name`, and the entries sit as siblings below it:

```yaml
controller_joints:
  groups_name: [head]           # which groups exist
  head:
    joints_name: [head_pan_joint]   # which joints in this group
    head_pan_joint:                 # the entry itself
      axis: 0
      speed: 0.1
```

So: `controllers_name`, `groups_name`, `joints_name`, `poses_name`,
`blends_name`, `cycles_name`. If you write `joints:` or `groups:` the loader
does not see it and that group simply does nothing.

Group names are not free-form where topics are concerned: each group named in
`groups_name` must have a matching entry under
`common.yaml`'s `robot_topic_name.joint_trajectory_topic`, which is where its
trajectory topic comes from.

## Enables: how a control is armed

Most controllers take `enable_button` and/or `enable_axis`. Either one arms
the control; **with neither set the control is always live**. That is the
usual cause of "it moves when I don't touch the trigger".

A trigger reads as an axis, not a button. `enable_axis` fires above 0.5.

## Workflow

### Adding a new robot

1. **Copy the closest existing robot** rather than starting blank —
   `sobit_home` if it has arms, `sobit_edu` if it is simpler. Ask the user
   which is closest if it is not obvious.
2. **Write `common.yaml` first.** Get the joint-trajectory topics right by
   checking the real controllers: `ros2 control list_controllers`, or
   `ros2 topic list | grep joint_trajectory`. Everything downstream references
   these group names.
3. **Add one device file, starting with the smallest working controller set.**
   `controller_velocity` alone is a good first target: it proves topics and
   launch work before joint mapping is in play.
4. **Verify** (below), then add controllers one at a time.

### Updating an existing robot

Change the smallest thing, verify, repeat. The node reports each block it
loaded, so a change that produces no change in that output did not take
effect.

### Finding button and axis numbers

Never guess these — they vary by pad and driver. Read them live:

```bash
ros2 topic echo /<robot_name>/joy --once
```

Press the control, echo again, and diff the `buttons` and `axes` arrays. A
trigger appears in `axes`, not `buttons`.

## Verify — this is the important part

The node validates its own config at startup and says exactly what it
rejected. Always run it after an edit rather than reasoning about whether the
YAML is right:

```bash
ros2 run sobits_teleop sobits_teleop --ros-args \
  --params-file config/<robot>/common.yaml \
  --params-file config/<robot>/<device>.yaml
```

Read three things in the output:

1. **`Config:`** lines — which controllers loaded, and which are "not
   configured". A block you expected but do not see here is not being read.
2. **`Unknown parameter '<key>' — check for a typo`** — a key nothing reads.
   This catches misspellings and stale keys, and is the fastest way to find a
   wrong name. Zero of these is the goal.
3. **Specific rejections** — for example `Pose 'x' group 'y': 3 joints but 2
   positions`, or `Group 'z' has no robot_topic_name.joint_trajectory_topic
   entry`. Each names the offending group and reason.

A config that loads clean but does nothing usually means an enable is
unbound, or a group name has no topic entry.

To confirm motion, echo the trajectory topic while driving the input:

```bash
ros2 topic echo /<robot>/head_position_controller/joint_trajectory
```

**Rebuild before testing a launch or an installed config.** `colcon` copies
config and launch files at build time, so an edited file is not what runs
until you rebuild:

```bash
colcon build --packages-select sobits_teleop
```

Skipping this produces confusing results where the old config appears to
still be in effect.

## Editing YAML safely

Prefer targeted edits over rewriting a file with a YAML dumper. Round-tripping
through `yaml.safe_dump` strips every comment, reorders nothing usefully, and
can emit anchors (`*id001`) for repeated lists — which the ROS parameter
parser rejects outright with `Will not support aliasing`. If you do need to
dump, disable aliases:

```python
class NoAlias(yaml.SafeDumper):
    def ignore_aliases(self, d): return True
```

Check the result parses before handing it back: `python3 -c "import yaml;
yaml.safe_load(open('f.yaml'))"`.

## References

- `references/controllers.md` — every key of all five controller blocks, with
  what it does and its default. Read this when writing or editing any block.
- `references/recipes.md` — worked examples: new robot from scratch, adding a
  gripper blend, moving a joint to a different stick, one-arm robot.
- `scripts/validate_config.py` — static checks that do not need a robot:
  cross-file group/topic consistency, list-key naming, duplicate button
  bindings. Run it before launching to catch the cheap mistakes.
