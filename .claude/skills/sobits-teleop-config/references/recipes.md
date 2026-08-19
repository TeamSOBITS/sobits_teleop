# Recipes

Worked examples for the common config tasks.

- [New robot from scratch](#new-robot-from-scratch)
- [Porting a robot to another device](#porting-a-robot-to-another-device)
- [Moving a joint to a different stick](#moving-a-joint-to-a-different-stick)
- [Adding a gripper open/close](#adding-a-gripper-openclose)
- [One-arm robot](#one-arm-robot)
- [Disabling a controller temporarily](#disabling-a-controller-temporarily)
- [Debugging: the control does nothing](#debugging-the-control-does-nothing)

---

## New robot from scratch

Start from the closest existing robot, not a blank file.

```bash
cp -r config/sobit_edu config/my_robot     # simple robot, no arms
# or
cp -r config/sobit_home config/my_robot    # has arms, tracking, poses
```

**1. Fix `common.yaml` first.** Get the real controller topics:

```bash
ros2 control list_controllers
ros2 topic list | grep joint_trajectory
```

```yaml
robot_topic_name:
  base_frame: base_footprint
  joint_states_topic: joint_states
  cmd_vel_topic: cmd_vel
  joint_trajectory_topic:
    head: head_position_controller/joint_trajectory
    arm:  arm_position_controller/joint_trajectory
base_max_speed:
  linear:  0.8    # the robot's real limit
  angular: 1.5
```

**2. Cut the device file down to velocity only** to prove topics and launch:

```yaml
/**:
  ros__parameters:
    controllers_name: [controller_velocity]
    controller_velocity:
      enable_button: 5
      axis_sign: 1
      linear:  { x_axis: 1, y_axis: 0, scale: 0.2, fast_scale: 0.6 }
      angular: { axis: 3, scale: 0.2, fast_scale: 0.6 }
```

Build, run, check `Config: controller_velocity` and zero unknown parameters,
then drive the base.

**3. Add controllers one at a time**, verifying after each. Delete every group
inherited from the source robot that this robot does not have — a leftover
group whose joints do not exist produces `No URDF limits for 'x'` at runtime.

---

## Porting a robot to another device

Only the device file changes; `common.yaml` is shared.

```bash
cp config/my_robot/ps4.yaml config/my_robot/keyboard.yaml
```

Then rebind every `button`, `axis`, `enable_axis` and `fast_button` to the new
device's numbering — see "Finding button and axis numbers" in SKILL.md. The
group and joint names carry over unchanged.

Keyboard has no analog sticks, so for blends prefer `to_button`/`from_button`
over `axis`, and for `controller_velocity` expect the keyboard driver's axes
rather than a real stick.

---

## Moving a joint to a different stick

Change `axis` (and `axis_sign` if the direction inverts):

```yaml
head_pan_joint:
  axis: 3          # was 0
  axis_sign: -1    # right stick reads inverted on this pad
```

If the new stick already drives another joint, add `dominant_over` to both so
a diagonal push does not move both:

```yaml
head_pan_joint:  { axis: 3, dominant_over: 4 }
head_tilt_joint: { axis: 4, dominant_over: 3 }
```

---

## Adding a gripper open/close

Two poses, then either a cycle (button steps between them) or a blend
(analog sweep). Both, if you want each.

```yaml
controller_poses:
  poses:
    poses_name: [hand_open, hand_close]
    hand_open:
      button: -1                    # -1: only reachable via cycle/blend
      groups_name: [hand_right]
      hand_right:
        joints_name: [finger_a, finger_b]
        positions:   [1.571, 0.0]
    hand_close:
      button: -1
      groups_name: [hand_right]
      hand_right:
        joints_name: [finger_a, finger_b]
        positions:   [-0.5, -1.571]

  cycles:
    cycles_name: [grip_toggle]
    grip_toggle:
      button: 6
      poses: [hand_open, hand_close]

  blends:
    blends_name: [grip]
    grip:
      from: hand_open
      to:   hand_close
      enable_axis: 6      # hold the trigger
      axis: 4             # stick sweeps the grip
      axis_sign: -1
      speed: 0.6
```

Both endpoints must define the same group and share joints, or the blend is
skipped with `poses share no joints`.

A joint owned by another control (a knuckle jogged from `controller_joints`)
should be left out of the poses so the blend does not fight it.

---

## One-arm robot

No launcher change is needed — both arm backends iterate
`controller_cartesian.groups_name`.

```yaml
controller_cartesian:
  groups_name: [arm_right]      # just the one
  arm_right:
    enable_axis: 7
    controller_frame_name: right_controller_odom
    controller_echo_frame_name: right_controller_link
    end_effector_frame_name: hand_right_end_effector_link
    target_frame_name: right_target_link
```

Then remove the other arm's dependents or they warn as unread keys: its
`controller_joints` entry, its poses/blend/cycle, and its
`joint_trajectory_topic` line in `common.yaml`.

With no arms at all, simply do not pass `use_moveit:=true`.

---

## Disabling a controller temporarily

Comment it out of `controllers_name`. The block can stay in the file; its keys
are exempt from the unknown-parameter check while disabled.

```yaml
controllers_name:
  - controller_joints
  # - controller_velocity      # base disabled for this session
```

---

## Debugging: the control does nothing

Work down this list — each step rules out one cause.

1. **Did the block load?** Check the `Config:` line. If it is under "not
   configured", either it is missing from `controllers_name` or its gating key
   is absent.

2. **Any unknown parameters?** `Unknown parameter 'x' — check for a typo` names
   the exact key nothing read. A misspelled list key (`joints:` for
   `joints_name:`) silently drops the whole group.

3. **Is the group topic mapped?** `Group 'x' has no
   robot_topic_name.joint_trajectory_topic entry` means `common.yaml` is
   missing that group.

4. **Is the enable bound?** With `button` and `enable_axis` both unset the
   control is always live; with the wrong number it never arms. Confirm the
   real number from `/joy`.

5. **Is the config the one running?** `colcon build --packages-select
   sobits_teleop` — the launcher reads the installed copy, not your edit.

6. **Is anything published?** `ros2 topic echo /<robot>/<group>_position_controller/joint_trajectory`
   while driving the input. Messages here but no motion means the controller
   or hardware side, not this config.
