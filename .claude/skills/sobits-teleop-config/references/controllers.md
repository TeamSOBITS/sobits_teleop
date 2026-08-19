# Controller block reference

Every key of the five controller blocks, plus `common.yaml`.

- [common.yaml](#commonyaml)
- [controller_joints](#controller_joints)
- [controller_velocity](#controller_velocity)
- [controller_poses](#controller_poses)
- [controller_tracking](#controller_tracking)
- [controller_cartesian](#controller_cartesian)

---

## common.yaml

Per-robot, always loaded, shared by every device.

```yaml
/**:
  ros__parameters:
    teleop_rate_hz: 100.0           # control loop rate
    robot_topic_name:
      base_frame: base_footprint    # frame all tracking resolves in
      joint_states_topic: joint_states
      cmd_vel_topic: cmd_vel
      joint_trajectory_topic:       # one entry per joint group
        head:  head_position_controller/joint_trajectory
        body:  body_position_controller/joint_trajectory
    base_max_speed:                 # robot capability, not preference
      linear:  1.0                  # m/s
      angular: 1.0                  # rad/s
```

`joint_trajectory_topic` is the authority for group names. Any group named in
any controller must appear here, or it is skipped with
`Group 'x' has no robot_topic_name.joint_trajectory_topic entry`.

`base_frame` must be fixed relative to the arm's kinematic root.

---

## controller_joints

Jogs one joint per entry. Holding the axis integrates position, so releasing
leaves the joint where it is.

```yaml
controller_joints:
  groups_name: [head]
  head:
    joints_name: [head_pan_joint]
    head_pan_joint:
      button: 2           # enable button; -1 or omitted = unused
      enable_axis: -1     # enable axis (a trigger); -1 = unused
      fast_button: 6      # hold for fast_speed
      fast_axis: -1       # axis alternative to fast_button
      axis: 0             # the axis that drives the joint
      axis_sign: 1        # -1 to invert
      dominant_over: -1   # only drive while |axis| exceeds this axis
      speed: 0.1          # rad per 50 ms tick at full deflection
      fast_speed: 0.5
```

With both `button` and `enable_axis` unset the joint is **always live**.

`dominant_over` exists for two joints on one physical stick: the vertical
joint only moves when the vertical push exceeds the horizontal, so a diagonal
does not drive both.

---

## controller_velocity

Base twist. `scale` is a **fraction of `base_max_speed`**, not a velocity.

```yaml
controller_velocity:
  enable_button: 5        # arms both linear and angular
  fast_enable_button: 7
  enable_axis: -1
  fast_enable_axis: -1
  axis_sign: 1
  linear:
    x_axis: 1
    y_axis: 0
    scale: 0.2            # 0.2 x base_max_speed.linear
    fast_scale: 0.6
  angular:
    axis: 3
    scale: 0.2
    fast_scale: 0.6
```

Output is direct, not integrating: releasing the stick sends zero.

---

## controller_poses

Named configurations, plus two ways to move between them.

```yaml
controller_poses:
  pose_action: move_to_pose   # MoveToPose server, for server-resolved poses
  time_from_start: 3.0        # default seconds to reach a pose
  trigger: 8                  # optional modifier button for all poses

  poses:
    poses_name: [initial_pose]
    initial_pose:
      button: 2
      time_from_start: 3.0    # per-pose override
      groups_name: [head]
      head:
        button: 4             # optional: fires this group alone
        joints_name: [head_pan_joint, head_tilt_joint]
        positions: [0.0, 0.0]

  blends:                     # sweep between two poses
    blends_name: [grip]
    grip:
      from: hand_open
      to:   hand_close
      exclude_groups: []      # groups of the two poses to leave alone
      enable_axis: 6
      enable_button: -1
      axis: 4                 # sign picks the pose, size the speed
      axis_sign: -1
      to_button: -1           # button alternative for devices with no axis
      from_button: -1
      speed: 0.6

  cycles:                     # step through poses, one per press
    cycles_name: [grip_toggle]
    grip_toggle:
      button: 6
      exclude_groups: []
      poses: [hand_open, hand_close]
```

A pose omitting `groups_name` falls back to the `MoveToPose` action, which
resolves it server-side — useful when joint values live in another package.

`joints_name` and `positions` must be the same length or the group is skipped.

Blends drive every group the two poses share, minus `exclude_groups`, and
within each group every joint they share. Releasing holds position, which is
what lets a gripper keep hold of an object.

---

## controller_tracking

Joints follow a TF frame's motion since the latch. Group names are free.

```yaml
controller_tracking:
  groups_name: [head, body]
  head:
    enable_axis: 2                # hold to latch
    target_frame_name: hmd_odom   # frame whose motion is followed
    motion_scale: 1.0
    joints_name: [head_pan_joint, head_tilt_joint]
    head_pan_joint:
      type: rotation              # rotation | prismatic
      axis: yaw                   # rotation: roll|pitch|yaw
      sign: 1
    head_tilt_joint:
      type: rotation
      axis: pitch
      sign: -1
```

`type: prismatic` takes `axis: x|y|z` instead. One group may mix both.

The latch reads the trigger from `/joy`, not from TF, so releasing always
stops tracking even when the frame is stale. On TF recovery it re-anchors
rather than replaying the gap.

Invalid `type` or an `axis` not belonging to that type is reported by name and
the joint skipped.

---

## controller_cartesian

An arm end effector follows a controller's pose, handed to the arm backend
rather than published as joint positions.

```yaml
controller_cartesian:
  groups_name: [arm_right]
  arm_right:
    enable_axis: 7                                    # grip; hold to track
    controller_frame_name: right_controller_odom      # TF the arm follows
    controller_echo_frame_name: right_controller_link # RViz echo of it
    end_effector_frame_name: hand_right_end_effector_link
    target_frame_name: right_target_link
    motion_scale: 2.0
    proximity_threshold: 0.0        # m; 0 = latch immediately
    proximity_angle_threshold: 0.0  # rad
```

A group here is recognised by having `end_effector_frame_name`; without it the
group is reported and skipped.

Both arm-backend launchers read `controller_cartesian.groups_name` to decide
which arms to drive, so the list is count-agnostic — a one-arm robot just
lists one group and needs no launcher change.

The launcher forwards `target_frame_name` and `end_effector_frame_name` from
the device yaml into the servo bridge, so those two live here and not in
`arm_backend_servo.yaml`.
