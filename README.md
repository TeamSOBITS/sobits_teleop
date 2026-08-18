<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS TELEOP

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#introduction">Introduction</a></li>
    <li><a href="#setup">Setup</a></li>
    <li>
      <a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#create-config-files">Create Config Files</a></li>
        <li><a href="#launch-arguments">Launch Arguments</a></li>
        <li><a href="#run-teleop-node">Run Teleop Node</a></li>
        <li><a href="#arm-tracking-backends-meta-quest">Arm-Tracking Backends (Meta Quest)</a></li>
        <li><a href="#porting-to-a-new-robot">Porting to a New Robot</a></li>
        <li><a href="#arm-scale-calibration-meta-quest">Arm Scale Calibration (Meta Quest)</a></li>
        <li><a href="#simulation-mode">Simulation Mode</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

A package for teleoperating robots using a joystick (PS4, PS5), Meta Quest, or a keyboard.\
For Meta Quest setup instructions, please refer to [this repository](https://github.com/TeamSOBITS/meta_quest_teleoperation).

The package is **robot-agnostic**: everything robot-specific lives in one
config directory (`config/{robot_name}/`), selected at launch with
`robot_name:=<name>`. Ready-made configs are provided for the SOBITS robots
(`sobit_home`, `sobit_pro`, `sobit_edu`, `sobit_mini`, `sobit_light`); porting
to a new robot means writing that directory — no code or launch changes
(see [Porting to a New Robot](#porting-to-a-new-robot)).

Supported input devices: `ps4`, `ps5`, `quest`, `keyboard`

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- SETUP -->
## Setup

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12+ |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of your colcon workspace.
    ```sh
    cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    git clone -b jazzy-devel https://github.com/TeamSOBITS/sobits_teleop
    ```

3. Navigate into the repository.
    ```sh
    cd sobits_teleop/
    ```

4. Install the dependent packages.
    ```sh
    bash install.sh
    ```

5. Build the workspace.
    ```sh
    cd ~/colcon_ws/
    colcon build --symlink-install
    source ~/colcon_ws/install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE -->
## Launch and Usage

Basic workflow:

1. Create or verify config files for your robot and input device.
2. Connect the input device to your PC.
3. Launch the teleop node with the appropriate arguments.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Create Config Files

Each robot has its own directory under `config/{robot_name}/` — this directory
is the only robot-specific part of the package:

| File | Purpose |
|---|---|
| `robot.yaml` | ROS topic names for joint controllers and cmd_vel |
| `{device}.yaml` | Button/axis mappings for the selected input device |
| `arm_backend_plan.yaml` | Plan-and-replace arm-tracking backend tuning (Quest only) |
| `arm_backend_servo.yaml` | MoveIt Servo arm-tracking backend tuning (Quest only) |
| `arm_scale_calibrator.yaml` | Arm reach calibration parameters (Quest only) |

The **arm identity** — which arms exist, their planning groups, target/EE
frames and controller topics — is declared once, in `{device}.yaml`
(the per-arm `arm:` blocks) and `robot.yaml` (the trajectory-topic map).
The launchers inject it into whichever arm-tracking backend runs, so the
two `arm_backend_*.yaml` files contain tuning only and are largely
robot-independent.

<details>
<summary>robot.yaml (example)</summary>

```yaml
/**:
  ros__parameters:
    robot_topic_name:
      joint_states_topic: joint_states
      joint_trajectory_topic:
        head:      head_position_controller/joint_trajectory
        body:      body_position_controller/joint_trajectory
        arm_left:  arm_left_position_controller/joint_trajectory
        arm_right: arm_right_position_controller/joint_trajectory
      cmd_vel_topic: cmd_vel
```

</details>

<details>
<summary>{device}.yaml (example)</summary>

```yaml
/**:
  ros__parameters:

    control_joints:       # Joint trajectory groups to control
      groups:
        - head
        - arm_left
      head:
        names:
          - head_tilt_joint
          - head_pan_joint
        head_tilt_joint:
          button:      2    # Enable button
          fast_button: 6    # Hold to move faster
          axis:        1    # Joystick axis
          axis_sign:   1    # Invert direction if needed
          speed:       0.1  # Speed when button pressed
          fast_speed:  0.5  # Speed when fast_button pressed

    control_poses:        # Move to predefined poses
      trigger: 8            # Optional modifier button; omit for none
      time_from_start: 3.0  # Default seconds to reach a pose
      pose_list:
        - initial_pose
        - pre_manipulation_pose
      initial_pose:
        button: 2
        # Listing `groups` defines the pose HERE and publishes joint
        # trajectories; omit it to resolve pose_name via the MoveToPose action.
        groups:
          - head
          - arm_left
        head:
          joints:    [ head_pan_joint, head_tilt_joint ]
          positions: [ 0.0,            0.0             ]
        arm_left:
          joints:    [ arm_left_elbow_joint ]
          positions: [ 2.5 ]
      pre_manipulation_pose:
        button: 3           # No `groups` -> MoveToPose action backend

    control_velocity:     # Mobile base control
      button:             5
      fast_button:        7
      linear_x_axis:      1
      linear_y_axis:      0
      angular_axis:       3
      axis_sign:          1
      linear_scale:       0.1
      angular_scale:      0.3
      fast_linear_scale:  0.2
      fast_angular_scale: 0.6
```

</details>

#### Pose backends

`control_poses` entries resolve one of two ways, chosen per pose:

| Pose defines `groups` | Backend | Behaviour |
|---|---|---|
| Yes | Joint trajectory topics | Joints/positions come straight from the YAML and are published to each group's controller. No action server needed. |
| No | `MoveToPose` action | Only `pose_name` is sent; the pose is resolved server-side. |

Each name under `groups` must be a joint group declared in `robot.yaml`, which
supplies the trajectory topic (override per group with `joint_trajectory_topic`).
`joints` and `positions` must be the same length — a mismatched group is skipped
with an error at startup instead of moving the robot. Joints not listed are not
commanded, so they hold whatever the controller last had. For a single-group
pose you may put `joints`/`positions` directly under the pose and drop `groups`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Launch Arguments

All options are passed as CLI arguments — no need to edit the launch file.

| Argument | Default | Description |
|---|---|---|
| `robot_name` | `sobit_home` | Robot name — selects the config directory |
| `device` | `ps4` | Input device: `ps4`, `ps5`, `quest`, `keyboard` |
| `joystick_device` | `/dev/input/js0` | Joystick device path (PS4/PS5 only) |
| `ros_ip` | `0.0.0.0` | PC IP address for Quest TCP connection |
| `use_ds4drv` | `True` | Launch `ds4drv` alongside (PS4 only) |
| `use_moveit` | `false` | Launch an arm-tracking backend (Quest only) |
| `use_servo` | `false` | Pick the MoveIt Servo backend instead of plan-and-replace (requires `use_moveit:=true`) |
| `use_sim_time` | `false` | Use simulation clock (Gazebo) |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Run Teleop Node

#### PS4 / PS5

1. Pair the controller with your PC via Bluetooth.
2. Verify the connection with `jstest-gtk`.
3. Launch:
    ```sh
    ros2 launch sobits_teleop sobits_teleop.launch.py \
      robot_name:=sobit_home \
      device:=ps4
    ```

> [!Note]
> When using `ds4drv` inside a Docker container, mount `/dev/input/` and `/run/udev` into the container.

#### Meta Quest

1. Ensure the Quest and your PC are on the same Wi-Fi network.
2. Find your PC's IP address:
    ```sh
    hostname -I
    ```
3. Launch the teleop node, passing your PC's IP:
    ```sh
    ros2 launch sobits_teleop sobits_teleop.launch.py \
      robot_name:=sobit_home \
      device:=quest \
      ros_ip:=<YOUR_PC_IP>
    ```
4. Start the Unity project on the Quest. Enter the same IP shown in the launcher via the **menu button** on the left controller.

For Quest setup, refer to [meta_quest_teleoperation](https://github.com/TeamSOBITS/meta_quest_teleoperation).

#### Keyboard

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=keyboard
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Arm-Tracking Backends (Meta Quest)

Two interchangeable backends follow the hand-controller TF pose in real time.
Both consume the same interface — the `{side}_target_link` TF plus the
`/{robot_name}/{arm_name}/moveit_track_enabled` (`std_msgs/Bool`) topic, which
the grip button toggles automatically — so they can be swapped with one launch
argument.

| Backend | Selected by | How it tracks |
|---|---|---|
| Plan-and-replace (`moveit_arm_controller`) | `use_moveit:=true` | Streams short Cartesian plans via MoveIt2; falls back to OMPL near singularities |
| MoveIt Servo (`servo_node` + `servo_target_bridge`) | `use_moveit:=true use_servo:=true` | One `moveit_servo` instance per arm does differential IK at 50 Hz |

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=<robot_name> \
  device:=quest \
  ros_ip:=<YOUR_PC_IP> \
  use_moveit:=true \
  use_servo:=true        # omit for the plan-and-replace backend
```

Both launchers live in `launch/include/` and inject the arm identity from
`quest.yaml` / `robot.yaml`, so the backend configs are tuning-only.

#### Plan-and-replace tuning — `config/{robot_name}/arm_backend_plan.yaml`

```yaml
arm_teleop:
  update_rate_hz:          50.0   # tracking loop frequency
  max_cartesian_step_m:    0.10   # max step per cycle
  eef_step_m:              0.02   # Cartesian interpolation resolution
  min_cartesian_fraction:   0.2   # fall back to OMPL below this fraction
  replan_threshold_m:      0.03   # replan when target drifts this far
  preempt_threshold_m:     0.30   # cancel in-flight trajectory beyond this
  arrival_threshold_m:     0.03   # skip planning when EE is this close
  velocity_scaling:        0.90
  acceleration_scaling:    0.80
  publish_mode: topic             # stream trajectories (best for teleop)
```

#### Servo tuning — `config/{robot_name}/arm_backend_servo.yaml`

One file for all servo-stack nodes; the shared `/**:` section holds the tuning.
Robot-dependent points worth checking when porting:

```yaml
moveit_servo:
  scale: {linear: 1.5, rotational: 3.0}  # EE speed caps [m/s, rad/s]
  publish_joint_velocities: false  # keep false if the arm JTC rejects
                                   # trajectories ending with nonzero velocity
  lower_singularity_threshold: 50.0
  hard_stop_singularity_threshold: 200.0  # do NOT disable (joint windup)
  joint_limit_margins: [0.02]

servo_bridge:
  pose_rate_hz: 100.0
  max_reach: 1.10   # reach-clamp sphere radius around each arm's shoulder [m]
                    # — set to ~90-95% of YOUR robot's shoulder→EE chain length
```

The bridge clamps targets to the `max_reach` sphere so an out-of-reach hand
cannot drag the arm into its full-extension singularity.

##### Singularity halt recovery

`hard_stop_singularity_threshold` latches a servo e-stop
(`HALT_FOR_SINGULARITY`) and servo then ignores pose commands, so retargeting
alone cannot recover — without help the operator must release the grip and
re-latch. The bridge watches each servo's `~/status` and recovers automatically:

```yaml
servo_bridge:
  reset_on_halt: true           # run the recovery below on a latched halt
  reset_cooldown_s: 2.0         # min gap between attempts
  joint_escape_time_s: 1.0      # escape trajectory duration; 0 disables
  joint_escape_lookback_s: 1.0  # escape to where the arm was this far back
  escape_step: 0.005            # Cartesian nudge per tick [m]; 0 disables
  escape_timeout_s: 2.0         # then give up and ask for a re-latch
```

Recovery pauses servo, drives the arm out **in jointspace** — no Jacobian is
involved, so the singularity cannot block it — then resumes servo once the
escape trajectory has had time to run. The escape target is the arm's joint
configuration `joint_escape_lookback_s` ago, not the newest one: the last
command before a halt sits right next to the singularity and would land back
in it. `escape_step` additionally walks the Cartesian command back toward the
last healthy EE pose, which only helps before the arm parks *on* the singular
pose. If a halt outlives `escape_timeout_s` the override is released and the
operator is told to re-latch, so a persistent halt cannot pin the arm forever.

##### Per-arm naming

Per-arm frames and topics are derived from the arm name using templates, so an
arm that follows the convention needs no per-arm config. `{arm}` expands to the
full name (`arm_right`) and `{side}` to the name without the `arm_` prefix
(`right`):

```yaml
servo_bridge:
  naming:
    target_frame_name:       "{side}_target_link"
    end_effector_frame_name: "hand_{side}_end_effector_link"
    reach_origin_frame:      "{arm}_shoulder_tilt_link"
    servo_node:              "servo_{arm}"
    enable_topic:            "{arm}/moveit_track_enabled"
    joint_traj_topic:        "{arm}_position_controller/joint_trajectory"
    status_topic:            "{servo_node}/status"
```

If your robot names its links differently, edit the templates — no rebuild is
needed. `status_topic` expands `{servo_node}` from the resolved node name, so
it follows a `servo_node` override. A single arm can still be handled by
setting the key directly under `servo_bridge.{arm_name}.*`, which wins over
the template.

The servo backend requires `ros-$ROS_DISTRO-moveit-servo` (installed by
`install.sh`) and a running `move_group` under `/{robot_name}` — the launcher
fetches the robot model from it at startup.

#### Gripper controls (Quest, both backends)

| Input | Action |
|---|---|
| Grip button (`enable_axis`) | Hold to track the arm |
| Pose button (`pose_button`) | Toggle `pose_open` / `pose_close` |
| Trigger + stick left/right | Adaptive open/close curl |
| Trigger + stick up/down | Rotate the grip-type joint (`single_joint.name`) |

#### Head controls (Quest)

Hold `quest_control.head.head_mode` to latch head tracking; the head then
follows the HMD pose. The latch is driven by the trigger state from `/joy`, not
by the HMD transform, so releasing always stops tracking even while the Quest
TF is stale. When the TF recovers after a dropout the latch re-anchors on the
current pose rather than replaying the whole gap as one jump.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Porting to a New Robot

1. Create `config/{robot_name}/` with `robot.yaml` (controller topics) and one
   `{device}.yaml` per input device you use.
2. For Quest arm teleop, add `arm_<side>`/`hand_<side>` groups to `quest.yaml`
   (`target_frame_name:`, `end_effector_frame_name:`, gripper mapping) — this is
   the single source of arm identity for both backends.
3. Copy `arm_backend_plan.yaml` / `arm_backend_servo.yaml` from an existing
   robot and adjust the tuning (`max_reach` to your arm length; speed caps;
   thresholds).
4. Launch with `robot_name:={robot_name}` — nothing else changes.

Assumptions: arms driven by `joint_trajectory_controller` topic interfaces
(see `robot.yaml`), a MoveIt config with one planning group per arm whose last
SRDF link is the end-effector, and `move_group` running under
`/{robot_name}`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Arm Scale Calibration (Meta Quest)

The `scale` parameter in `quest.yaml` maps the human arm reach to the robot arm reach. Run `arm_scale_calibrator` once per operator to compute the correct value automatically.

#### When to run

Run once per operator, or whenever the robot URDF arm reach changes. The result is pasted into `config/{robot_name}/quest.yaml` under `arm_right.scale` and `arm_left.scale`.

#### Configuration

`config/{robot_name}/arm_scale_calibrator.yaml`:

```yaml
robot_arm_reach_m: 1.2926      # full shoulder→EE chain length from URDF

right_frame:  "right_controller_odom"
left_frame:   "left_controller_odom"
parent_frame: "base_footprint"

grip_axis: 7                   # right grip axis index in Joy message
```

For a **single-arm robot**, set the unused frame to `""`:

```yaml
right_frame: "right_controller_odom"
left_frame:  ""   # disabled — only right arm measured
```

#### How to run

```sh
ros2 run sobits_teleop arm_scale_calibrator --ros-args \
  --params-file install/sobits_teleop/share/sobits_teleop/config/{robot_name}/arm_scale_calibrator.yaml \
  -p joy_topic:=/{robot_name}/joy
```

> [!Note]
> Replace `{robot_name}` with your robot's name (e.g. `sobit_home`).

#### Procedure

**Step 1 — Set start position**
1. Hold both controllers in a natural standing position.
2. Extend **both arms straight forward** toward the robot.
3. Press and release the **right grip button** to capture the start positions.

**Step 2 — Sweep to T-pose**
1. Slowly sweep both arms **out to your sides** (elbows straight) to a full T-pose.
2. Press and release the **right grip button** again when fully extended.

> [!Note]
> The sweep must take at least 2 seconds. If too fast, the calibrator will warn you and allow a retry.

#### Reading the result

```
=== RESULTS ===
  Human arm reach used   : 0.9150 m
  Robot arm reach        : 1.2926 m

  Recommended scale = 1.2926 / 0.9150 = 1.4126

Update config/sobit_home/quest.yaml:
    scale: 1.4126   # (both right and left)
```

Paste the value into `quest.yaml`:

```yaml
arm_right:
  scale: 1.4126
arm_left:
  scale: 1.4126
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Simulation Mode

When running with a Gazebo simulation, pass `use_sim_time:=true` so all nodes use the simulation clock:

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=keyboard \
  use_sim_time:=true
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [ ] Add pseudo inverse kinematics
- [ ] Migrate parameter loading to `generate_parameter_library` (config typos become startup errors)
- [ ] Restore `check_collisions: true` once left-arm hardware is back (exclude pairs via ACM if needed)

See the [open issues][issues-url] for a full list of proposed features and known issues.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_teleop/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_teleop/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_teleop/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_teleop/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[license-url]: LICENSE
