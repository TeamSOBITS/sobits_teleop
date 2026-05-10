<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

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
        <li><a href="#moveit-arm-controller-meta-quest">MoveIt Arm Controller (Meta Quest)</a></li>
        <li><a href="#arm-scale-calibration-meta-quest">Arm Scale Calibration (Meta Quest)</a></li>
        <li><a href="#simulation-mode">Simulation Mode</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

A package for teleoperating SOBITS robots using a joystick (PS4, PS5), Meta Quest, or a keyboard.\
For Meta Quest setup instructions, please refer to [this repository](https://github.com/TeamSOBITS/meta_quest_teleoperation).

Supported robots: `sobit_home`, `sobit_pro`, `sobit_edu`, `sobit_mini`, `sobit_light`\
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

Each robot has its own directory under `config/{robot_name}/`:

| File | Purpose |
|---|---|
| `robot.yaml` | ROS topic names for joint controllers and cmd_vel |
| `{device}.yaml` | Button/axis mappings for the selected input device |
| `moveit_arm_controller.yaml` | MoveIt arm tracking parameters (Quest only) |
| `arm_scale_calibrator.yaml` | Arm reach calibration parameters (Quest only) |

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
      trigger: 8
      pose_list:
        - initial_pose
        - pre_manipulation_pose
      initial_pose:
        button: 2

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
| `use_moveit` | `false` | Launch MoveIt arm controller (Quest only) |
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

### MoveIt Arm Controller (Meta Quest)

When using the Quest for arm teleoperation, the `moveit_arm_controller` node uses MoveIt2 to follow the controller's TF pose in real time via Cartesian path planning.

Enable it alongside the Quest device with `use_moveit:=true`:

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  ros_ip:=<YOUR_PC_IP> \
  use_moveit:=true
```

The node is configured via `config/{robot_name}/moveit_arm_controller.yaml`:

```yaml
arm_teleop:
  arms: [arm_left, arm_right]   # must match SRDF planning group names

  arm_left:
    planning_group:    "arm_left"
    target_frame:      "left_target_link"   # TF broadcast by sobits_teleop
    base_frame:        "base_footprint"
    trajectory_topic:  "arm_left_position_controller/joint_trajectory"

  update_rate_hz:          25.0   # tracking loop frequency
  max_cartesian_step_m:     0.5   # max step per cycle
  eef_step_m:              0.03   # Cartesian interpolation resolution
  min_cartesian_fraction:   0.2   # fall back to OMPL below this fraction
  replan_threshold_m:      0.02   # replan when target drifts this far (idle)
  preempt_threshold_m:     0.15   # cancel in-flight trajectory when target moves this far
  arrival_threshold_m:     0.03   # skip planning when EE is this close to target
  velocity_scaling:         0.6
  acceleration_scaling:     0.6
  traj_lookahead_ms:         40
  ompl_planning_timeout_s:  0.5
```

Each arm is enabled/disabled at runtime by publishing to `/{robot_name}/{arm_name}/moveit_track_enabled` (`std_msgs/Bool`). The Quest controller's grip button in `sobits_teleop` publishes to this topic automatically.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

### Arm Scale Calibration (Meta Quest)

The `scale` parameter in `quest.yaml` maps the human arm reach to the robot arm reach. Run `arm_scale_calibrator` once per operator to compute the correct value automatically.

#### When to run

Run once per operator, or whenever the robot URDF arm reach changes. The result is pasted into `config/{robot_name}/quest.yaml` under `right.scale` and `left.scale`.

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
right:
  scale: 1.4126
left:
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
