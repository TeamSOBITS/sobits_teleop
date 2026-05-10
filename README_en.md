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
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
    </li>
    <li>
      <a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#create-config-file">Create Config File</a></li>
        <li><a href="#run-teleop-node">Run Teleop Node</a></li>
        <li><a href="#arm-scale-calibration">Arm Scale Calibration (Meta Quest)</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

<!-- ![SOBITS TELEOP](sobits_teleop/docs/img/sobits_teleop.png) -->

A package for teleoperating SOBITS robots using a joystick (PS4, PS5), Meta Quest, or a keyboard.
For Meta Quest setup instructions, please refer to [this repository](https://github.com/TeamSOBITS/meta_quest_teleoperation).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Setup -->
## Setup

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco|
| Python | 3.12~ |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobits_teleop
    ```

3. Navigate into the repository.
    ```sh
    $ cd sobits_teleop/
    ```

4. Install the dependent packages.
    ```sh
    $ bash install.sh
    ```

5. Compile the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage
Basic Workflow for Using sobits_teleop

1. Create a config file 
   Create a config file corresponding to the robot and the input device.

2. Run the teleop node
   Ensure that the device is connected to the PC, then launch the teleoperation node.

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

---

## Create Config File

Select the devices to be used for the robot and teleoperation (e.g., PS4, Meta Quest, etc.).

Configure the robot settings in sobits_teleop/config/{robot_name}/{robot_name}.yaml and the controller settings in sobits_teleop/config/{robot_name}/{device_name}.yaml.

<details>
<summary>robot.yaml(例) </summary>

```yaml

/**:
  ros__parameters:
  
    robot_topic_name:
      joint_states_topic: joint_states  # Specify the topic name for joint_states
      joint_trajectory_topic: # Specify the joint_trajectory topic names for each part
        head : head_position_controller/joint_trajectory
        body : body_position_controller/joint_trajectory
        arm_left : arm_left_position_controller/joint_trajectory
        arm_right : arm_right_position_controller/joint_trajectory
      cmd_vel_topic : cmd_vel # Specify the topic name for command_velocity
```

</details>

<details>
<summary>{使用するデバイス}.yaml(例) </summary>

```yaml

/**:
  ros__parameters:

control_joints: # Define the joint_trajectory_controllers to be operated
      groups:
      - head
      - body
      - arm_left
      - arm_right
      head:
        names: # Define the joints within the joint_trajectory_controller
        - head_tilt_joint
        - head_pan_joint
        head_tilt_joint:
          button : 2           # Trigger button definition
          fast_button : 6      # Increases movement speed while this button is held
          axis : 1              # Controlled by joystick tilt
          axis_sign : 1         # Invert joystick direction (positive/negative)
          speed : 0.1           # Speed when the trigger button is pressed
          fast_speed : 0.5      # Speed when the fast_button is pressed
      ...

    control_poses: # Move the robot to predefined poses
      trigger : 8        # Activation trigger definition
      pose_list:         # List of predefined poses
        - initial_pose
        - ninja_pose
        - detecting_high_pose
        - pre_manipulation_pose
      initial_pose:
        button : 2             # Sets to initial_pose when pressed while the trigger is active
      ...

    control_velocity: # Control mobile base (wheels)
      button : 5               # Activation trigger definition
      fast_button : 7          # Increases movement speed while this button is held
      linear_x_axis : 1        # Forward/backward movement via joystick tilt
      linear_y_axis : 0        # Lateral (left/right) movement via joystick tilt
      angular_axis : 3         # Rotation (yaw) via joystick tilt
      axis_sign : 1            # Invert rotation direction
      linear_scale : 0.1       # Linear velocity scale
      angular_scale : 0.3      # Angular velocity scale
      fast_linear_scale : 0.2  # Linear velocity scale when fast_button is pressed
      fast_angular_scale : 0.6 # Angular velocity scale when fast_button is pressed
```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Run Teleop Node

Connect the device to your PC via Bluetooth or other means. When using a DualShock controller, run the `jstest-gtk` command to confirm the connection status.
After that, configure the launch file and run it.

> [!Note]
> When using ds4drv inside a Docker container, you must mount `/dev/input/` and `/run/udev`
<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Arm Scale Calibration (Meta Quest)

When using Meta Quest controllers for arm teleoperation, the `scale` parameter in `quest.yaml` maps the human arm reach to the robot arm reach. The `arm_scale_calibrator` tool measures your personal arm reach and computes the correct value automatically.

### When to run

Run this once per operator (or whenever the robot's URDF arm reach changes). The result is a single number to paste into `config/{robot_name}/quest.yaml` under `right.scale` and `left.scale`.

### Configuration

Each robot has its own calibrator config at `config/{robot_name}/arm_scale_calibrator.yaml`:

```yaml
# Full kinematic chain shoulder→EE (metres) — from the robot URDF
robot_arm_reach_m: 1.2926

# TF frames used for controller position lookup.
# Set a frame to "" to disable that arm (e.g. single-arm robots).
right_frame:  "right_controller_odom"
left_frame:   "left_controller_odom"
parent_frame: "base_footprint"

# Axis index of the right grip in the Joy message
grip_axis: 7
```

For a **single-arm robot**, disable the unused side by setting its frame to an empty string:

```yaml
right_frame: "right_controller_odom"
left_frame:  ""   # disabled — only right arm will be measured
```

### How to run

Make sure the Quest is connected and publishing TFs, then:

```sh
ros2 run sobits_teleop arm_scale_calibrator --ros-args \
  --params-file install/sobits_teleop/share/sobits_teleop/config/{robot_name}/arm_scale_calibrator.yaml \
  -p joy_topic:=/{robot_name}/joy
```

> [!Note]
> Replace `{robot_name}` with your robot's name (e.g. `sobit_home`).

### Procedure

The calibrator guides you through two steps using the **right grip button**:

**Step 1 — Set start position**
1. Hold both controllers at your sides in a natural standing position.
2. Extend **both arms straight forward**, pointing toward the robot.
3. Press and release the **right grip button** to capture the start positions.

**Step 2 — Sweep to T-pose**
1. Slowly sweep both arms **out to your sides** (elbows straight) until you reach a full T-pose.
2. Press and release the **right grip button** again when your arms are fully extended sideways.

> [!Note]
> The sweep must take at least 2 seconds. If you release the grip too quickly the calibrator will warn you and wait for another attempt.

### Reading the result

The calibrator prints a summary to the console:

```
=== RESULTS ===
  Human arm reach used   : 0.9150 m
  Robot arm reach        : 1.2926 m

  Recommended scale = 1.2926 / 0.9150 = 1.4126

Update config/sobit_home/quest.yaml:
    scale: 1.4126   # (both right and left)
```

Copy the `scale` value into your robot's `quest.yaml`:

```yaml
right:
  scale: 1.4126
left:
  scale: 1.4126
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MILESTONE -->
## Milestone

- [ ] Add pseudo inverse kinematics
- [ ] Add inverse kinematics support for Meta Quest

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
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
