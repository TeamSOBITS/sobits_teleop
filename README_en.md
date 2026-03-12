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
