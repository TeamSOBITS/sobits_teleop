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
| Ubuntu | 22.04 (Noble Numbat) |
| ROS    | Humble Hawksbill|
| Python | 3.10~ |

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

Select the robot and the input device to be used for teleoperation (e.g., PS4 controller, Meta Quest).

Create a configuration file at: `sobits_teleop/config/{robot}/{device}.yaml`

Then, specify the joints you want to control and the `cmd_vel` settings.

<details>
<summary>Example:</summary>

```yaml
joints:
  head_tilt_joint:
    joint_trajectory_topic: /sobit_home/head_position_controller/joint_trajectory
    mode_button: 2           # △
    fast_mode_button: 6      # L2
    axis: 1                  # Left stick (up/down)
    axis_sign: 1
    speed: 0.1
    fast_speed: 0.5
    min_pos: -0.7853
    max_pos: 0.52

  head_pan_joint:
    joint_trajectory_topic: /sobit_home/head_position_controller/joint_trajectory
    mode_button: 2           # △
    fast_mode_button: 6      # L2
    axis: 0                  # Left stick (left/right)
    axis_sign: 1
    speed: 0.1
    fast_speed: 0.5
    min_pos: -0.8726
    max_pos: 0.8726

cmd_vel:
  cmd_vel_topic: "/sobit_home/cmd_vel"
  mode_button: 5             # R1
  fast_mode_button: 7        # R2
  linear_x_axis: 1           # Left stick (up/down)
  linear_y_axis: 0           # Left stick (left/right)
  angular_axis: 3            # Right stick (left/right)
  linear_scale: 0.1
  angular_scale: 0.1
  fast_linear_scale: 0.2
  fast_angular_scale: 0.2
```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Run Teleop Node

Connect the input device to your PC via Bluetooth or another method, and verify that the joystick controller is recognized using the jstest-gtk command.
After that, configure the launch file and run it.

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
