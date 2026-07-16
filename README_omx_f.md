# OpenManipulator-X (`omx_f`) teleop — PS4 / keyboard

Teleop profile for the ROBOTIS OpenManipulator-X (5-DOF follower, `omx_f`) using
`sobits_teleop`. Publishes to `/arm_controller/joint_trajectory` and
`/gripper_controller/joint_trajectory`.

## 1. Dependencies (once, into the workspace `src/`)

```bash
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
sudo apt install -y ros-jazzy-joy-linux jstest-gtk
pip install pynput --break-system-packages          # for keyboard_joy
cd ~/colcon_ws && rosdep install --from-paths src --ignore-src -y -r
```

### Gripper override (needed to drive the gripper from the joystick)
`sobits_teleop` commands the gripper as a trajectory, but the stock OMX config uses
a `GripperActionController`. In
`open_manipulator/open_manipulator_bringup/config/omx_f/hardware_controller_manager.yaml`
set the gripper to a trajectory controller:

```yaml
      gripper_controller:
        type: joint_trajectory_controller/JointTrajectoryController
# ...
  gripper_controller:
    ros__parameters:
      joints: [gripper_joint_1]
      command_interfaces: [position]
      state_interfaces: [position, velocity]
      allow_partial_joints_goal: true
```

> This edit is in ROBOTIS's repo, so it's lost on a re-clone. TODO: mirror
> `open_manipulator` under TeamSOBITS (with this change) so it can just be pulled.

## 2. Build

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-up-to open_manipulator_bringup sobits_teleop keyboard_joy
source install/setup.bash
```

If you later `apt upgrade` `ros2_control`, rebuild the OMX driver against it or the
hardware init throws `std::bad_alloc`:
```bash
rm -rf build/dynamixel_* install/dynamixel_* && colcon build --symlink-install --packages-up-to open_manipulator_bringup
```

## 3. Run

Find the follower's port (the OpenRB-150 with 12 V connected):
```bash
ls /dev/serial/by-id/     # e.g. /dev/ttyACM1
```

```bash
# arm (leave running)
ros2 launch open_manipulator_bringup omx_f.launch.py port_name:=/dev/ttyACM1 init_position:=false

# teleop — PS4 (Bluetooth, native driver)
ros2 launch sobits_teleop sobits_teleop.launch.py robot_name:=omx_f device:=ps4 use_ds4drv:=False namespace:=/

# teleop — keyboard (focus the "KeyboardJoy" window it opens)
ros2 launch sobits_teleop sobits_teleop.launch.py robot_name:=omx_f device:=keyboard namespace:=/
```

`namespace:=/` runs teleop in the global namespace so it reaches the (global)
`robot_state_publisher` and loads the joint limits. Omit it (defaults to
`robot_name`) on robots whose bringup is namespaced.

### Controls (hold the button **and** move the left stick)
| Button | Joint | Stick |
|---|---|---|
| ○ | joint1 | left/right |
| ✕ | joint2 | up/down |
| △ | joint3 | up/down |
| □ | joint4 | left/right |
| R1 | joint5 | up/down |
| R2 | gripper | up/down |
| L1 | (hold) faster | — |

## 4. Tuning — `config/omx_f/ps4.yaml` (live: edit + relaunch teleop, no rebuild)

- `command_duration` — trajectory horizon per point (s).
- `max_lead` — how far the command may lead the real joint (rad). Smaller = crisper
  stop; larger = pushes harder (e.g. a wrist working against gravity). Can be set
  per-joint (see `joint4`), which overrides the global value.
- `speed` / `fast_speed` — per-joint step size (fast_speed used while L1 held).
