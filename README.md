# Rokae ROS 2 workspace

This workspace keeps the application-owned ROS 2 sources separate from the
vendor xCoreSDK. The SDK still supplies its headers, Eigen copy and binary
library.

Expected Linux layout:

```text
/home/niic/
|-- cxl/
|   `-- xCoreSDK-v0.7.1.ar_6/
`-- rokae_ws/
```

## Build

```bash
cd /home/niic/rokae_ws
source /opt/ros/humble/setup.bash

# This matches the current robot computer.
export ROKAE_SDK_ROOT=/home/niic/cxl/xCoreSDK-v0.7.1.ar_6

colcon build --symlink-install
source install/setup.bash
```

The driver selects `lib/Linux/aarch64` or `lib/Linux/x86_64` from the target
processor reported by CMake. Check the robot computer architecture with:

```bash
uname -m
```

For an unsupported or cross-compiled target, select the exact Linux library
explicitly:

```bash
colcon build --symlink-install --cmake-args \
  -DROKAE_SDK_ROOT="/home/niic/cxl/xCoreSDK-v0.7.1.ar_6" \
  -DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.a
```

## Start

The launch file starts the state publisher and the MoveAbsJ action server. It
does not send an action goal and does not power on either robot.

```bash
source /opt/ros/humble/setup.bash
source /home/niic/rokae_ws/install/setup.bash
ros2 launch rokae_bringup dual_arm.launch.py
```

Optional launch switches:

```bash
# State topics only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_move_server:=false

# Action server only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false
```

The action names are:

```text
/left_arm/move_absj
/right_arm/move_absj
```

The state topics are:

```text
/left_arm/joint_states
/left_arm/tcp_pose
/right_arm/joint_states
/right_arm/tcp_pose
```

Before the first motion test, use one arm, a small target delta, low speed, a
clear workspace and an accessible E-stop. The values in
`rokae_bringup/config/dual_arm.yaml` are initial limits, not a replacement for
the controller's safety configuration.

## Motion programs

`rokae_motion` is an `ament_python` package. Its installed entry points are:

```text
moveabsj_client
dual_arm_program
```

Validate a JSON file without moving:

```bash
ros2 run rokae_motion moveabsj_client \
  /path/to/program.json
```

Run the one-command program after editing
`src/rokae_motion/config/dual_moveabsj_program.json`:

```bash
ros2 run rokae_motion dual_arm_program
```

The checked-in JSON file is locked with `replace_before_use: true` and null
joint targets, so it cannot send a motion goal until measured values replace
all null entries.
