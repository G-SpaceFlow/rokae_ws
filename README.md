# Rokae ROS 2 workspace

完整的 Topic、Service、Action、参数、单位和错误行为说明见
[`docs/ROS2_INTERFACE_REFERENCE.md`](docs/ROS2_INTERFACE_REFERENCE.md)。

This workspace keeps the application-owned ROS 2 sources separate from the
vendor xCoreSDK. The SDK still supplies its headers, Eigen copy and binary
library.

Expected Linux layout:

```text
/home/niic/
|-- aide/hardware/arm/
|   `-- xCoreSDK-v0.7.1.ar_6/
|-- dbstest_ws/                 # seer_interfaces / chassis integration
`-- rokae_ws/
```

## Build

```bash
cd /home/niic/rokae_ws
source /opt/ros/humble/setup.bash
source /home/niic/dbstest_ws/install/setup.bash

# This matches the current robot computer.
export ROKAE_SDK_ROOT=/home/niic/aide/hardware/arm/xCoreSDK-v0.7.1.ar_6

# The driver automatically migrates the former /home/niic/cxl SDK cache value.
colcon build --symlink-install
source install/local_setup.bash
```

The driver selects `lib/Linux/aarch64` or `lib/Linux/x86_64` from the target
processor reported by CMake. Check the robot computer architecture with:

```bash
uname -m
```

For a custom SDK, unsupported target or cross-compiled target, configure the
driver package explicitly before building the full workspace:

```bash
colcon build --symlink-install --packages-select rokae_driver --cmake-args \
  -DROKAE_SDK_ROOT="/home/niic/aide/hardware/arm/xCoreSDK-v0.7.1.ar_6" \
  -DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.a
colcon build --symlink-install
```

## Start

The launch file starts the state publisher, MoveAbsJ action server, MoveL
service, Linker Hand end-CAN service, robot-initialization service and the
vision trigger/target cache. Starting the launch file alone does not send a
motion or hand command, trigger vision or power on either robot.

All arm SDK interfaces run inside the single `ros_dual_arm_driver` process.
That process owns exactly two `rokae::ArRobot` objects (one left and one right)
and shares them between state, Jacobian, initialization, MoveAbsJ, MoveL and
hand-CAN interfaces. Per-arm SDK access is serialized, and MoveAbsJ, MoveL and
initialization share a per-arm command lock so independent ROS interfaces
cannot control the same arm concurrently.

```bash
source /opt/ros/humble/setup.bash
source /home/niic/dbstest_ws/install/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
ros2 launch rokae_bringup dual_arm.launch.py
```

The workspace selects `rmw_cyclonedds_cpp` when `RMW_IMPLEMENTATION` is not
already set. This avoids a Fast DDS client-creation hang observed on the robot
computer while still allowing an explicit operator override.

The deployed arm network values, matching the SDK programs under `cxl/`, are:

| Arm | Controller IP | Robot-computer IP |
| --- | --- | --- |
| Left | `192.168.4.160` | `192.168.4.10` |
| Right | `192.168.2.160` | `192.168.2.10` |

Both robot-computer addresses must be assigned to active Ethernet interfaces
before starting any Rokae driver node. The authoritative runtime copy is the
`rokae_bringup/config/dual_arm.yaml` file installed by the workspace.

Optional launch switches:

```bash
# State topics only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_move_server:=false start_movel_service:=false \
  start_hand_service:=false start_initializer_service:=false \
  start_servoj:=false start_go_home_service:=false \
  start_vision_target_server:=false

# Action server only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_movel_service:=false \
  start_hand_service:=false start_initializer_service:=false \
  start_servoj:=false start_go_home_service:=false \
  start_vision_target_server:=false

# MoveL services only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_hand_service:=false start_initializer_service:=false \
  start_servoj:=false start_go_home_service:=false \
  start_vision_target_server:=false

# ServoJ realtime topics only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_movel_service:=false start_hand_service:=false \
  start_initializer_service:=false start_go_home_service:=false \
  start_vision_target_server:=false

# Go-home services only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_movel_service:=false start_hand_service:=false \
  start_initializer_service:=false start_servoj:=false \
  start_vision_target_server:=false

# Linker Hand services only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_movel_service:=false start_initializer_service:=false \
  start_servoj:=false start_go_home_service:=false \
  start_vision_target_server:=false

# Robot initialization service only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_movel_service:=false start_hand_service:=false \
  start_servoj:=false start_go_home_service:=false \
  start_vision_target_server:=false

# Vision trigger and target cache only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false \
  start_movel_service:=false start_hand_service:=false \
  start_initializer_service:=false start_servoj:=false \
  start_go_home_service:=false
```

The action names are:

```text
/left_arm/move_absj
/right_arm/move_absj
```

The realtime joint-position command topics are:

```text
/left_arm/servoj   rokae_interfaces/msg/ServoJ
/right_arm/servoj  rokae_interfaces/msg/ServoJ
/dual_arm/servoj   rokae_interfaces/msg/DualArmServoJ
```

Single-arm messages contain `enable` and seven joint positions in radians.
The dual-arm message contains `enable` plus left and right seven-joint arrays.
With `enable=true`, publish continuously at the configured 50 Hz and begin at
the current `joint_states`; `enable=false` stops the session. A 0.10 s command
watchdog, controller soft limits, a 0.02 rad per-frame step limit and the same
per-arm command locks used by MoveAbsJ/MoveL protect the interface. The driver
does not power on the arms automatically.

The linear-motion services are:

```text
/left_arm/move_l
/right_arm/move_l
/left_arm/move_l_relative
/right_arm/move_l_relative
/left_arm/move_l_target
/right_arm/move_l_target
/left_arm/get_cartesian_state
/right_arm/get_cartesian_state
```

`move_l_target` is used by visual motion modes. It replaces the absolute TCP
position and only the requested absolute RPY axes while preserving the
controller's current complete seven-axis Cartesian configuration. This avoids
inventing an `elbow` value for a vision target.

The Linker Hand services are:

```text
/left_arm/control_hand
/right_arm/control_hand
```

The `op.cpp`-style initialization service is:

```text
/initialize_robots
```

The fixed home-position services are:

```text
/left_arm/go_home
/right_arm/go_home
/dual_arm/go_home
```

They use `std_srvs/srv/Trigger` and MoveAbsJ. The configured joint targets are
the left and right origin arrays marked in `cxl/moveabsj.cpp`. A request causes
real robot motion, does not power on the arms, and shares the same per-arm
command locks and runtime safety checks as the other motion interfaces.

```bash
ros2 service call /left_arm/go_home std_srvs/srv/Trigger
ros2 service call /right_arm/go_home std_srvs/srv/Trigger
ros2 service call /dual_arm/go_home std_srvs/srv/Trigger
```

These are empty `Trigger` requests. Joint targets are fixed in the driver
configuration and are not supplied on the command line.

One request initializes both arms in order: connect, select
`NrtCommand`, select automatic mode, power on, then verify that the final
power state is `on`. It does not send a motion command. It can be called
independently with:

```bash
ros2 service call /initialize_robots std_srvs/srv/Trigger "{}"
```

The initialization node uses `192.168.4.10` and `192.168.2.10` from the
`rokae_robot_initializer` section of `dual_arm.yaml`, matching the currently
deployed `cxl/op.cpp`.

They expose the `control_hand.cpp` commands `open`, `half`, `close`,
`position`, `motors`/`joints`, `speed` and `pressure`. Position and speed
values are integer Linker Hand protocol bytes in `[0, 255]`. For example:

```bash
ros2 service call /left_arm/control_hand \
  rokae_interfaces/srv/ControlHand \
  "{command: motors, values: [255, 160, 69, 69, 69, 69]}"
```

The request pose is an absolute TCP pose relative to the controller's external
reference frame: `[x, y, z, rx, ry, rz]`, with XYZ in metres and XYZ Euler
angles in radians. `elbow` is radians, `speed_mm_s` is mm/s and `zone_mm` is
the blending radius in mm. For example:

```bash
ros2 service call /right_arm/move_l rokae_interfaces/srv/MoveL \
  "{pose: [0.280941, -0.314533, -0.489353, -2.761879, 0.340152, -0.746153], \
elbow: 0.009431, speed_mm_s: 50.0, zone_mm: 0.0}"
```

The driver rejects a request that exceeds the configured per-command
translation, orientation, speed or zone limits. Inspect and tune the
`rokae_movel_service` section of `dual_arm.yaml` for the actual cell.

The state topics are:

```text
/left_arm/joint_states
/left_arm/tcp_pose
/left_arm/jacobian
/right_arm/joint_states
/right_arm/tcp_pose
/right_arm/jacobian
```

Each Jacobian topic uses `std_msgs/msg/Float64MultiArray`. Its `data` field is
the xCoreSDK row-major `6 x 7` flange Jacobian: rows are
`vx, vy, vz, wx, wy, wz`, and columns are `joint_1 ... joint_7`. The layout
dimensions are labelled `twist` (size 6, stride 42) and `joint` (size 7,
stride 7). The matrix is expressed relative to each robot's base frame.

`publish_jacobian` is enabled for both state-publisher instances in
`dual_arm.yaml`. It can disable only the Jacobian publishers while leaving
joint and TCP state publishing active. The publisher skips model computation
when the topic has no subscribers. The launch argument
`start_state_publisher:=false` instead disables the complete state publisher,
including joint, TCP and Jacobian topics.

Before the first motion test, use one arm, a small target delta, low speed, a
clear workspace and an accessible E-stop. The values in
`rokae_bringup/config/dual_arm.yaml` are initial limits, not a replacement for
the controller's safety configuration.

## Motion programs

`rokae_motion` is an `ament_python` package. Its installed entry points are:

```text
bt_runner
bt_control
vision_target_server
chassis_navigation
```

### Behavior-tree runner

The behavior-tree runner uses the following YAML hierarchy:

```text
task -> location -> behavior -> action
```

The supported action types are `move_absj`, `move_l`, `move_l_relative`,
`vision`, `move_l_vision`, `navigate`, `hand` and `wait`.
The Python files have deliberately separate responsibilities:

```text
run_bt.py             command-line arguments and program entry point
control.py            fixed workflow groups, steps and repeat counts
bt_runner_config.py   dry-run, confirmation and failure policy
bt_executor.py        YAML parsing, validation, filtering and execution flow
bt_actions.py         concrete arm, hand and Wait behaviors
executor_services.py  low-level ROS 2 action and service calls
chassis_navigation.py scheduler-command to Seer station bridge
vision_motion.py      pure motion_mode 1-6 target calculations
vision_target_server.py  detector trigger and in-memory target cache
```

Validate and display the installed safe template without moving:

```bash
ros2 run rokae_motion bt_runner
```

Use a YAML file outside the package:

```bash
ros2 run rokae_motion bt_runner /path/to/tree.yaml
```

Select any hierarchy level for focused debugging:

```bash
ros2 run rokae_motion bt_runner /path/to/tree.yaml \
  --task-id task_001 \
  --location-id location_001 \
  --behavior-id behavior_001 \
  --action-id action_002
```

Dry-run is always the default. Actual execution requires all three:

1. Valid values for every enabled motion action.
2. `execution_locked: false` at the YAML root.
3. The `--execute` option followed by typing `EXECUTE` interactively.

```bash
ros2 run rokae_motion bt_runner /path/to/checked_tree.yaml \
  --action-id action_002 --execute
```

For a fixed multi-step workflow, edit `BEHAVIOR_TREE_PATH` and the `WORKFLOW`
list in `rokae_motion/control.py`, then run it without an extra execution
option:

```bash
ros2 run rokae_motion bt_control
```

Before its first enabled action, `bt_control` automatically calls
`/initialize_robots` to initialize and power on both arms. If either arm
cannot be initialized, the workflow stops without sending an action. The
bringup launch must therefore be running with
`start_initializer_service:=true` (the default). This automatic step applies
only to `bt_control`; the general-purpose `bt_runner` does not power on the
robots automatically.

The source file can also be run directly after sourcing ROS and the workspace:

```bash
python3 /home/niic/rokae_ws/src/rokae_motion/rokae_motion/control.py
```

Use `--dry-run` when only a preview is needed:

```bash
ros2 run rokae_motion bt_control --dry-run
```

`--dry-run` returns before creating the ROS executor, so it neither calls the
initialization service nor sends a robot command.

Each workflow group supports `name`, `repeat` and ordered `steps`. A step must
contain `task_id` and may narrow the selection with `location_id`,
`behavior_id` and `action_id`. Actual workflow execution still respects the
YAML `execution_locked` safeguard, but `bt_control` does not require the
interactive `EXECUTE` input. `bt_runner` remains dry-run by default and still
requires its `--execute` option.

An action with `enabled: false` is displayed but skipped. This permits the
checked-in template to retain null target placeholders safely. An action with
`optional: true` logs a runtime failure and continues; all other failures stop
the selected sequence. `--continue-on-action-fail` can be used during
deliberate debugging to continue after any action failure.

A relative linear action offsets the current TCP position and preserves its
seven-axis elbow. Only the fields that need to change have to be written.
Omitted `dx`, `dy` or `dz` fields mean zero displacement. Omitted `rx`, `ry`
or `rz` fields preserve the corresponding current orientation component;
written orientation fields are absolute RPY targets:

```yaml
- id: lift_arms
  name: dual_arm_relative_lift
  type: move_l_relative
  relative_offset:
    rotate_with_waist: false
    left:
      dx: 0.02
      rz: -0.7
    right:
      dz: 0.02
  speed_mm_s: 30.0
  zone_mm: 0.0
```

Offsets are metres in each controller's external-reference frame. Unlike the
reference humanoid project, Rokae has no waist frame, so
`rotate_with_waist: true` is rejected. Orientation values are absolute XYZ
Euler angles in radians, not increments added to the current orientation. If
only `left` or `right` is present, only that arm receives a service request.

### Seer station navigation

The `navigate` behavior uses the same station IDs as the Seer map:

```text
LM1 -> ARRIVE_HOME
LM2 -> ARRIVE_A
LM3 -> ARRIVE_B
```

Start the bridge by itself:

```bash
ros2 run rokae_motion chassis_navigation
```

Or include it in the bringup launch:

```bash
ros2 launch rokae_bringup dual_arm.launch.py \
  start_chassis_navigation:=true
```

The Seer driver must provide the `/seer/navigate` Action. Navigation belongs
to the `location` entry and runs once before that location's behaviors:

```yaml
locations:
  - id: navigate_to_a
    name: navigate_chassis_to_a
    command: LM2
    behaviors:
      # The location's behaviors run after ARRIVE_A.
      ...
```

The stable navigation type and connection/response timeouts are stored in
`rokae_motion/config/navigation.yaml`, so behavior trees only contain the
per-location command.

Use `--skip-nav` with either `bt_runner` or `bt_control` to omit all enabled
location navigation. A timeout, navigation failure or Ctrl+C requests
`/bt_navigation_server/cancel_navigation`, with the Seer cancellation service
used as a fallback.

### Vision target cache and motion modes

The ROS 2 port keeps the reference project's topic and service names:

```text
subscribed JSON:
  /yolo_vision/front_points_base_json
  /yolo_vision/wall_angle
  /yolo_vision/mode5_points_json

subscribed ArUco pose:
  /tool/pose

published triggers:
  /yolo_vision/control
  /yolo_vision/target_labels
  /aruco/enable

services:
  /bt_target_server/trigger_detect
  /bt_target_server/get_target
  /bt_target_server/set_offset
```

This component stores parsed detector JSON, named points, box left/right
points, scalar offsets and ArUco tool poses in memory. It does not save
RGB/depth images and its cache is cleared when the node restarts. The detector
itself must be started separately.

Trigger and cache a pair of points manually:

```bash
ros2 service call /bt_target_server/trigger_detect \
  rokae_interfaces/srv/GetVisionTarget \
  "{source: yolo, key: sample, trigger_value: 2, labels: [tray], motion_mode: 0, \
point_names: [left, right]}"
```

Read the same cached points without triggering the detector:

```bash
ros2 service call /bt_target_server/get_target \
  rokae_interfaces/srv/GetVisionTarget \
  "{key: sample, motion_mode: 1, point_names: [left, right]}"
```

For the ArUco tool detector, start `aruco_tool_launch.py` separately and use:

```bash
# Reuse an existing /camera color/depth publisher (default).
ros2 launch aruco_scanner aruco_tool_launch.py

# Or start this workspace's configured RealSense camera. It publishes aligned
# /camera/color/image_raw, /camera/depth/image_rect_raw and camera_info.
ros2 launch aruco_scanner aruco_tool_launch.py start_camera:=true
```

The installed vision stack requires a NumPy 1.x version compatible with the
NVIDIA PyTorch build and an OpenCV release that still supports NumPy 1.x. The
tested versions on this computer are NumPy 1.26.4 and OpenCV 4.11.0; these
constraints are recorded in `aruco_scanner/setup.py`.

Use the following behavior-tree action after the detector is running:

```yaml
- id: detect_tool
  name: detect_target
  type: vision
  source: aruco
  echo_topic: /tool/pose
  pub_topic: /aruco/enable
  key: "1-1-1-1"
  trigger_value: 1
  points: [tool]
```

`pub_topic` and `echo_topic` select the actual trigger publisher and result
subscription used by `vision_target_server`. It publishes `1`, waits for the
first result, continues sampling for one second, caches the newest complete
result, and then publishes `0`.

The two-point box detector uses `source: box_grab_points`,
`pub_topic: /box/enable`, `echo_topic: /box_grab_points`, and
`points: [left_center, right_center]`. Its `BoxGrabPoints` message must use
`base_link`.

Use `type: vision` only to trigger and cache data, followed by
`type: move_l_vision` to consume it. `motion_mode` belongs only on the
`move_l_vision` action. Its six imported modes are:

1. Two named points become independent absolute left/right targets.
2. One point becomes the desired dual-arm midpoint while preserving the
   current arm formation.
3. One point plus per-arm offsets; only written arms move.
4. One target point minus one current point is applied to both arms.
5. Independent target-current point differences are applied per arm.
6. A cached scalar such as `y_offset` corrects a selected axis and hand set.

Modes 4, 5 and 6 support a skip threshold. Modes 2 through 6 preserve current
orientation unless an absolute `rx`, `ry` or `rz` is written. Mode 1 can add
per-arm `dx/dy/dz` offsets and absolute RPY from `relative_offset`; omitted
orientation axes fall back to the active entry in
`config/vision_offsets.yaml`.

The full locked template for all six modes is
`behavior_trees/examples/全部动作示例.yaml`. This Rokae port intentionally
does not include the reference project's whole-body IK, waist motion or waist
coordinate rotation. `rotate_with_waist: true` is rejected during YAML
validation.

A hand action can address one or both hands. The explicit `targets` form is
recommended because it makes the side and six-motor order clear:

```yaml
- id: set_hand_positions
  name: set_dual_hand_positions
  type: hand
  command: motors
  targets:
    # M1-M6: thumb bend, thumb side swing, index, middle, ring, little finger
    left: [255, 160, 69, 69, 69, 69]
    right: [255, 160, 69, 69, 69, 69]
```

Commands `open`, `half`, `close` and `pressure` use a `hands` list:

```yaml
- id: open_left
  type: hand
  command: open
  hands: [left]
```

For compatibility with `zj_robot_bt_action`, `q` may contain exactly 12
values: left M1-M6 followed by right M1-M6. Unlike that humanoid project's
joint values, Rokae Linker Hand values are integer CAN protocol bytes in
`[0, 255]`, not radians:

```yaml
- id: close_both
  type: hand
  q: [69, 69, 69, 69, 69, 69,
      69, 69, 69, 69, 69, 69]
```

The installed templates include
`rokae_motion/behavior_trees/examples/展会动作.yaml` and
`rokae_motion/behavior_trees/examples/全部动作示例.yaml`. The complete
example is execution-locked and covers every supported action type, including
hand speed, presets, independent motor positions, the compatible 12-value
`q` layout and pressure reads.
Version-controlled programs belong under `behavior_trees/examples`,
`behavior_trees/debug` or `behavior_trees/production`. Temporary on-site
programs can live under `/home/niic/rokae_ws/programs` and be passed to
`bt_runner` by absolute path.
