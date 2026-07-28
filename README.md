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

The launch file starts the state publisher, MoveAbsJ action server and MoveL
service. It does not send a motion command and does not power on either robot.

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
  start_state_publisher:=false start_movel_service:=false

# MoveL services only
ros2 launch rokae_bringup dual_arm.launch.py \
  start_state_publisher:=false start_move_server:=false
```

The action names are:

```text
/left_arm/move_absj
/right_arm/move_absj
```

The linear-motion services are:

```text
/left_arm/move_l
/right_arm/move_l
/left_arm/move_l_relative
/right_arm/move_l_relative
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
bt_runner
moveabsj_client
dual_arm_program
```

### Behavior-tree runner

The behavior-tree runner uses the following YAML hierarchy:

```text
task -> location -> behavior -> action
```

The supported action types are `move_absj`, `move_l`, `move_l_relative` and
`wait`.
The Python files have deliberately separate responsibilities:

```text
run_bt.py             command-line arguments and program entry point
control.py            fixed workflow groups, steps and repeat counts
bt_runner_config.py   dry-run, confirmation and failure policy
bt_executor.py        YAML parsing, validation, filtering and execution flow
bt_actions.py         concrete MoveAbsJ, MoveL and Wait behaviors
executor_services.py  low-level ROS 2 action and service calls
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

The source file can also be run directly after sourcing ROS and the workspace:

```bash
python3 /home/niic/rokae_ws/src/rokae_motion/rokae_motion/control.py
```

Use `--dry-run` when only a preview is needed:

```bash
ros2 run rokae_motion bt_control --dry-run
```

Each workflow group supports `name`, `repeat` and ordered `steps`. A step must
contain `task_id` and may narrow the selection with `location_id`,
`behavior_id` and `action_id`. Actual workflow execution uses the same
`execution_locked` and interactive `EXECUTE` safeguards as `bt_runner`; the
complete expanded workflow is confirmed once. `bt_runner` remains dry-run by
default and still requires its `--execute` option.

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

The installed template is
`rokae_motion/behavior_trees/examples/展会动作.yaml`. Its motion actions
are disabled and the entire file is execution-locked. Version-controlled
programs belong under `behavior_trees/examples`, `behavior_trees/debug` or
`behavior_trees/production`. Temporary on-site programs can live under
`/home/niic/rokae_ws/programs` and be passed to `bt_runner` by absolute path.

### Legacy MoveAbsJ program

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
