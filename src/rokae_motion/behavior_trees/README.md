# Behavior-tree programs

This directory contains version-controlled Rokae task programs:

```text
behavior_trees/
├── examples/     Safe templates and usage examples
├── debug/        Reusable diagnostic programs
└── production/   Reviewed production programs
```

Keep example files `execution_locked: true`, and disable any action that still
contains target placeholders. Before a real run, copy an example into
`debug/` or `production/`, replace every target with values checked against
live robot feedback, enable only the intended actions, and explicitly set
`execution_locked: false`.

`examples/全部动作示例.yaml` covers `move_absj`, `move_l`,
`move_l_relative`, Seer station `navigate`, `vision`, visual MoveL
`motion_mode` 1 through 6, `hand` and `wait` in one file. Its Linker Hand
values are integer end-CAN protocol bytes in `[0, 255]`; they are not joint
radians. The file stays execution-locked even though its complete actions are
enabled, so `bt_runner` can validate and preview every field without
commanding a robot, chassis or vision trigger.

Seer navigation fields belong directly on a `location`; they are not a
separate item inside `behaviors[].actions`. The location navigation completes
before that location's behaviors begin, or is omitted with `--skip-nav`.
Stable type and timeout defaults belong in `config/navigation.yaml`; each
location only needs its station command.

Vision calibration offsets belong in `config/vision_offsets.yaml`, not in a
frequently edited behavior tree. The reference project's whole-body IK and
waist rotation are intentionally outside this Rokae-only scope.

An ArUco tool-pose action uses `source: aruco`,
`echo_topic: /tool/pose`, `pub_topic: /aruco/enable`, `trigger_value: 1`
and one named point such as `points: [tool]`. A two-point box action uses
`source: box_grab_points`, `echo_topic: /box_grab_points`,
`pub_topic: /box/enable`, and `points: [left_center, right_center]`.
The single-point small-box action uses `source: small_box_target`,
`echo_topic: /small_box/target`, `pub_topic: /small_box/enable`, and
`points: [center]`.
The visual action only caches data; `motion_mode` belongs to the later
`move_l_vision` action. The server keeps sampling for one second after the
first result and caches the newest complete `base_link` data before disabling
detection.

For temporary on-site programs that should not be installed with the ROS
package, use `/home/niic/rokae_ws/programs/` and pass the absolute YAML path to
`bt_runner`.
