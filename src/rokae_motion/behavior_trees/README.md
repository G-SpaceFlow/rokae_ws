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
`move_l_relative`, `vision`, visual MoveL `motion_mode` 1 through 6, `hand`
and `wait` in one file. Its Linker Hand values are integer end-CAN protocol
bytes in `[0, 255]`; they are not joint radians. The file stays
execution-locked even though its complete actions are enabled, so `bt_runner`
can validate and preview every field without commanding a robot or triggering
vision.

Vision calibration offsets belong in `config/vision_offsets.yaml`, not in a
frequently edited behavior tree. The reference project's whole-body IK and
waist rotation are intentionally outside this Rokae-only scope.

For temporary on-site programs that should not be installed with the ROS
package, use `/home/niic/rokae_ws/programs/` and pass the absolute YAML path to
`bt_runner`.
