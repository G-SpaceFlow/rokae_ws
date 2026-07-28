# Behavior-tree programs

This directory contains version-controlled Rokae task programs:

```text
behavior_trees/
├── examples/     Safe templates and usage examples
├── debug/        Reusable diagnostic programs
└── production/   Reviewed production programs
```

Keep example motion actions disabled and `execution_locked: true`. Before a
real run, copy an example into `debug/` or `production/`, replace every target
with values checked against live robot feedback, enable only the intended
actions, and explicitly set `execution_locked: false`.

For temporary on-site programs that should not be installed with the ROS
package, use `/home/niic/rokae_ws/programs/` and pass the absolute YAML path to
`bt_runner`.
