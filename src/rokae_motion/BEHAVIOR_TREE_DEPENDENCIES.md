# Rokae motion behavior-tree dependency inventory

## Intended first implementation

The reference project does not use a behavior-tree framework. It uses a
custom Python YAML interpreter with this selectable hierarchy:

```text
task -> location -> behavior -> action
```

The first `rokae_motion` implementation should follow that lightweight model.
It makes individual actions easy to select from the command line and avoids
adding a framework before the required control-flow semantics are known.

## Available and configured

These dependencies are installed in the current ROS 2 Humble environment and
are declared by `rokae_motion`:

| Dependency | Planned use |
| --- | --- |
| `rclpy` | ROS 2 nodes, action clients and service clients |
| `action_msgs` | Inspect MoveAbsJ action result status |
| `control_msgs` | `FollowJointTrajectory` MoveAbsJ goals |
| `trajectory_msgs` | MoveAbsJ trajectory points |
| `geometry_msgs` | Standard Cartesian poses at the orchestration layer |
| `rokae_interfaces` | Call the Rokae MoveL services |
| `ament_index_python` | Locate installed YAML configuration |
| `python3-yaml` / `PyYAML` | Load and validate behavior-tree YAML |

`setuptools` remains the Python package build/runtime dependency.

## Available but not configured yet

These packages are installed, but the current arm-only scope does not require
them:

| Dependency | Add when |
| --- | --- |
| `std_msgs` | A workflow introduces simple trigger/status topics |
| `sensor_msgs` | A workflow directly consumes joint or sensor feedback |
| `nav_msgs` | Mobile-base navigation becomes part of the tree |

They should be declared only when an implemented action imports them.

## Missing optional dependencies

| Dependency | Status | Decision needed |
| --- | --- | --- |
| `py_trees` | Not installed | Optional alternative to the custom YAML interpreter |
| `py_trees_ros` | Not installed | Needed only with `py_trees` ROS integration |

No missing package is required for the reference-style first implementation.
Do not install these optional libraries until choosing between the lightweight
YAML interpreter and a framework-backed behavior tree.

## Reference dependencies that do not transfer directly

The reference repository targets ROS 1 and imports `rospy`, `catkin`,
`actionlib`, `tf`, and project-specific `upperlimb`, `hand`, and `navigation`
interfaces. They must not be copied into this ROS 2 package.

| ROS 1 reference | Rokae ROS 2 equivalent |
| --- | --- |
| `rospy` | `rclpy` |
| `catkin` | `ament_python` |
| `actionlib` | `rclpy.action` |
| `upperlimb/MoveJ` | `/left_arm/move_absj`, `/right_arm/move_absj` |
| `upperlimb/MoveL` | `/left_arm/move_l`, `/right_arm/move_l` |
| custom hand/navigation/vision services | Not available; define only when those devices enter scope |

## Next implementation boundary

The next stage can add:

1. A YAML schema with `task/location/behavior/action`.
2. An executor that selects `task_id`, `behavior_id`, or `action_id`.
3. Separate adapters for MoveAbsJ, MoveL and wait actions.
4. Dry-run validation by default, with explicit confirmation before motion.

Vision, gripper and navigation actions should wait until their ROS 2
interfaces are specified.
