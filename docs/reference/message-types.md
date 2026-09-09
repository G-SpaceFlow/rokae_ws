---
title: Message Type · 消息类型
outline: [2, 3]
---

# Message Type · 消息类型

本页集中说明完整 API 表格中出现的消息类型。实际安装后的字段定义以
`ros2 interface show <类型>` 输出为准。

## 标准消息

<a id="sensor-msgs-msg-jointstate"></a>

### `sensor_msgs/msg/JointState`

关节状态消息，包含 `name`、`position`、`velocity` 和 `effort` 数组。本驱动当前
发布七个关节的 `position`，顺序由 `name` 明确，角度单位为 rad。

```bash
ros2 interface show sensor_msgs/msg/JointState
ros2 topic echo /aide/upperlimb/joint_states/left_arm --once
```

[查看状态 Topic 示例](/api/state#_5-1-joint-states)

<a id="geometry-msgs-msg-posestamped"></a>

### `geometry_msgs/msg/PoseStamped`

带时间戳和参考坐标系的位姿，核心字段为 `header` 与 `pose`。位置单位 m，姿态为
四元数。

```bash
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 topic echo /aide/upperlimb/tcp_pose/left_arm --once
```

[查看 TCP 状态示例](/api/state#_5-2-tcp-pose)

<a id="std-msgs-msg-float64multiarray"></a>

### `std_msgs/msg/Float64MultiArray`

浮点数组消息。本驱动用它发布行优先的 `6 × 7` Jacobian，`data` 共 42 个元素。

```bash
ros2 interface show std_msgs/msg/Float64MultiArray
ros2 topic echo /aide/upperlimb/jacobian/left_arm --once
```

[查看 Jacobian 示例](/api/state#_5-4-jacobian)

<a id="geometry-msgs-msg-pose"></a>

### `geometry_msgs/msg/Pose`

由 `position {x,y,z}` 和 `orientation {x,y,z,w}` 组成。本驱动将其作为 ServoL 的
绝对 TCP 目标，位置单位 m，姿态必须是单位四元数。

```bash
ros2 interface show geometry_msgs/msg/Pose
```

[查看 ServoL 发布示例](/api/servoj#_6-6-servol-发布与停止示例)

<a id="std-msgs-msg-bool"></a>

### `std_msgs/msg/Bool`

仅包含 `bool data`。ServoL 停止 Topic 使用 `data: true` 请求停止对应实时会话。

```bash
ros2 topic pub --once /aide/upperlimb/servol/stop/left_arm \
  std_msgs/msg/Bool "{data: true}"
```

<a id="std-msgs-msg-string"></a>

### `std_msgs/msg/String`

仅包含 `string data`，用于发布序列化状态或桥接文本。使用前应确认对应 Topic
约定的数据格式，不应把任意字符串直接解释为运动指令。

```bash
ros2 interface show std_msgs/msg/String
```

## Rokae 自定义消息

<a id="rokae-interfaces-msg-tcpstate"></a>

### `rokae_interfaces/msg/TcpState`

```text
std_msgs/Header header
geometry_msgs/Pose pose
float64[3] orientation_rpy
```

与标准 `tcp_pose` 同步发布；`pose` 包含位置和四元数，
`orientation_rpy` 是 SDK XYZ Euler `[roll,pitch,yaw]`，单位 rad。

[查看 TCP 组合状态](/api/state#_5-3-tcp-state)

<a id="rokae-interfaces-msg-servoj"></a>

### `rokae_interfaces/msg/ServoJ`

```text
bool enable
float64[7] positions
```

`positions` 按 J1…J7 排列，单位 rad。`enable=false` 时位置数组被忽略并停止会话。

```bash
ros2 interface show rokae_interfaces/msg/ServoJ
```

[查看单臂 ServoJ 示例](/api/servoj#_6-3-发布示例)

<a id="rokae-interfaces-msg-dualarmservoj"></a>

### `rokae_interfaces/msg/DualArmServoJ`

```text
bool enable
float64[7] left_positions
float64[7] right_positions
```

一帧同时携带左右臂关节目标；双臂模式需要同时取得两侧控制权。

[查看双臂 ServoJ 示例](/api/servoj#_6-3-发布示例)

<a id="rokae-interfaces-msg-dualarmservol"></a>

### `rokae_interfaces/msg/DualArmServoL`

```text
geometry_msgs/Pose left_pose
geometry_msgs/Pose right_pose
```

左右目标均为各自外部参考系中的绝对 TCP 位姿。

[查看 ServoL 接口约束](/api/servoj#_6-4-servol-消息与坐标系)
