---
title: 📊 状态反馈
outline: [2, 4]
---

# 📊 状态反馈

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

状态发布频率由每只手臂状态节点的 `rate_hz` 参数设置，当前部署值为 20 Hz；
发布器使用队列深度 10。

## 5.1 `joint_states`

| 字段 | 当前内容 |
| --- | --- |
| `header.stamp` | 驱动发布时间 |
| `name` | 对应手臂的 7 个标准关节名 |
| `position` | SDK `jointPos()`，7 个关节角，rad |
| `velocity` | 当前未填写 |
| `effort` | 当前未填写 |

查看数据：

```bash
ros2 topic echo /aide/upperlimb/joint_states/left_arm
```

## 5.2 `tcp_pose`

类型为 `geometry_msgs/msg/PoseStamped`：

- `pose.position`：TCP 的 XYZ，单位 m。
- `pose.orientation`：驱动把 SDK 位姿转换成四元数。
- `header.frame_id`：对应手臂配置的外部参考坐标系名称。

```bash
ros2 topic echo /aide/upperlimb/tcp_pose/right_arm
```

## 5.3 `tcp_state`

类型为 `rokae_interfaces/msg/TcpState`，与 `tcp_pose` 使用同一个采样值和
时间戳。`pose` 保留标准位置与四元数，`orientation_rpy` 同时给出 SDK
XYZ Euler `[roll,pitch,yaw]`，单位 rad：

```bash
ros2 topic echo /aide/upperlimb/tcp_state/left_arm
```

## 5.4 `jacobian`

类型为 `std_msgs/msg/Float64MultiArray`。`data` 是按行优先展开的 `6 x 7`
矩阵：

```text
行：vx, vy, vz, wx, wy, wz
列：joint_1, joint_2, ..., joint_7
索引：data[row * 7 + column]
```

`layout`：

| 维度 | label | size | stride |
| --- | --- | ---: | ---: |
| 0 | `twist` | 6 | 42 |
| 1 | `joint` | 7 | 7 |

`publish_jacobian=false` 时不存在对应发布器。启用后若没有订阅者，驱动跳过
Jacobian 计算。

```bash
ros2 topic echo /aide/upperlimb/jacobian/left_arm --once
```
