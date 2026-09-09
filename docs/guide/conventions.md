---
title: 📐 单位与坐标系
outline: [2, 4]
---

# 📐 单位与坐标系

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 3.1 手臂与关节名称

所有上肢 ROS 接口采用统一格式：

```text
/aide/upperlimb/<function>/<target>
```

- `aide`：机器人名称。
- `upperlimb`：上肢子系统。
- `function`：接口功能，例如 `joint_states`、`servoj`、`move_l`。
- `target`：`left_arm`、`right_arm` 或 `dual_arm`。
- 作用于整个上肢且无需区分目标的接口可省略 `target`，例如
  `/aide/upperlimb/initialize`。

| 机械臂 | 接口目标后缀 | 关节名称 |
| --- | --- | --- |
| 左臂 | `/aide/upperlimb/.../left_arm` | `left_joint_1` ... `left_joint_7` |
| 右臂 | `/aide/upperlimb/.../right_arm` | `right_joint_1` ... `right_joint_7` |

## 3.2 单位

| 物理量 | 单位 |
| --- | --- |
| 关节角、臂角、RPY | rad |
| 关节速度 | rad/s |
| 笛卡尔位置 | m |
| MoveL 线速度 | mm/s |
| 过渡区 `zone_mm` | mm |
| 时间 | s，名称明确为 `_ms` 的参数除外 |
| Jacobian 线速度行 | m/s |
| Jacobian 角速度行 | rad/s |

## 3.3 坐标系和姿态

- MoveL 的 TCP 位姿相对于机器人控制器配置的外部参考坐标系
  `CoordinateType::endInRef`。
- `MoveL` 和 `MoveLRelative` 中的姿态使用 XYZ Euler RPY，单位
  为 rad。
- `/aide/upperlimb/tcp_pose/left_arm` 默认 `frame_id=left_external_ref`；右臂默认
  `frame_id=right_external_ref`。
- `/aide/upperlimb/tcp_state/{arm}` 与 `tcp_pose` 同步发布，并额外包含 SDK
  原始 XYZ Euler RPY 姿态。
- Jacobian 是 SDK 返回的法兰相对机器人基座的 Jacobian。它与 `tcp_pose` 的
  TCP/外部参考坐标语义不同，使用前必须按具体控制算法确认工具和坐标变换。
