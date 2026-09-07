---
title: 📐 单位与坐标系
outline: [2, 4]
---

# 📐 单位与坐标系

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 3.1 手臂与关节名称

| 机械臂 | 命名空间 | 关节名称 |
| --- | --- | --- |
| 左臂 | `/left_arm` | `left_joint_1` ... `left_joint_7` |
| 右臂 | `/right_arm` | `right_joint_1` ... `right_joint_7` |

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
- `MoveL`、`MoveLRelative` 和 `MoveLTarget` 中的姿态使用 XYZ Euler RPY，单位
  为 rad。
- `/left_arm/tcp_pose` 默认 `frame_id=left_external_ref`；右臂默认
  `frame_id=right_external_ref`。
- Jacobian 是 SDK 返回的法兰相对机器人基座的 Jacobian。它与 `tcp_pose` 的
  TCP/外部参考坐标语义不同，使用前必须按具体控制算法确认工具和坐标变换。
