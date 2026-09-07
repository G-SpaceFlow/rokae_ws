---
title: 🎯 MoveAbsJ 与非实时路径
outline: [2, 4]
---

# 🎯 MoveAbsJ 与非实时路径

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 7.1 接口

```text
/left_arm/move_absj
/right_arm/move_absj
Type: control_msgs/action/FollowJointTrajectory
```

这是 `FollowJointTrajectory` 的受限适配器，不是完整轨迹控制器：一个 Goal 只
允许一个轨迹点，并转换为一条 SDK `MoveAbsJCommand`。

## 7.2 Goal 约束

- `trajectory.points` 必须恰好包含 1 个点。
- `joint_names` 必须包含对应手臂全部 7 个关节名；顺序可以不同，驱动按名称
  映射。
- `positions` 必须包含 7 个有限数值，单位 rad。
- `velocities`、`accelerations` 和 `effort` 必须为空。
- `time_from_start` 必须为 0。
- 速度和超时不从 Goal 读取，而由驱动参数控制。

## 7.3 Feedback

| 字段 | 内容 |
| --- | --- |
| `joint_names` | 标准 7 关节名称 |
| `desired.positions` | 最终目标关节角 |
| `actual.positions` | 当前 SDK 关节反馈 |
| `error.positions` | `desired - actual` |

## 7.4 Result 与取消

- 成功：`SUCCESSFUL`，机器人空闲且最大关节误差在配置容差内。
- 目标无效、参数无效或 SDK 前置调用失败：Goal 被拒绝或返回
  `INVALID_GOAL`。
- 超时、反馈异常、越限或超速：停止并复位运动，返回
  `PATH_TOLERANCE_VIOLATED`。
- 机器人停止但没有达到目标容差：`GOAL_TOLERANCE_VIOLATED`。
- Action 取消被接受；驱动执行停止和复位后返回 canceled 状态。

## 7.5 调用示例

```bash
ros2 action send_goal --feedback /left_arm/move_absj \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], points: [{positions: [0.0, -1.0, 1.2, 0.0, 0.6, 0.0, 0.0]}]}}"
```

目标值必须替换为现场验证过的安全位置。
