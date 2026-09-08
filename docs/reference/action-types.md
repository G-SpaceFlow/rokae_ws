---
title: Action Type · 动作类型
outline: [2, 3]
---

# Action Type · 动作类型

Action 适合可反馈、可取消的长时间动作。当前上肢关节运动使用 ROS 2 标准
`FollowJointTrajectory` 类型的受限适配。

<a id="control-msgs-action-followjointtrajectory"></a>

## `control_msgs/action/FollowJointTrajectory`

### Goal

本驱动要求 Goal 只包含一个轨迹点，必须给出对应手臂全部七个关节名称和七个
有限位置值；`velocity`、`acceleration`、`effort` 留空，`time_from_start` 为 0。

### Feedback

返回 `desired.positions`、`actual.positions` 和 `error.positions`，用于观察目标、
反馈及关节误差。

### Result 与取消

结果沿用标准错误码；目标越限、超速、超时或不到位会失败。取消请求被接受后，
驱动执行停止与运动复位，再返回 canceled 状态。

```bash
ros2 interface show control_msgs/action/FollowJointTrajectory

ros2 action send_goal --feedback /aide/upperlimb/move_absj/left_arm \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], points: [{positions: [0.0, -1.0, 1.2, 0.0, 0.6, 0.0, 0.0]}]}}"
```

目标关节角必须替换为现场验证过的安全位置。

[查看 MoveAbsJ 完整约束](/api/moveabsj)

