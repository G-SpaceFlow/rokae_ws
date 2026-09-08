---
title: 🎯 MoveAbsJ 与非实时路径
outline: [2, 4]
---

# 🎯 MoveAbsJ 与非实时路径

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 7.1 接口

```text
/aide/upperlimb/move_absj/left_arm
/aide/upperlimb/move_absj/right_arm
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
ros2 action send_goal --feedback /aide/upperlimb/move_absj/left_arm \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], points: [{positions: [0.0, -1.0, 1.2, 0.0, 0.6, 0.0, 0.0]}]}}"
```

目标值必须替换为现场验证过的安全位置。

## 7.6 MoveJ By Path Service

```text
/aide/upperlimb/movej_by_path/left_arm
/aide/upperlimb/movej_by_path/right_arm
/aide/upperlimb/movej_by_path/dual_arm
Type: rokae_interfaces/srv/MoveJByPath
```

服务请求中的路径是展平的一维数组，每 7 个数为一个关节路点，顺序为
`joint_1 ... joint_7`，单位 rad：

```text
单臂：joint_positions       = [q1..q7, q1..q7, ...]
双臂：left_joint_positions  = [q1..q7, q1..q7, ...]
      right_joint_positions = [q1..q7, q1..q7, ...]
```

单臂和双臂均要求 2 至 100 个路点；双臂左右路点数量必须相同。速度、关节
速度比例、过渡半径和超时由 `rokae_movej_by_path_service` 参数统一设置，当前
默认值见 [11.8 MoveJ By Path 参数](/ROS2_INTERFACE_REFERENCE#_11-8-movej-by-path-参数)。

平滑策略分两层：服务拒绝非有限值、软限位越界和相邻路点过大的关节跳变；SDK
再通过一次 `moveAppend(vector<MoveAbsJCommand>)` 进行整条路径规划。`zone_mm > 0`
时控制器对相邻路点做 blending，保证速度连续；`zone_mm = 0` 时每个路点精确
停靠，适合调试但会产生明显停顿。服务不会循环调用单点 MoveAbsJ，因此不会在
路点之间人为插入 ROS 调度延迟。

```bash
# 左臂三点路径示例（请替换为现场确认的安全角度）
ros2 service call /aide/upperlimb/movej_by_path/left_arm \
  rokae_interfaces/srv/MoveJByPath \
  "{joint_positions: [0.0, -1.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.1, -0.9, 1.1, 0.0, 0.5, 0.0, 0.0]}"

# 双臂路径必须一一对应、点数相同
ros2 service call /aide/upperlimb/movej_by_path/dual_arm \
  rokae_interfaces/srv/MoveJByPath \
  "{left_joint_positions: [...], right_joint_positions: [...] }"
```

服务执行期间独占对应机械臂控制锁；双臂服务同时取得两侧锁。机器人必须已
初始化、上电且处于空闲状态，服务不会自动上电。
