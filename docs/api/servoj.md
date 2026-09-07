---
title: ⚡ ServoJ 实时控制
outline: [2, 4]
---

# ⚡ ServoJ 实时控制

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 6.1 消息结构

单臂消息 `rokae_interfaces/msg/ServoJ`：

```text
bool enable
float64[7] positions
```

双臂消息 `rokae_interfaces/msg/DualArmServoJ`：

```text
bool enable
float64[7] left_positions
float64[7] right_positions
```

数组顺序均为 `joint_1 ... joint_7`，单位 rad。`enable=true` 表示启动或更新
目标；`enable=false` 表示停止相应通道，此时位置数组会被忽略。

## 6.2 工作方式与安全约束

ROS 订阅回调只保存最新目标，驱动线程按照 `period_s` 固定周期调用 SDK
`sendCommand(JointPosition)`。启动顺序为 `RtCommand`、`setServoJoint()`、
`startMove(jointPosition)`；停止后调用 `stopServoJoint()` 并恢复
`NrtCommand`。

- 左、右单臂通道可以分别运行；双臂通道必须同时取得两侧控制锁。
- ServoJ 与 MoveAbsJ、MoveL、初始化共用 `commandMutex`，同一机械臂不能被
  两种运动接口同时控制。
- 首帧相对于当前反馈以及相邻目标之间的每关节变化不得超过
  `max_command_step_rad`。
- 每个目标必须位于控制器软限位以内，并保留 `soft_limit_margin_rad` 余量。
- 超过 `command_timeout_s` 没有收到新目标时，驱动会停止实时模式。
- 驱动不会自动上电；开始前必须显式调用 `/initialize_robots`。

QoS 为 `KeepLast(1) + best_effort + volatile`，防止旧目标在队列中累积。

## 6.3 发布示例

以下命令只是消息格式示例。真实控制时应由控制程序持续发布，并从当前
`joint_states` 开始，以不超过 `max_command_step_rad` 的小步长更新：

```bash
ros2 topic pub -r 50 /left_arm/servoj rokae_interfaces/msg/ServoJ \
  "{enable: true, positions: [J1, J2, J3, J4, J5, J6, J7]}"

ros2 topic pub -r 50 /dual_arm/servoj \
  rokae_interfaces/msg/DualArmServoJ \
  "{enable: true, left_positions: [L1, L2, L3, L4, L5, L6, L7], \
right_positions: [R1, R2, R3, R4, R5, R6, R7]}"
```

显式停止：

```bash
ros2 topic pub --once /left_arm/servoj rokae_interfaces/msg/ServoJ \
  "{enable: false, positions: [0, 0, 0, 0, 0, 0, 0]}"
```

停止持续发布也会在默认 0.10 s 后触发看门狗停止。
