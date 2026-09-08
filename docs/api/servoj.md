---
title: ⚡ ServoJ 与 ServoL 实时控制
outline: [2, 4]
---

# ⚡ ServoJ 与 ServoL 实时控制

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
- 驱动不会自动上电；开始前必须显式调用 `/aide/upperlimb/initialize`。

QoS 为 `KeepLast(1) + best_effort + volatile`，防止旧目标在队列中累积。

## 6.3 发布示例

以下命令只是消息格式示例。真实控制时应由控制程序持续发布，并从当前
`joint_states` 开始，以不超过 `max_command_step_rad` 的小步长更新：

```bash
ros2 topic pub -r 100 /aide/upperlimb/servoj/left_arm rokae_interfaces/msg/ServoJ \
  "{enable: true, positions: [J1, J2, J3, J4, J5, J6, J7]}"

ros2 topic pub -r 100 /aide/upperlimb/servoj/dual_arm \
  rokae_interfaces/msg/DualArmServoJ \
  "{enable: true, left_positions: [L1, L2, L3, L4, L5, L6, L7], \
right_positions: [R1, R2, R3, R4, R5, R6, R7]}"
```

显式停止：

```bash
ros2 topic pub --once /aide/upperlimb/servoj/left_arm rokae_interfaces/msg/ServoJ \
  "{enable: false, positions: [0, 0, 0, 0, 0, 0, 0]}"
```

停止持续发布也会在默认 0.10 s 后触发看门狗停止。

## 6.4 ServoL 消息与坐标系

单臂输入使用 `geometry_msgs/msg/Pose`，双臂输入使用
`rokae_interfaces/msg/DualArmServoL`：

```text
geometry_msgs/Pose left_pose
geometry_msgs/Pose right_pose
```

输入是 TCP 相对当前外部参考系的绝对目标位姿，位置单位 m，姿态使用单位
四元数。驱动按照 Pico 遥操示例的方式读取启动锚点、`baseFrame` 和当前
`toolset`，把 EndInRef TCP 目标转换成 SDK 需要的 FlanInBase 后下发。

## 6.5 ServoL 工作方式与安全约束

- 左臂、右臂和双臂是三个互斥会话模式；双臂模式同时取得两侧控制锁。
- ROS 回调只更新最新绝对目标，SDK 控制线程默认以 100 Hz 执行。
- 首个目标先与当前 TCP 锚点对齐，再按平移和旋转单周期步长限制逼近目标。
- 超过目标跳变、软限位、状态超时、ROS 看门狗或周期延迟限制会停止会话。
- `command_pose/*` 是驱动生成的命令轨迹，不是机械臂实测反馈；实际 TCP 反馈
  应读取 `/aide/upperlimb/tcp_pose/{target}`。
- ServoL 不会自动上电，启动前必须显式完成对应机械臂初始化。

QoS 为 `KeepLast(1) + best_effort + volatile`，避免实时目标在 DDS 队列中累积。

## 6.6 ServoL 发布与停止示例

以下只展示消息格式。真实机械臂运行前必须确认 TCP、工具、外部参考系、负载、
碰撞阈值和目标可达性，并从当前位姿开始小步发送：

```bash
ros2 topic pub -r 100 /aide/upperlimb/servol/left_arm geometry_msgs/msg/Pose \
  "{position: {x: X, y: Y, z: Z}, orientation: {x: QX, y: QY, z: QZ, w: QW}}"

ros2 topic pub --once /aide/upperlimb/servol/stop/left_arm std_msgs/msg/Bool \
  "{data: true}"
```
