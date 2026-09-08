---
title: 完整 API 文档
outline: false
---

# Rokae 双臂 ROS 2 接口技术文档

> 文档状态：当前工作区实现
> ROS 版本：ROS 2 Humble
> SDK：xCoreSDK v0.7.1.ar_6
> 最后核对：2026-09-08

## 1. 文档范围

本文档描述 `rokae_ws` 当前对外提供的 ROS 2 接口，重点是
`ros_dual_arm_driver` 中的机械臂、状态、初始化和灵巧手接口，并附带默认
bringup 中启动的视觉目标服务和可选底盘桥接接口。

接口分层如下：

```text
应用与行为树
  ├── 视觉目标服务
  ├── 底盘桥接
  └── 机械臂客户端
          ↓ ROS 2
ros_dual_arm_driver
  ├── FK / IK Services
  ├── MoveAbsJ Action
  ├── MoveL Services
  ├── ServoJ Realtime Topics
  ├── ServoL Realtime Topics
  ├── State Topics
  ├── Initialization Service
  └── Linker Hand Services
          ↓ xCoreSDK
左臂 ArRobot                 右臂 ArRobot
```

当前驱动同时提供非实时 Move、FK/IK 运动学计算、ServoJ 实时关节位置流和
ServoL 实时笛卡尔位姿流接口。阻抗和力矩控制尚未作为 ROS 接口发布，参见
[实现进度与已知限制](/guide/status)。

## 2. API 快速查询

本节采用固定接口卡片，适合按名称快速检索。字段定义、请求响应和调用示例请点击每张卡片中的 `Type`。

- [上肢状态 Topics（6）](#_2-1-上肢状态-topics-6)
- [ServoJ 实时控制 Topics（3）](#_2-2-servoj-实时控制-topics-3)
- [ServoL 实时控制 Topics（10）](#_2-3-servol-实时控制-topics-10)
- [上肢运动 Actions（2）](#_2-4-上肢运动-actions-2)
- [底层控制 Services（26）](#_2-5-底层控制-services-26)
- [上层视觉目标接口示例](/api/vision)
- [可选底盘桥接接口示例](/api/chassis)

### 2.1 上肢状态 Topics（6）

#### 1. joint_states/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/joint_states/left_arm` |
| Type | [`sensor_msgs/msg/JointState`](/reference/message-types#sensor-msgs-msg-jointstate) |
| Direction | Publish |
| Description | 发布左臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 2. joint_states/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/joint_states/right_arm` |
| Type | [`sensor_msgs/msg/JointState`](/reference/message-types#sensor-msgs-msg-jointstate) |
| Direction | Publish |
| Description | 发布右臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 3. tcp_pose/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/tcp_pose/left_arm` |
| Type | [`geometry_msgs/msg/PoseStamped`](/reference/message-types#geometry-msgs-msg-posestamped) |
| Direction | Publish |
| Description | 发布左臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `left_external_ref` |

#### 4. tcp_pose/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/tcp_pose/right_arm` |
| Type | [`geometry_msgs/msg/PoseStamped`](/reference/message-types#geometry-msgs-msg-posestamped) |
| Direction | Publish |
| Description | 发布右臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `right_external_ref` |

#### 5. jacobian/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/jacobian/left_arm` |
| Type | [`std_msgs/msg/Float64MultiArray`](/reference/message-types#std-msgs-msg-float64multiarray) |
| Direction | Publish |
| Description | 发布左臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

#### 6. jacobian/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/jacobian/right_arm` |
| Type | [`std_msgs/msg/Float64MultiArray`](/reference/message-types#std-msgs-msg-float64multiarray) |
| Direction | Publish |
| Description | 发布右臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

### 2.2 ServoJ 实时控制 Topics（3）

#### 1. servoj/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/left_arm` |
| Type | [`rokae_interfaces/msg/ServoJ`](/reference/message-types#rokae-interfaces-msg-servoj) |
| Direction | Subscribe |
| Description | 左臂实时关节空间位置控制 |
| Note | `enable=true` 更新七关节目标；`false` 停止；单位 rad |

#### 2. servoj/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/right_arm` |
| Type | [`rokae_interfaces/msg/ServoJ`](/reference/message-types#rokae-interfaces-msg-servoj) |
| Direction | Subscribe |
| Description | 右臂实时关节空间位置控制 |
| Note | `enable=true` 更新七关节目标；`false` 停止；单位 rad |

#### 3. servoj/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/dual_arm` |
| Type | [`rokae_interfaces/msg/DualArmServoJ`](/reference/message-types#rokae-interfaces-msg-dualarmservoj) |
| Direction | Subscribe |
| Description | 双臂同周期实时关节空间位置控制 |
| Note | 单帧包含左右各七个关节目标；启动时同时取得双臂控制锁 |

### 2.3 ServoL 实时控制 Topics（10）

#### 1. servol/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/left_arm` |
| Type | [`geometry_msgs/msg/Pose`](/reference/message-types#geometry-msgs-msg-pose) |
| Direction | Subscribe |
| Description | 左臂实时笛卡尔 TCP 位姿目标 |
| Note | 目标位于外部参考系；位置 m、姿态四元数；默认 100 Hz |

#### 2. servol/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/right_arm` |
| Type | [`geometry_msgs/msg/Pose`](/reference/message-types#geometry-msgs-msg-pose) |
| Direction | Subscribe |
| Description | 右臂实时笛卡尔 TCP 位姿目标 |
| Note | 目标位于外部参考系；位置 m、姿态四元数；默认 100 Hz |

#### 3. servol/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/dual_arm` |
| Type | [`rokae_interfaces/msg/DualArmServoL`](/reference/message-types#rokae-interfaces-msg-dualarmservol) |
| Direction | Subscribe |
| Description | 双臂同周期实时笛卡尔 TCP 位姿目标 |
| Note | 单帧携带左右臂目标；同时取得双臂控制锁 |

#### 4. servol/stop

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop` |
| Type | [`std_msgs/msg/Bool`](/reference/message-types#std-msgs-msg-bool) |
| Direction | Subscribe |
| Description | 停止所有 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 5. servol/stop/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/left_arm` |
| Type | [`std_msgs/msg/Bool`](/reference/message-types#std-msgs-msg-bool) |
| Direction | Subscribe |
| Description | 停止左臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 6. servol/stop/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/right_arm` |
| Type | [`std_msgs/msg/Bool`](/reference/message-types#std-msgs-msg-bool) |
| Direction | Subscribe |
| Description | 停止右臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 7. servol/stop/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/dual_arm` |
| Type | [`std_msgs/msg/Bool`](/reference/message-types#std-msgs-msg-bool) |
| Direction | Subscribe |
| Description | 停止双臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 8. servol/command_pose/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/command_pose/left_arm` |
| Type | [`geometry_msgs/msg/PoseStamped`](/reference/message-types#geometry-msgs-msg-posestamped) |
| Direction | Publish |
| Description | 发布驱动实际生成的左臂 EndInRef 命令位姿 |
| Note | 这是命令状态，不是机械臂实测反馈 |

#### 9. servol/command_pose/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/command_pose/right_arm` |
| Type | [`geometry_msgs/msg/PoseStamped`](/reference/message-types#geometry-msgs-msg-posestamped) |
| Direction | Publish |
| Description | 发布驱动实际生成的右臂 EndInRef 命令位姿 |
| Note | 这是命令状态，不是机械臂实测反馈 |

#### 10. servol/status

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/status` |
| Type | [`std_msgs/msg/String`](/reference/message-types#std-msgs-msg-string) |
| Direction | Publish |
| Description | 发布 ServoL 会话阶段、停止原因和 SDK 错误 |
| Note | 诊断信息，不作为实时控制输入 |

### 2.4 上肢运动 Actions（2）

#### 1. move_absj/left_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/aide/upperlimb/move_absj/left_arm` |
| Type | [`control_msgs/action/FollowJointTrajectory`](/reference/action-types#control-msgs-action-followjointtrajectory) |
| Direction | Action Server |
| Description | 左臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

#### 2. move_absj/right_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/aide/upperlimb/move_absj/right_arm` |
| Type | [`control_msgs/action/FollowJointTrajectory`](/reference/action-types#control-msgs-action-followjointtrajectory) |
| Direction | Action Server |
| Description | 右臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

### 2.5 底层控制 Services（26）

#### 1. initialize

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 初始化左右臂并验证上电状态 |
| Note | 同时占用双臂；不发送运动命令 |

#### 2. movej_by_path/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/left_arm` |
| Type | [`rokae_interfaces/srv/MoveJByPath`](/reference/service-types#rokae-interfaces-srv-movejbypath) |
| Direction | Service Server |
| Description | 左臂多轨迹点关节空间运动 |
| Note | 请求携带 2 至 100 个七关节路点；使用 SDK 队列一次执行 |

#### 3. movej_by_path/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/right_arm` |
| Type | [`rokae_interfaces/srv/MoveJByPath`](/reference/service-types#rokae-interfaces-srv-movejbypath) |
| Direction | Service Server |
| Description | 右臂多轨迹点关节空间运动 |
| Note | 请求携带 2 至 100 个七关节路点；使用 SDK 队列一次执行 |

#### 4. movej_by_path/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/dual_arm` |
| Type | [`rokae_interfaces/srv/MoveJByPath`](/reference/service-types#rokae-interfaces-srv-movejbypath) |
| Direction | Service Server |
| Description | 双臂同步多轨迹点关节空间运动 |
| Note | 左右路径点数必须相同；同时取得双臂控制锁 |

#### 5. move_l/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l/left_arm` |
| Type | [`rokae_interfaces/srv/MoveL`](/reference/service-types#rokae-interfaces-srv-movel) |
| Direction | Service Server |
| Description | 左臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 6. move_l/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l/right_arm` |
| Type | [`rokae_interfaces/srv/MoveL`](/reference/service-types#rokae-interfaces-srv-movel) |
| Direction | Service Server |
| Description | 右臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 7. move_l_relative/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_relative/left_arm` |
| Type | [`rokae_interfaces/srv/MoveLRelative`](/reference/service-types#rokae-interfaces-srv-movelrelative) |
| Direction | Service Server |
| Description | 左臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 8. move_l_relative/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_relative/right_arm` |
| Type | [`rokae_interfaces/srv/MoveLRelative`](/reference/service-types#rokae-interfaces-srv-movelrelative) |
| Direction | Service Server |
| Description | 右臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 9. move_l_target/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_target/left_arm` |
| Type | [`rokae_interfaces/srv/MoveLTarget`](/reference/service-types#rokae-interfaces-srv-moveltarget) |
| Direction | Service Server |
| Description | 左臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 10. move_l_target/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_target/right_arm` |
| Type | [`rokae_interfaces/srv/MoveLTarget`](/reference/service-types#rokae-interfaces-srv-moveltarget) |
| Direction | Service Server |
| Description | 右臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 11. get_cartesian_state/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/get_cartesian_state/left_arm` |
| Type | [`rokae_interfaces/srv/GetCartesianState`](/reference/service-types#rokae-interfaces-srv-getcartesianstate) |
| Direction | Service Server |
| Description | 查询左臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 12. get_cartesian_state/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/get_cartesian_state/right_arm` |
| Type | [`rokae_interfaces/srv/GetCartesianState`](/reference/service-types#rokae-interfaces-srv-getcartesianstate) |
| Direction | Service Server |
| Description | 查询右臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 13. control_hand/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/control_hand/left_arm` |
| Type | [`rokae_interfaces/srv/ControlHand`](/reference/service-types#rokae-interfaces-srv-controlhand) |
| Direction | Service Server |
| Description | 通过左臂末端 CAN 控制左灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

#### 14. control_hand/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/control_hand/right_arm` |
| Type | [`rokae_interfaces/srv/ControlHand`](/reference/service-types#rokae-interfaces-srv-controlhand) |
| Direction | Service Server |
| Description | 通过右臂末端 CAN 控制右灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

#### 15. go_home/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/left_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 左臂回到配置的原点关节位置 |
| Note | 使用 MoveAbsJ；不会自动上电；执行期间独占左臂 |

#### 16. go_home/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/right_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 右臂回到配置的原点关节位置 |
| Note | 使用 MoveAbsJ；不会自动上电；执行期间独占右臂 |

#### 17. go_home/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/dual_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 左右臂分别回到各自原点 |
| Note | 同时取得双臂控制锁；任一侧失败会停止两侧 |

#### 18. initialize/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/left_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 完整初始化并上电左臂 |
| Note | 不发送运动指令 |

#### 19. initialize/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/right_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 完整初始化并上电右臂 |
| Note | 不发送运动指令 |

#### 20. initialize/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/dual_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 完整初始化并上电双臂 |
| Note | 与无目标后缀的兼容接口行为一致 |

#### 21. power_on/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/power_on/left_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 按需给左臂上电 |
| Note | 已上电时直接成功返回，避免重复切换模式 |

#### 22. power_on/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/power_on/right_arm` |
| Type | [`std_srvs/srv/Trigger`](/reference/service-types#std-srvs-srv-trigger) |
| Direction | Service Server |
| Description | 按需给右臂上电 |
| Note | 已上电时直接成功返回，避免重复切换模式 |

#### 23. fk/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/fk/left_arm` |
| Type | [`rokae_interfaces/srv/ForwardKinematics`](/reference/service-types#rokae-interfaces-srv-forwardkinematics) |
| Direction | Service Server |
| Description | 根据左臂七关节角计算 TCP 位姿 |
| Note | 只计算，不上电、不运动 |

#### 24. fk/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/fk/right_arm` |
| Type | [`rokae_interfaces/srv/ForwardKinematics`](/reference/service-types#rokae-interfaces-srv-forwardkinematics) |
| Direction | Service Server |
| Description | 根据右臂七关节角计算 TCP 位姿 |
| Note | 只计算，不上电、不运动 |

#### 25. ik/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/ik/left_arm` |
| Type | [`rokae_interfaces/srv/InverseKinematics`](/reference/service-types#rokae-interfaces-srv-inversekinematics) |
| Direction | Service Server |
| Description | 根据左臂 TCP 位姿计算七关节角 |
| Note | SDK 结果经 FK 回算校验；只计算，不运动 |

#### 26. ik/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/ik/right_arm` |
| Type | [`rokae_interfaces/srv/InverseKinematics`](/reference/service-types#rokae-interfaces-srv-inversekinematics) |
| Direction | Service Server |
| Description | 根据右臂 TCP 位姿计算七关节角 |
| Note | SDK 结果经 FK 回算校验；只计算，不运动 |

## 接口类型与示例

完整 API 页面只负责快速查找接口。字段结构、请求/响应以及调用方式分别收录在：

- [Message Type · 消息类型](/reference/message-types)
- [Service Type · 服务类型](/reference/service-types)
- [Action Type · 动作类型](/reference/action-types)

具体运动流程、参数限制和安全说明请从左侧“接口示例”进入对应功能页面。
