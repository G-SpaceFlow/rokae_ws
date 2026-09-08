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
[未实现接口](#_13-未实现接口)。

## 2. API 快速查询

本节采用固定接口卡片，适合按名称快速检索。更完整的数据字段、参数约束与调用
示例见后续章节。

- [上肢状态 Topics（6）](#_2-1-上肢状态-topics-6)
- [ServoJ 实时控制 Topics（3）](#_2-2-servoj-实时控制-topics-3)
- [ServoL 实时控制 Topics（10）](#_2-3-servol-实时控制-topics-10)
- [上肢运动 Actions（2）](#_2-4-上肢运动-actions-2)
- [底层控制 Services（26）](#_2-5-底层控制-services-26)
- [上层视觉目标接口](#_14-上层视觉目标接口)
- [可选底盘桥接接口](#_15-可选底盘桥接接口)

### 2.1 上肢状态 Topics（6）

#### 1. joint_states/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/joint_states/left_arm` |
| Type | `sensor_msgs/msg/JointState` |
| Direction | Publish |
| Description | 发布左臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 2. joint_states/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/joint_states/right_arm` |
| Type | `sensor_msgs/msg/JointState` |
| Direction | Publish |
| Description | 发布右臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 3. tcp_pose/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/tcp_pose/left_arm` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布左臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `left_external_ref` |

#### 4. tcp_pose/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/tcp_pose/right_arm` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布右臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `right_external_ref` |

#### 5. jacobian/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/jacobian/left_arm` |
| Type | `std_msgs/msg/Float64MultiArray` |
| Direction | Publish |
| Description | 发布左臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

#### 6. jacobian/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/jacobian/right_arm` |
| Type | `std_msgs/msg/Float64MultiArray` |
| Direction | Publish |
| Description | 发布右臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

### 2.2 ServoJ 实时控制 Topics（3）

#### 1. servoj/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/left_arm` |
| Type | `rokae_interfaces/msg/ServoJ` |
| Direction | Subscribe |
| Description | 左臂实时关节空间位置控制 |
| Note | `enable=true` 更新七关节目标；`false` 停止；单位 rad |

#### 2. servoj/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/right_arm` |
| Type | `rokae_interfaces/msg/ServoJ` |
| Direction | Subscribe |
| Description | 右臂实时关节空间位置控制 |
| Note | `enable=true` 更新七关节目标；`false` 停止；单位 rad |

#### 3. servoj/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servoj/dual_arm` |
| Type | `rokae_interfaces/msg/DualArmServoJ` |
| Direction | Subscribe |
| Description | 双臂同周期实时关节空间位置控制 |
| Note | 单帧包含左右各七个关节目标；启动时同时取得双臂控制锁 |

### 2.3 ServoL 实时控制 Topics（10）

#### 1. servol/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/left_arm` |
| Type | `geometry_msgs/msg/Pose` |
| Direction | Subscribe |
| Description | 左臂实时笛卡尔 TCP 位姿目标 |
| Note | 目标位于外部参考系；位置 m、姿态四元数；默认 100 Hz |

#### 2. servol/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/right_arm` |
| Type | `geometry_msgs/msg/Pose` |
| Direction | Subscribe |
| Description | 右臂实时笛卡尔 TCP 位姿目标 |
| Note | 目标位于外部参考系；位置 m、姿态四元数；默认 100 Hz |

#### 3. servol/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/dual_arm` |
| Type | `rokae_interfaces/msg/DualArmServoL` |
| Direction | Subscribe |
| Description | 双臂同周期实时笛卡尔 TCP 位姿目标 |
| Note | 单帧携带左右臂目标；同时取得双臂控制锁 |

#### 4. servol/stop

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop` |
| Type | `std_msgs/msg/Bool` |
| Direction | Subscribe |
| Description | 停止所有 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 5. servol/stop/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/left_arm` |
| Type | `std_msgs/msg/Bool` |
| Direction | Subscribe |
| Description | 停止左臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 6. servol/stop/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/right_arm` |
| Type | `std_msgs/msg/Bool` |
| Direction | Subscribe |
| Description | 停止右臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 7. servol/stop/dual_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/stop/dual_arm` |
| Type | `std_msgs/msg/Bool` |
| Direction | Subscribe |
| Description | 停止双臂 ServoL 模式 |
| Note | 发布 `true` 生效 |

#### 8. servol/command_pose/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/command_pose/left_arm` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布驱动实际生成的左臂 EndInRef 命令位姿 |
| Note | 这是命令状态，不是机械臂实测反馈 |

#### 9. servol/command_pose/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/command_pose/right_arm` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布驱动实际生成的右臂 EndInRef 命令位姿 |
| Note | 这是命令状态，不是机械臂实测反馈 |

#### 10. servol/status

| 字段 | 值 |
| --- | --- |
| Topic Name | `/aide/upperlimb/servol/status` |
| Type | `std_msgs/msg/String` |
| Direction | Publish |
| Description | 发布 ServoL 会话阶段、停止原因和 SDK 错误 |
| Note | 诊断信息，不作为实时控制输入 |

### 2.4 上肢运动 Actions（2）

#### 1. move_absj/left_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/aide/upperlimb/move_absj/left_arm` |
| Type | `control_msgs/action/FollowJointTrajectory` |
| Direction | Action Server |
| Description | 左臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

#### 2. move_absj/right_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/aide/upperlimb/move_absj/right_arm` |
| Type | `control_msgs/action/FollowJointTrajectory` |
| Direction | Action Server |
| Description | 右臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

### 2.5 底层控制 Services（26）

#### 1. initialize

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 初始化左右臂并验证上电状态 |
| Note | 同时占用双臂；不发送运动命令 |

#### 2. movej_by_path/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/left_arm` |
| Type | `rokae_interfaces/srv/MoveJByPath` |
| Direction | Service Server |
| Description | 左臂多轨迹点关节空间运动 |
| Note | 请求携带 2 至 100 个七关节路点；使用 SDK 队列一次执行 |

#### 3. movej_by_path/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/right_arm` |
| Type | `rokae_interfaces/srv/MoveJByPath` |
| Direction | Service Server |
| Description | 右臂多轨迹点关节空间运动 |
| Note | 请求携带 2 至 100 个七关节路点；使用 SDK 队列一次执行 |

#### 4. movej_by_path/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/movej_by_path/dual_arm` |
| Type | `rokae_interfaces/srv/MoveJByPath` |
| Direction | Service Server |
| Description | 双臂同步多轨迹点关节空间运动 |
| Note | 左右路径点数必须相同；同时取得双臂控制锁 |

#### 5. move_l/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l/left_arm` |
| Type | `rokae_interfaces/srv/MoveL` |
| Direction | Service Server |
| Description | 左臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 6. move_l/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l/right_arm` |
| Type | `rokae_interfaces/srv/MoveL` |
| Direction | Service Server |
| Description | 右臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 7. move_l_relative/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_relative/left_arm` |
| Type | `rokae_interfaces/srv/MoveLRelative` |
| Direction | Service Server |
| Description | 左臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 8. move_l_relative/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_relative/right_arm` |
| Type | `rokae_interfaces/srv/MoveLRelative` |
| Direction | Service Server |
| Description | 右臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 9. move_l_target/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_target/left_arm` |
| Type | `rokae_interfaces/srv/MoveLTarget` |
| Direction | Service Server |
| Description | 左臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 10. move_l_target/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/move_l_target/right_arm` |
| Type | `rokae_interfaces/srv/MoveLTarget` |
| Direction | Service Server |
| Description | 右臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 11. get_cartesian_state/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/get_cartesian_state/left_arm` |
| Type | `rokae_interfaces/srv/GetCartesianState` |
| Direction | Service Server |
| Description | 查询左臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 12. get_cartesian_state/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/get_cartesian_state/right_arm` |
| Type | `rokae_interfaces/srv/GetCartesianState` |
| Direction | Service Server |
| Description | 查询右臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 13. control_hand/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/control_hand/left_arm` |
| Type | `rokae_interfaces/srv/ControlHand` |
| Direction | Service Server |
| Description | 通过左臂末端 CAN 控制左灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

#### 14. control_hand/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/control_hand/right_arm` |
| Type | `rokae_interfaces/srv/ControlHand` |
| Direction | Service Server |
| Description | 通过右臂末端 CAN 控制右灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

#### 15. go_home/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/left_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 左臂回到配置的原点关节位置 |
| Note | 使用 MoveAbsJ；不会自动上电；执行期间独占左臂 |

#### 16. go_home/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/right_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 右臂回到配置的原点关节位置 |
| Note | 使用 MoveAbsJ；不会自动上电；执行期间独占右臂 |

#### 17. go_home/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/go_home/dual_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 左右臂分别回到各自原点 |
| Note | 同时取得双臂控制锁；任一侧失败会停止两侧 |

#### 18. initialize/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/left_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 完整初始化并上电左臂 |
| Note | 不发送运动指令 |

#### 19. initialize/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/right_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 完整初始化并上电右臂 |
| Note | 不发送运动指令 |

#### 20. initialize/dual_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/initialize/dual_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 完整初始化并上电双臂 |
| Note | 与无目标后缀的兼容接口行为一致 |

#### 21. power_on/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/power_on/left_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 按需给左臂上电 |
| Note | 已上电时直接成功返回，避免重复切换模式 |

#### 22. power_on/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/power_on/right_arm` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 按需给右臂上电 |
| Note | 已上电时直接成功返回，避免重复切换模式 |

#### 23. fk/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/fk/left_arm` |
| Type | `rokae_interfaces/srv/ForwardKinematics` |
| Direction | Service Server |
| Description | 根据左臂七关节角计算 TCP 位姿 |
| Note | 只计算，不上电、不运动 |

#### 24. fk/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/fk/right_arm` |
| Type | `rokae_interfaces/srv/ForwardKinematics` |
| Direction | Service Server |
| Description | 根据右臂七关节角计算 TCP 位姿 |
| Note | 只计算，不上电、不运动 |

#### 25. ik/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/ik/left_arm` |
| Type | `rokae_interfaces/srv/InverseKinematics` |
| Direction | Service Server |
| Description | 根据左臂 TCP 位姿计算七关节角 |
| Note | SDK 结果经 FK 回算校验；只计算，不运动 |

#### 26. ik/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/aide/upperlimb/ik/right_arm` |
| Type | `rokae_interfaces/srv/InverseKinematics` |
| Direction | Service Server |
| Description | 根据右臂 TCP 位姿计算七关节角 |
| Note | SDK 结果经 FK 回算校验；只计算，不运动 |

## 3. 约定

### 3.1 手臂与关节名称

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

### 3.2 单位

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

### 3.3 坐标系和姿态

- MoveL 的 TCP 位姿相对于机器人控制器配置的外部参考坐标系
  `CoordinateType::endInRef`。
- `MoveL`、`MoveLRelative` 和 `MoveLTarget` 中的姿态使用 XYZ Euler RPY，单位
  为 rad。
- `/aide/upperlimb/tcp_pose/left_arm` 默认 `frame_id=left_external_ref`；右臂默认
  `frame_id=right_external_ref`。
- Jacobian 是 SDK 返回的法兰相对机器人基座的 Jacobian。它与 `tcp_pose` 的
  TCP/外部参考坐标语义不同，使用前必须按具体控制算法确认工具和坐标变换。

## 4. 启动与发现

### 4.1 启动完整系统

```bash
source /opt/ros/humble/setup.bash
source /home/niic/dbstest_ws/install/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
ros2 launch rokae_bringup dual_arm.launch.py
```

启动驱动不会自动上电，也不会发送运动或灵巧手命令。运动前显式调用：

```bash
ros2 service call /aide/upperlimb/initialize std_srvs/srv/Trigger "{}"
```

### 4.2 Launch 开关

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `params_file` | `rokae_bringup/config/dual_arm.yaml` | 参数文件 |
| `start_state_publisher` | `true` | 关节、TCP 和 Jacobian 发布 |
| `start_move_server` | `true` | MoveAbsJ Action |
| `start_movel_service` | `true` | MoveL 与笛卡尔状态服务 |
| `start_hand_service` | `true` | 灵巧手服务 |
| `start_initializer_service` | `true` | 双臂初始化服务 |
| `start_go_home_service` | `true` | 左、右和双臂回原服务 |
| `start_kinematics_service` | `true` | 左、右臂 FK/IK 计算服务 |
| `start_servoj` | `true` | 左、右和双臂 ServoJ 订阅接口 |
| `start_servol` | `true` | 左、右和双臂 ServoL 订阅接口 |
| `start_movej_by_path_service` | `true` | 左、右和双臂 MoveJ 路径服务 |
| `start_vision_target_server` | `true` | 视觉目标缓存服务 |
| `start_chassis_navigation` | `false` | Seer 底盘桥接 |

### 4.3 检查接口

```bash
ros2 topic list
ros2 service list
ros2 action list
ros2 node list
```

## 5. 状态 Topics

状态发布频率由每只手臂状态节点的 `rate_hz` 参数设置，当前部署值为 20 Hz；
发布器使用队列深度 10。

### 5.1 `joint_states`

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

### 5.2 `tcp_pose`

类型为 `geometry_msgs/msg/PoseStamped`：

- `position`：TCP 的 XYZ，单位 m。
- `orientation`：驱动把 SDK 位姿转换成四元数。
- `header.frame_id`：对应手臂配置的外部参考坐标系名称。

```bash
ros2 topic echo /aide/upperlimb/tcp_pose/right_arm
```

### 5.3 `jacobian`

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

## 6. ServoJ 与 ServoL 实时控制 Topics

### 6.1 消息结构

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

### 6.2 工作方式与安全约束

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

### 6.3 发布示例

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

### 6.4 ServoL 消息与坐标系

单臂输入使用 `geometry_msgs/msg/Pose`，双臂输入使用
`rokae_interfaces/msg/DualArmServoL`：

```text
geometry_msgs/Pose left_pose
geometry_msgs/Pose right_pose
```

输入是 TCP 相对当前外部参考系的绝对目标位姿，位置单位 m，姿态使用单位
四元数。驱动按照 Pico 遥操示例的方式读取启动锚点、`baseFrame` 和当前
`toolset`，把 EndInRef TCP 目标转换成 SDK 需要的 FlanInBase 后下发。

### 6.5 ServoL 工作方式与安全约束

- 左臂、右臂和双臂是三个互斥会话模式；双臂模式同时取得两侧控制锁。
- ROS 回调只更新最新绝对目标，SDK 控制线程默认以 100 Hz 执行。
- 首个目标先与当前 TCP 锚点对齐，再按平移和旋转单周期步长限制逼近目标。
- 超过目标跳变、软限位、状态超时、ROS 看门狗或周期延迟限制会停止会话。
- `command_pose/*` 是驱动生成的命令轨迹，不是机械臂实测反馈；实际 TCP 反馈
  应读取 `/aide/upperlimb/tcp_pose/{target}`。
- ServoL 不会自动上电，启动前必须显式完成对应机械臂初始化。

QoS 为 `KeepLast(1) + best_effort + volatile`，避免实时目标在 DDS 队列中累积。

### 6.6 ServoL 发布与停止示例

以下只展示消息格式。真实机械臂运行前必须确认 TCP、工具、外部参考系、负载、
碰撞阈值和目标可达性，并从当前位姿开始小步发送：

```bash
ros2 topic pub -r 100 /aide/upperlimb/servol/left_arm geometry_msgs/msg/Pose \
  "{position: {x: X, y: Y, z: Z}, orientation: {x: QX, y: QY, z: QZ, w: QW}}"

ros2 topic pub --once /aide/upperlimb/servol/stop/left_arm std_msgs/msg/Bool \
  "{data: true}"
```

## 7. MoveAbsJ Action

### 7.1 接口

```text
/aide/upperlimb/move_absj/left_arm
/aide/upperlimb/move_absj/right_arm
Type: control_msgs/action/FollowJointTrajectory
```

这是 `FollowJointTrajectory` 的受限适配器，不是完整轨迹控制器：一个 Goal 只
允许一个轨迹点，并转换为一条 SDK `MoveAbsJCommand`。

### 7.2 Goal 约束

- `trajectory.points` 必须恰好包含 1 个点。
- `joint_names` 必须包含对应手臂全部 7 个关节名；顺序可以不同，驱动按名称
  映射。
- `positions` 必须包含 7 个有限数值，单位 rad。
- `velocities`、`accelerations` 和 `effort` 必须为空。
- `time_from_start` 必须为 0。
- 速度和超时不从 Goal 读取，而由驱动参数控制。

### 7.3 Feedback

| 字段 | 内容 |
| --- | --- |
| `joint_names` | 标准 7 关节名称 |
| `desired.positions` | 最终目标关节角 |
| `actual.positions` | 当前 SDK 关节反馈 |
| `error.positions` | `desired - actual` |

### 7.4 Result 与取消

- 成功：`SUCCESSFUL`，机器人空闲且最大关节误差在配置容差内。
- 目标无效、参数无效或 SDK 前置调用失败：Goal 被拒绝或返回
  `INVALID_GOAL`。
- 超时、反馈异常、越限或超速：停止并复位运动，返回
  `PATH_TOLERANCE_VIOLATED`。
- 机器人停止但没有达到目标容差：`GOAL_TOLERANCE_VIOLATED`。
- Action 取消被接受；驱动执行停止和复位后返回 canceled 状态。

### 7.5 调用示例

```bash
ros2 action send_goal --feedback /aide/upperlimb/move_absj/left_arm \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], points: [{positions: [0.0, -1.0, 1.2, 0.0, 0.6, 0.0, 0.0]}]}}"
```

目标值必须替换为现场验证过的安全位置。

### 7.6 MoveJ By Path Service

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
默认值见 [11.8 MoveJ By Path 参数](#_11-8-movej-by-path-参数)。

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

## 8. MoveL Services

所有 MoveL 服务是阻塞服务：调用在完成、失败或超时后返回。它们使用 SDK
`NrtCommand + MoveLCommand`，不是实时 ServoL。

### 8.1 绝对 MoveL

```text
/aide/upperlimb/move_l/left_arm
/aide/upperlimb/move_l/right_arm
Type: rokae_interfaces/srv/MoveL
```

请求：

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `pose` | `float64[6]` | m, rad | `[x,y,z,rx,ry,rz]` 绝对 TCP 位姿 |
| `elbow` | `float64` | rad | 七轴臂角 |
| `speed_mm_s` | `float64` | mm/s | TCP 线速度 |
| `zone_mm` | `float64` | mm | 过渡半径，0 表示精确停点 |

响应：`success` 和可诊断的 `message`。绝对接口显式构造新的 SDK
`CartesianPosition` 并设置 `hasElbow=true`。

```bash
ros2 service call /aide/upperlimb/move_l/right_arm rokae_interfaces/srv/MoveL \
  "{pose: [0.280941, -0.314533, -0.489353, -2.761879, 0.340152, -0.746153], elbow: 0.009431, speed_mm_s: 50.0, zone_mm: 0.0}"
```

### 8.2 相对 MoveL

```text
/aide/upperlimb/move_l_relative/left_arm
/aide/upperlimb/move_l_relative/right_arm
Type: rokae_interfaces/srv/MoveLRelative
```

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `translation` | `float64[3]` | m | 外部参考系中的 `[dx,dy,dz]` |
| `orientation_override` | `bool[3]` | - | 是否覆盖对应 RPY 轴 |
| `orientation_rpy` | `float64[3]` | rad | 被覆盖轴的绝对 RPY 值 |
| `speed_mm_s` | `float64` | mm/s | TCP 线速度 |
| `zone_mm` | `float64` | mm | 过渡半径 |

驱动复制控制器当前完整 `CartesianPosition`，保留臂角、构型和外部轴，然后
叠加位移并按选择覆盖姿态。

```bash
ros2 service call /aide/upperlimb/move_l_relative/left_arm \
  rokae_interfaces/srv/MoveLRelative \
  "{translation: [0.0, 0.0, 0.01], orientation_override: [false, false, false], orientation_rpy: [0.0, 0.0, 0.0], speed_mm_s: 20.0, zone_mm: 0.0}"
```

### 8.3 构型保持 MoveL

```text
/aide/upperlimb/move_l_target/left_arm
/aide/upperlimb/move_l_target/right_arm
Type: rokae_interfaces/srv/MoveLTarget
```

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `position` | `float64[3]` | m | 绝对 TCP XYZ |
| `orientation_override` | `bool[3]` | - | 是否覆盖对应 RPY 轴 |
| `orientation_rpy` | `float64[3]` | rad | 被覆盖轴的绝对姿态 |
| `speed_mm_s` | `float64` | mm/s | TCP 线速度 |
| `zone_mm` | `float64` | mm | 过渡半径 |

该接口主要供视觉运动使用。它从当前 SDK 位姿复制臂角、构型和外部轴，只替换
目标位置与指定姿态轴，避免视觉系统伪造七轴臂角。

### 8.4 读取笛卡尔状态

```text
/aide/upperlimb/get_cartesian_state/left_arm
/aide/upperlimb/get_cartesian_state/right_arm
Type: rokae_interfaces/srv/GetCartesianState
```

请求为空。成功响应中的 `pose` 是 `[x,y,z,rx,ry,rz]`，单位分别为 m 和 rad。
为了返回同一控制阶段的一致数据，该服务也会尝试取得该手臂的控制锁；手臂被
其他运动命令占用时返回失败。持续监控请使用 `tcp_pose` topic。

## 9. 初始化与回原服务

### 9.1 初始化

```text
/aide/upperlimb/initialize
/aide/upperlimb/initialize/left_arm
/aide/upperlimb/initialize/right_arm
/aide/upperlimb/initialize/dual_arm
/aide/upperlimb/power_on/left_arm
/aide/upperlimb/power_on/right_arm
Type: std_srvs/srv/Trigger
```

`initialize/left_arm` 和 `initialize/right_arm` 分别处理单臂；`initialize` 与
`initialize/dual_arm` 同时取得左右臂控制锁。完整初始化依次执行：

```text
检查连接 → NrtCommand → automatic → power on → 验证 PowerState::on
```

它不发送运动命令。任一手臂正在执行其他控制任务时，对应请求会被拒绝。
`power_on/{target}` 会先检查当前电源状态；已经上电时直接成功返回，避免重复
切换控制模式，否则执行对应单臂的完整初始化上电流程。

```bash
ros2 service call /aide/upperlimb/initialize std_srvs/srv/Trigger "{}"
```

### 9.2 回原

```text
/aide/upperlimb/go_home/left_arm
/aide/upperlimb/go_home/right_arm
/aide/upperlimb/go_home/dual_arm
Type: std_srvs/srv/Trigger
```

原点来自 SDK 示例 `cxl/moveabsj.cpp`：

```text
left:  [1.712167996,  1.570796327, -1.570796327, 0, 0, 0, 0]
right: [1.7121,      -1.570796327, -1.570796327, 0, 0, 0, 0]
```

服务使用非实时 `MoveAbsJCommand`。单臂服务只取得对应手臂的控制锁；双臂服务
先同时取得两侧控制锁并校验两侧，再依次启动两条命令。任意一侧准备、启动、
速度监控、软限位或到位检查失败时，双臂服务会对两侧执行 `moveReset()`。

```bash
ros2 service call /aide/upperlimb/go_home/left_arm std_srvs/srv/Trigger
ros2 service call /aide/upperlimb/go_home/right_arm std_srvs/srv/Trigger
ros2 service call /aide/upperlimb/go_home/dual_arm std_srvs/srv/Trigger
```

`Trigger` 是固定的空请求类型，不接收关节角；服务内部直接使用上面的原点。
命令中的 `std_srvs/srv/Trigger` 是 ROS 2 通用 CLI 要求填写的接口类型，不是
运动参数。调用会产生真实运动。驱动不会自动上电，调用前应完成初始化并确认
工作空间、负载和急停条件。

## 10. Linker Hand 服务

```text
/aide/upperlimb/control_hand/left_arm
/aide/upperlimb/control_hand/right_arm
Type: rokae_interfaces/srv/ControlHand
```

请求：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command` | `string` | 命令名称，不区分大小写 |
| `values` | `uint8[6]` | M1-M6 参数；部分命令只使用 `values[0]` |

支持命令：

| command | `values` 使用方式 | 说明 |
| --- | --- | --- |
| `open` | 忽略 | 六电机位置设为 255 |
| `half` | 忽略 | 六电机位置设为 160 |
| `close` | 忽略 | 六电机位置设为 69 |
| `position` | 使用 `values[0]` | 六电机使用同一位置 |
| `motors` | 使用全部 6 项 | 分别设置 M1-M6 位置 |
| `joints` | 使用全部 6 项 | `motors` 的兼容别名 |
| `speed` | 使用 `values[0]` | 六电机使用同一速度 |
| `pressure` | 忽略 | 请求压力原始 CAN 响应 |

响应：

| 字段 | 说明 |
| --- | --- |
| `success` | 请求是否成功 |
| `message` | 结果或错误描述 |
| `frame_id` | 匹配回复的 CAN ID |
| `data` | 原始 CAN 回复字节 |

```bash
ros2 service call /aide/upperlimb/control_hand/left_arm \
  rokae_interfaces/srv/ControlHand \
  "{command: motors, values: [255, 160, 69, 69, 69, 69]}"
```

左右手分别串行处理自己的请求。灵巧手使用对应机械臂的共享 SDK 连接和 SDK
调用锁，但不占用机械臂运动控制锁，因此设计上允许手臂运动时控制末端手。

## 11. 参数参考

以下“当前值”来自 `rokae_bringup/config/dual_arm.yaml`，不是经过安全认证的
控制器参数。

### 11.1 网络

| 参数 | 左臂当前值 | 右臂当前值 |
| --- | --- | --- |
| `robot_ip` | `192.168.4.160` | `192.168.2.160` |
| `local_ip` | `192.168.4.10` | `192.168.2.10` |

所有内部节点对同一手臂必须使用完全相同的 IP 参数，否则共享硬件注册表会拒绝
不一致配置。

### 11.2 MoveAbsJ 参数

每个参数分别位于 `left_arm.*` 和 `right_arm.*`：

| 参数 | 当前值 | 有效范围 | 说明 |
| --- | ---: | ---: | --- |
| `speed_mm_s` | 50.0 | `(0,4000]` | SDK MoveAbsJ 速度参数 |
| `joint_speed_scale` | 0.05 | `[0.01,1.0]` | 关节速度比例 |
| `zone_mm` | 0.0 | `[0,200]` | 过渡半径 |
| `timeout_s` | 60.0 | `[1,300]` | 执行超时 |
| `soft_limit_margin_rad` | 0.08 | `[0,0.30]` | 目标与软限位的最小余量 |
| `max_goal_delta_rad` | 2.3 | `(0,pi]` | 单关节最大目标变化量 |
| `max_joint_speed_rad_s` | 0.40 | `(0,3.0]` | 运行时速度监控阈值 |
| `goal_tolerance_rad` | 0.01 | `(0,0.20]` | 最终最大关节误差 |

驱动还要求控制器软限位已启用、机器人已上电且目标位于带余量的软限位内。

### 11.3 MoveL 参数

| 参数 | 当前值 | 驱动约束 | 说明 |
| --- | ---: | ---: | --- |
| `timeout_s` | 60.0 | `[1,300]` | 总执行超时 |
| `max_translation_delta_m` | 0.50 | 需为有效有限值 | 单次请求最大平移距离 |
| `max_rotation_delta_rad` | 1.0 | 需为有效有限值 | 单次请求最大 RPY 差值范数 |
| `max_speed_mm_s` | 250.0 | 请求速度 `(0,max]` | 允许的最大 TCP 速度 |
| `max_zone_mm` | 20.0 | 请求 zone `[0,max]` | 允许的最大过渡半径 |
| `motion_start_timeout_s` | 3.0 | `(0,min(timeout,10)]` | 等待运动状态启动 |
| `goal_position_tolerance_m` | 0.002 | `(0,0.05]` | 最终位置容差 |
| `goal_rotation_tolerance_rad` | 0.02 | `(0,0.20]` | 最终姿态容差 |

### 11.4 状态发布参数

| 参数 | 左/右当前值 | 说明 |
| --- | --- | --- |
| `arm_name` | `left` / `right` | 关节名称前缀和共享硬件键 |
| `frame_id` | `left_external_ref` / `right_external_ref` | TCP 消息坐标系 |
| `rate_hz` | 20.0 | 发布频率，有效范围 `(0,200]` |
| `publish_jacobian` | `true` | 是否创建 Jacobian 发布器 |

### 11.5 灵巧手参数

| 参数 | 左手当前值 | 右手当前值 | 说明 |
| --- | ---: | ---: | --- |
| `can_id` | 40 (`0x28`) | 39 (`0x27`) | 标准 CAN ID |
| `receive_timeout_ms` | 200 | 200 | 单次接收超时 |
| `receive_attempts` | 3 | 3 | 接收重试次数 |
| `reply_delay_ms` | 1000 | 1000 | 发送后等待回复的时间 |

### 11.6 ServoJ 参数

| 参数 | 当前值 | 驱动约束 | 说明 |
| --- | ---: | ---: | --- |
| `period_s` | 0.01 | `[0.001,0.1]` | SDK 固定下发周期，默认 100 Hz |
| `lookahead_s` | 0.01 | `[period_s,1.0]` | SDK ServoJ 前瞻时间 |
| `gain` | 0.0 | `[0,1000]` | SDK ServoJ 控制增益 |
| `command_timeout_s` | 0.10 | `[2*period_s,2.0]` | ROS 目标断流看门狗 |
| `max_command_step_rad` | 0.02 | `(0,0.3]` | 首帧和相邻帧单关节最大变化 |
| `soft_limit_margin_rad` | 0.08 | `[0,0.3]` | 目标与控制器软限位的最小余量 |

### 11.7 回原参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `left_arm.home_positions` | `[1.712167996,1.570796327,-1.570796327,0,0,0,0]` | 左臂原点，rad |
| `right_arm.home_positions` | `[1.7121,-1.570796327,-1.570796327,0,0,0,0]` | 右臂原点，rad |
| `speed_mm_s` | 50.0 | SDK MoveAbsJ 通用速度参数 |
| `joint_speed_scale` | 0.05 | 关节速度比例 |
| `zone_mm` | 0.0 | 原点精确停止，不做过渡 |
| `timeout_s` | 60.0 | 回原超时 |
| `soft_limit_margin_rad` | 0.08 | 目标软限位余量 |
| `max_goal_delta_rad` | 2.3 | 当前点到原点的单关节最大允许差值 |
| `max_joint_speed_rad_s` | 0.40 | 连续两次超限即停止 |
| `goal_tolerance_rad` | 0.01 | 最大到位误差 |

### 11.8 MoveJ By Path 参数

| 参数 | 当前值 | 有效范围 | 说明 |
| --- | ---: | --- | --- |
| `speed_mm_s` | 80.0 | `(0,4000]` | SDK 路径速度参数 |
| `joint_speed_scale` | 0.10 | `[0.01,1.0]` | 关节速度比例 |
| `zone_mm` | 5.0 | `[0,200]` | 路点 blending 半径；0 为精确停点 |
| `timeout_s` | 120.0 | `[1,600]` | 整条路径执行超时 |
| `soft_limit_margin_rad` | 0.08 | `[0,0.30]` | 路点与软限位的最小余量 |
| `max_waypoint_delta_rad` | 0.35 | `(0,pi]` | 当前点/相邻路点单关节最大跳变 |
| `goal_tolerance_rad` | 0.02 | `(0,0.20]` | 最终路点最大关节误差 |

### 11.9 ServoL 参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `period_s` | 0.01 | SDK 下发周期，100 Hz |
| `lookahead_s` | 0.01 | SDK 前瞻时间 |
| `command_timeout_s` | 0.10 | ROS 目标断流看门狗 |
| `state_timeout_s` | 0.10 | 实时关节状态超时 |
| `max_cycle_lateness_s` | 0.04 | 控制周期最大允许延迟 |
| `max_translation_step_m` | 0.0005 | 单周期最大 TCP 平移步长 |
| `max_rotation_step_rad` | 0.0043633231 | 单周期最大姿态步长，约 0.25° |
| `max_target_jump_m` | 0.10 | ROS 相邻目标最大平移跳变 |
| `max_target_jump_rad` | 0.7853981634 | ROS 相邻目标最大姿态跳变 |
| `soft_limit_margin_rad` | 0.08 | 关节软限位余量 |
| `initial_alignment_timeout_s` | 2.0 | 首目标与当前锚点对齐超时 |
| `rt_network_tolerance_percent` | 20 | SDK 实时网络容差百分比 |

### 11.10 FK/IK 参数

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `ik_verify_position_tolerance_m` | 0.0001 | IK 经 FK 回算的位置误差上限 |
| `ik_verify_orientation_tolerance_rad` | 0.001 | IK 经 FK 回算的姿态误差上限 |

## 12. 控制权、并发和状态语义

统一驱动进程只创建两个 `ArRobot`：左、右臂各一个。每只手臂有两类内部锁：

```text
commandMutex：控制任务级独占
sdkMutex：单次 SDK 函数调用串行化
```

`MoveAbsJ`、所有 `MoveL`、ServoJ、ServoL、回原和初始化使用 `commandMutex`。同一只
手臂一次只允许一个控制任务，左右臂可以并行；双臂 ServoJ 和双臂回原同时
取得两侧锁。状态发布和灵巧手只短暂使用 `sdkMutex`。

内部控制锁目前没有发布为 ROS topic，SDK 的 `operationState()` 也没有独立状态
topic。因此：

- `operationState=moving` 表示控制器在运动，不等价于 ROS 控制权归属。
- `commandMutex` 能阻止统一进程内的命令冲突，但无法阻止外部程序直接创建
  `ArRobot` 并绕过驱动。
- 不应在统一驱动运行时并行启动 `cxl/servol.cpp` 等直接 SDK 控制程序。

后续若增加 `control_state`，应同时表达 `occupied`、`owner`、SDK operation
state、实际运动状态、控制模式和错误，而不是只发布一个 `moving` 布尔值。

## 13. 未实现接口

以下能力存在于 xCoreSDK 或本地 `cxl` 示例，但尚未接入统一 ROS 驱动：

| 能力 | 当前状态 |
| --- | --- |
| 关节阻抗 | 未接入 |
| 笛卡尔阻抗 | 未接入 |
| 实时力矩控制 | 未接入 |
| SDK 拖动模式 | 未接入 |
| MoveJ、MoveC、MoveCF、MoveSP | 未接入 |
| `/aide/upperlimb/control_state/left_arm`、`/aide/upperlimb/control_state/right_arm` | 尚未定义 |
| 对外控制权获取/释放服务 | 尚未定义 |

后续实时接口也应像 ServoJ、ServoL 一样作为独占的会话式接口接入统一驱动，
不能把连续实时目标简单转换成多次阻塞运动调用。

## 14. 上层视觉目标接口

本节接口由 `rokae_motion/vision_target_server.py` 提供，不属于 xCoreSDK 驱动。

| Service | 类型 | 作用 |
| --- | --- | --- |
| `/bt_target_server/trigger_detect` | `rokae_interfaces/srv/GetVisionTarget` | 触发检测并缓存/返回目标 |
| `/bt_target_server/get_target` | `rokae_interfaces/srv/GetVisionTarget` | 读取缓存目标 |
| `/bt_target_server/set_offset` | `rokae_interfaces/srv/SetVisionOffset` | 设置内存中的双臂位姿偏移 |
| `/bt_target_server/clear_cache` | `std_srvs/srv/Trigger` | 清除所有缓存目标 |

`GetVisionTarget` 请求主要字段：

- `source`：`yolo`、`aruco`、`box_grab_points` 或 `small_box_target`。
- `echo_topic` / `pub_topic`：兼容动态触发与回显接口。
- `key`、`labels`、`point_names`：目标选择条件。
- `trigger_value`：发给检测器的整数触发值。
- `motion_mode`：行为树视觉运动解释模式，当前支持 0 至 6。

响应包含 `success`、`message`、`left_pose` 和 `right_pose`。视觉服务只计算或
缓存目标，不直接调用 SDK；运动仍通过 `move_l_target` 等驱动接口执行。

固定的视觉依赖 topic 包括：

```text
/yolo_vision/front_points_base_json   std_msgs/msg/String
/yolo_vision/wall_angle               std_msgs/msg/String
/yolo_vision/mode5_points_json        std_msgs/msg/String
/yolo_vision/control                  std_msgs/msg/Int32
/yolo_vision/target_labels            std_msgs/msg/String
/aruco/enable                         std_msgs/msg/Int32
/tool/pose                            geometry_msgs/msg/PoseStamped
/box/enable                           std_msgs/msg/Int32
/box_grab_points                      box_detection_interfaces/msg/BoxGrabPoints
/small_box/enable                     std_msgs/msg/Int32
/small_box/target                     box_detection_interfaces/msg/SmallBoxTarget
```

其中部分 topic 名可以通过 `bt_target_server` 参数覆盖。

## 15. 可选底盘桥接接口

`start_chassis_navigation=true` 时启动：

| 名称 | 类型 | 方向/作用 |
| --- | --- | --- |
| `/scheduler/cmd/chassis` | `std_msgs/msg/String` | 订阅 `LM1`、`LM2`、`LM3` |
| `/chassis/state` | `std_msgs/msg/String` | 发布到站或失败状态 |
| `/bt_navigation_server/cancel_navigation` | `std_srvs/srv/Trigger` | 取消当前导航 |
| `/seer/navigate` | `seer_interfaces/action/Navigate` | 桥接器调用的 Seer Action |

状态映射：

| 命令 | 成功状态 |
| --- | --- |
| `LM1` | `ARRIVE_HOME` |
| `LM2` | `ARRIVE_A` |
| `LM3` | `ARRIVE_B` |

失败状态为 `NAVIGATION_FAILED`，取消状态为 `NAVIGATION_CANCELED`。

## 16. 安全使用要求

1. 确认本机网卡已经配置 `192.168.4.10` 和 `192.168.2.10`，并确认左右臂 IP
   没有互换。
2. 第一次调用只使用单臂、小位移、低速度、`zone_mm=0`。
3. 清空工作空间，操作员保持急停可触及。
4. MoveAbsJ 前确认关节顺序、弧度单位和控制器软限位。
5. MoveL 前确认外部参考坐标系、TCP、工具负载、臂角和姿态约定。
6. ServoJ 必须从当前关节反馈开始连续发送，不能直接发布远离当前位置的目标。
7. ServoL 输入是 TCP 相对外部参考系位姿；驱动转换到法兰相对基坐标系后发送。
8. 不要同时运行绕过 ROS 驱动的 SDK 控制程序。
9. YAML 中的限制值只是软件请求边界，不能替代控制器安全配置、碰撞检测或
   风险评估。

## 17. 实现位置

| 内容 | 源文件 |
| --- | --- |
| 共享 SDK 对象与互斥 | `src/rokae_driver/include/rokae_driver/shared_arm_hardware.hpp` |
| 统一进程入口 | `src/rokae_driver/src/ros_dual_arm_driver.cpp` |
| 接口命名常量 | `src/rokae_driver/include/rokae_driver/interface_names.hpp` |
| MoveAbsJ | `src/rokae_driver/src/ros_moveabsj_action_server.cpp` |
| MoveJ By Path | `src/rokae_driver/src/ros_movej_by_path_service.cpp` |
| MoveL 与笛卡尔状态 | `src/rokae_driver/src/ros_movel_service.cpp` |
| ServoJ 实时关节控制 | `src/rokae_driver/src/ros_servoj_subscriber.cpp` |
| ServoL 实时笛卡尔控制 | `src/rokae_driver/src/ros_servol_subscriber.cpp` |
| FK/IK 运动学服务 | `src/rokae_driver/src/ros_kinematics_service.cpp` |
| 单臂与双臂回原 | `src/rokae_driver/src/ros_go_home_service.cpp` |
| 状态与 Jacobian | `src/rokae_driver/src/ros_pos_publisher.cpp` |
| 初始化 | `src/rokae_driver/src/ros_robot_initializer_service.cpp` |
| 灵巧手 | `src/rokae_driver/src/ros_hand_service.cpp` |
| 自定义接口定义 | `src/rokae_interfaces/msg/`、`src/rokae_interfaces/srv/` |
| 部署参数 | `src/rokae_bringup/config/dual_arm.yaml` |
| Launch | `src/rokae_bringup/launch/dual_arm.launch.py` |

接口字段或行为发生变化时，应同时更新本文件、对应 `.msg`/`.srv` 注释和部署参数。

## 18. FK/IK 运动学服务

运动学服务分别提供左臂和右臂接口：

```text
/aide/upperlimb/fk/left_arm
/aide/upperlimb/fk/right_arm
/aide/upperlimb/ik/left_arm
/aide/upperlimb/ik/right_arm
```

服务使用 xCoreSDK `xMateModel<7>` 和控制器当前 `toolset`。位姿是 TCP 相对当前
外部参考坐标系的结果。服务只计算，不自动上电，也不发送运动指令；但需要驱动
已连接对应机器人，以读取机型模型和活动工具/工件坐标配置。

### 18.1 ForwardKinematics

类型：`rokae_interfaces/srv/ForwardKinematics`。

| 方向 | 字段 | 类型 | 单位/说明 |
| --- | --- | --- | --- |
| 请求 | `joints` | `float64[7]` | J1…J7，rad |
| 响应 | `success` | `bool` | SDK 计算是否成功 |
| 响应 | `message` | `string` | 结果或错误原因 |
| 响应 | `pose` | `geometry_msgs/Pose` | TCP 位姿，位置 m、姿态四元数 |
| 响应 | `elbow` | `float64` | 七轴臂角，rad |
| 响应 | `has_elbow` | `bool` | 臂角是否有效 |
| 响应 | `configuration` | `int32[]` | SDK 关节构型数据 |

```bash
ros2 service call /aide/upperlimb/fk/left_arm \
  rokae_interfaces/srv/ForwardKinematics \
  "{joints: [J1, J2, J3, J4, J5, J6, J7]}"
```

### 18.2 InverseKinematics

类型：`rokae_interfaces/srv/InverseKinematics`。

| 方向 | 字段 | 类型 | 单位/说明 |
| --- | --- | --- | --- |
| 请求 | `pose` | `geometry_msgs/Pose` | TCP 目标，外部参考系 |
| 请求 | `use_elbow` | `bool` | 是否使用指定臂角约束七轴选解 |
| 请求 | `elbow` | `float64` | 指定臂角，rad |
| 响应 | `joints` | `float64[7]` | IK 结果，rad |
| 响应 | `solved_pose` | `geometry_msgs/Pose` | IK 结果经 FK 回算的位姿 |
| 响应 | `position_error_m` | `float64` | FK 回算位置误差，m |
| 响应 | `orientation_error_rad` | `float64` | FK 回算姿态误差，rad |
| 响应 | `success/message` | `bool/string` | 校验结果或错误原因 |

IK 返回前会用同一个模型和 `toolset` 执行 FK 回算；误差超过 11.10 节配置的
阈值时返回失败。七轴机械臂通常存在多组逆解，因此输出关节角不保证与生成目标
位姿时使用的原始关节角完全相同。

### 18.3 Python 联合测试

测试程序先执行 FK，再把输出位姿传给 IK，最后对 IK 结果再次执行 FK：

```bash
source /opt/ros/humble/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
python3 /home/niic/rokae_ws/src/test/test_fk_ik.py
```

程序顶部使用 `ARM_TYPE=1` 选择左臂，`ARM_TYPE=2` 选择右臂。测试不会上电或
控制机械臂运动。
