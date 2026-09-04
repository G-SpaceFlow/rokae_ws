# Rokae 双臂 ROS 2 接口技术文档

> 文档状态：当前工作区实现  
> ROS 版本：ROS 2 Humble  
> SDK：xCoreSDK v0.7.1.ar_6  
> 最后核对：2026-09-04

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
  ├── MoveAbsJ Action
  ├── MoveL Services
  ├── State Topics
  ├── Initialization Service
  └── Linker Hand Services
          ↓ xCoreSDK
左臂 ArRobot                 右臂 ArRobot
```

当前驱动是非实时 ROS 接口。`ServoJ`、`ServoL`、阻抗和力矩控制尚未作为 ROS
接口发布，参见[未实现接口](#12-未实现接口)。

## 2. API 快速查询

本节采用固定接口卡片，适合按名称快速检索。更完整的数据字段、参数约束与调用
示例见后续章节。

- [上肢状态 Topics（6）](#21-上肢状态-topics6)
- [上肢运动 Actions（2）](#22-上肢运动-actions2)
- [底层控制 Services（11）](#23-底层控制-services11)
- [上层视觉目标接口](#13-上层视觉目标接口)
- [可选底盘桥接接口](#14-可选底盘桥接接口)

### 2.1 上肢状态 Topics（6）

#### 1. joint_states/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/left_arm/joint_states` |
| Type | `sensor_msgs/msg/JointState` |
| Direction | Publish |
| Description | 发布左臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 2. joint_states/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/right_arm/joint_states` |
| Type | `sensor_msgs/msg/JointState` |
| Direction | Publish |
| Description | 发布右臂七个关节的当前位置 |
| Note | 当前只填写 `position`，单位 rad；默认 20 Hz，QoS depth 10 |

#### 3. tcp_pose/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/left_arm/tcp_pose` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布左臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `left_external_ref` |

#### 4. tcp_pose/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/right_arm/tcp_pose` |
| Type | `geometry_msgs/msg/PoseStamped` |
| Direction | Publish |
| Description | 发布右臂当前 TCP 位姿 |
| Note | 位置单位 m，姿态为四元数，默认参考帧 `right_external_ref` |

#### 5. jacobian/left_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/left_arm/jacobian` |
| Type | `std_msgs/msg/Float64MultiArray` |
| Direction | Publish |
| Description | 发布左臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

#### 6. jacobian/right_arm

| 字段 | 值 |
| --- | --- |
| Topic Name | `/right_arm/jacobian` |
| Type | `std_msgs/msg/Float64MultiArray` |
| Direction | Publish |
| Description | 发布右臂当前位置的运动 Jacobian |
| Note | SDK 行优先 `6 x 7` 法兰 Jacobian；无订阅者时跳过计算 |

### 2.2 上肢运动 Actions（2）

#### 1. move_absj/left_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/left_arm/move_absj` |
| Type | `control_msgs/action/FollowJointTrajectory` |
| Direction | Action Server |
| Description | 左臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

#### 2. move_absj/right_arm

| 字段 | 值 |
| --- | --- |
| Action Name | `/right_arm/move_absj` |
| Type | `control_msgs/action/FollowJointTrajectory` |
| Direction | Action Server |
| Description | 右臂非实时关节空间点到点运动 |
| Note | 只接受一个七关节位置点；支持反馈、取消、超时与结果检查 |

### 2.3 底层控制 Services（11）

#### 1. initialize_robots

| 字段 | 值 |
| --- | --- |
| Service Name | `/initialize_robots` |
| Type | `std_srvs/srv/Trigger` |
| Direction | Service Server |
| Description | 初始化左右臂并验证上电状态 |
| Note | 同时占用双臂；不发送运动命令 |

#### 2. move_l/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/left_arm/move_l` |
| Type | `rokae_interfaces/srv/MoveL` |
| Direction | Service Server |
| Description | 左臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 3. move_l/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/right_arm/move_l` |
| Type | `rokae_interfaces/srv/MoveL` |
| Direction | Service Server |
| Description | 右臂绝对 TCP 直线运动 |
| Note | 显式接收 `[x,y,z,rx,ry,rz]` 和七轴臂角；阻塞至完成或失败 |

#### 4. move_l_relative/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/left_arm/move_l_relative` |
| Type | `rokae_interfaces/srv/MoveLRelative` |
| Direction | Service Server |
| Description | 左臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 5. move_l_relative/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/right_arm/move_l_relative` |
| Type | `rokae_interfaces/srv/MoveLRelative` |
| Direction | Service Server |
| Description | 右臂相对 TCP 直线运动 |
| Note | 位移相对于外部参考系；保留当前臂角、构型和未覆盖姿态轴 |

#### 6. move_l_target/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/left_arm/move_l_target` |
| Type | `rokae_interfaces/srv/MoveLTarget` |
| Direction | Service Server |
| Description | 左臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 7. move_l_target/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/right_arm/move_l_target` |
| Type | `rokae_interfaces/srv/MoveLTarget` |
| Direction | Service Server |
| Description | 右臂构型保持的绝对目标 MoveL |
| Note | 主要供视觉使用；保留控制器当前臂角、构型和外部轴 |

#### 8. get_cartesian_state/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/left_arm/get_cartesian_state` |
| Type | `rokae_interfaces/srv/GetCartesianState` |
| Direction | Service Server |
| Description | 查询左臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 9. get_cartesian_state/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/right_arm/get_cartesian_state` |
| Type | `rokae_interfaces/srv/GetCartesianState` |
| Direction | Service Server |
| Description | 查询右臂当前 TCP 位姿 |
| Note | 返回 `[x,y,z,rx,ry,rz]`；运动控制锁被占用时查询失败 |

#### 10. control_hand/left_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/left_arm/control_hand` |
| Type | `rokae_interfaces/srv/ControlHand` |
| Direction | Service Server |
| Description | 通过左臂末端 CAN 控制左灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

#### 11. control_hand/right_arm

| 字段 | 值 |
| --- | --- |
| Service Name | `/right_arm/control_hand` |
| Type | `rokae_interfaces/srv/ControlHand` |
| Direction | Service Server |
| Description | 通过右臂末端 CAN 控制右灵巧手 |
| Note | 支持开、半开、闭合、六电机位置、速度和压力读取 |

## 3. 约定

### 3.1 手臂与关节名称

| 机械臂 | 命名空间 | 关节名称 |
| --- | --- | --- |
| 左臂 | `/left_arm` | `left_joint_1` ... `left_joint_7` |
| 右臂 | `/right_arm` | `right_joint_1` ... `right_joint_7` |

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
- `/left_arm/tcp_pose` 默认 `frame_id=left_external_ref`；右臂默认
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
ros2 service call /initialize_robots std_srvs/srv/Trigger "{}"
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
ros2 topic echo /left_arm/joint_states
```

### 5.2 `tcp_pose`

类型为 `geometry_msgs/msg/PoseStamped`：

- `position`：TCP 的 XYZ，单位 m。
- `orientation`：驱动把 SDK 位姿转换成四元数。
- `header.frame_id`：对应手臂配置的外部参考坐标系名称。

```bash
ros2 topic echo /right_arm/tcp_pose
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
ros2 topic echo /left_arm/jacobian --once
```

## 6. MoveAbsJ Action

### 6.1 接口

```text
/left_arm/move_absj
/right_arm/move_absj
Type: control_msgs/action/FollowJointTrajectory
```

这是 `FollowJointTrajectory` 的受限适配器，不是完整轨迹控制器：一个 Goal 只
允许一个轨迹点，并转换为一条 SDK `MoveAbsJCommand`。

### 6.2 Goal 约束

- `trajectory.points` 必须恰好包含 1 个点。
- `joint_names` 必须包含对应手臂全部 7 个关节名；顺序可以不同，驱动按名称
  映射。
- `positions` 必须包含 7 个有限数值，单位 rad。
- `velocities`、`accelerations` 和 `effort` 必须为空。
- `time_from_start` 必须为 0。
- 速度和超时不从 Goal 读取，而由驱动参数控制。

### 6.3 Feedback

| 字段 | 内容 |
| --- | --- |
| `joint_names` | 标准 7 关节名称 |
| `desired.positions` | 最终目标关节角 |
| `actual.positions` | 当前 SDK 关节反馈 |
| `error.positions` | `desired - actual` |

### 6.4 Result 与取消

- 成功：`SUCCESSFUL`，机器人空闲且最大关节误差在配置容差内。
- 目标无效、参数无效或 SDK 前置调用失败：Goal 被拒绝或返回
  `INVALID_GOAL`。
- 超时、反馈异常、越限或超速：停止并复位运动，返回
  `PATH_TOLERANCE_VIOLATED`。
- 机器人停止但没有达到目标容差：`GOAL_TOLERANCE_VIOLATED`。
- Action 取消被接受；驱动执行停止和复位后返回 canceled 状态。

### 6.5 调用示例

```bash
ros2 action send_goal --feedback /left_arm/move_absj \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_joint_1, left_joint_2, left_joint_3, left_joint_4, left_joint_5, left_joint_6, left_joint_7], points: [{positions: [0.0, -1.0, 1.2, 0.0, 0.6, 0.0, 0.0]}]}}"
```

目标值必须替换为现场验证过的安全位置。

## 7. MoveL Services

所有 MoveL 服务是阻塞服务：调用在完成、失败或超时后返回。它们使用 SDK
`NrtCommand + MoveLCommand`，不是实时 ServoL。

### 7.1 绝对 MoveL

```text
/left_arm/move_l
/right_arm/move_l
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
ros2 service call /right_arm/move_l rokae_interfaces/srv/MoveL \
  "{pose: [0.280941, -0.314533, -0.489353, -2.761879, 0.340152, -0.746153], elbow: 0.009431, speed_mm_s: 50.0, zone_mm: 0.0}"
```

### 7.2 相对 MoveL

```text
/left_arm/move_l_relative
/right_arm/move_l_relative
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
ros2 service call /left_arm/move_l_relative \
  rokae_interfaces/srv/MoveLRelative \
  "{translation: [0.0, 0.0, 0.01], orientation_override: [false, false, false], orientation_rpy: [0.0, 0.0, 0.0], speed_mm_s: 20.0, zone_mm: 0.0}"
```

### 7.3 构型保持 MoveL

```text
/left_arm/move_l_target
/right_arm/move_l_target
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

### 7.4 读取笛卡尔状态

```text
/left_arm/get_cartesian_state
/right_arm/get_cartesian_state
Type: rokae_interfaces/srv/GetCartesianState
```

请求为空。成功响应中的 `pose` 是 `[x,y,z,rx,ry,rz]`，单位分别为 m 和 rad。
为了返回同一控制阶段的一致数据，该服务也会尝试取得该手臂的控制锁；手臂被
其他运动命令占用时返回失败。持续监控请使用 `tcp_pose` topic。

## 8. 初始化服务

```text
/initialize_robots
Type: std_srvs/srv/Trigger
```

一次请求同时取得左右臂控制锁，然后依次执行：

```text
检查连接 → NrtCommand → automatic → power on → 验证 PowerState::on
```

它不发送运动命令。任一手臂正在执行 MoveAbsJ 或 MoveL 时，请求会被拒绝。

```bash
ros2 service call /initialize_robots std_srvs/srv/Trigger "{}"
```

## 9. Linker Hand 服务

```text
/left_arm/control_hand
/right_arm/control_hand
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
ros2 service call /left_arm/control_hand \
  rokae_interfaces/srv/ControlHand \
  "{command: motors, values: [255, 160, 69, 69, 69, 69]}"
```

左右手分别串行处理自己的请求。灵巧手使用对应机械臂的共享 SDK 连接和 SDK
调用锁，但不占用机械臂运动控制锁，因此设计上允许手臂运动时控制末端手。

## 10. 参数参考

以下“当前值”来自 `rokae_bringup/config/dual_arm.yaml`，不是经过安全认证的
控制器参数。

### 10.1 网络

| 参数 | 左臂当前值 | 右臂当前值 |
| --- | --- | --- |
| `robot_ip` | `192.168.4.160` | `192.168.2.160` |
| `local_ip` | `192.168.4.10` | `192.168.2.10` |

所有内部节点对同一手臂必须使用完全相同的 IP 参数，否则共享硬件注册表会拒绝
不一致配置。

### 10.2 MoveAbsJ 参数

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

### 10.3 MoveL 参数

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

### 10.4 状态发布参数

| 参数 | 左/右当前值 | 说明 |
| --- | --- | --- |
| `arm_name` | `left` / `right` | 关节名称前缀和共享硬件键 |
| `frame_id` | `left_external_ref` / `right_external_ref` | TCP 消息坐标系 |
| `rate_hz` | 20.0 | 发布频率，有效范围 `(0,200]` |
| `publish_jacobian` | `true` | 是否创建 Jacobian 发布器 |

### 10.5 灵巧手参数

| 参数 | 左手当前值 | 右手当前值 | 说明 |
| --- | ---: | ---: | --- |
| `can_id` | 40 (`0x28`) | 39 (`0x27`) | 标准 CAN ID |
| `receive_timeout_ms` | 200 | 200 | 单次接收超时 |
| `receive_attempts` | 3 | 3 | 接收重试次数 |
| `reply_delay_ms` | 1000 | 1000 | 发送后等待回复的时间 |

## 11. 控制权、并发和状态语义

统一驱动进程只创建两个 `ArRobot`：左、右臂各一个。每只手臂有两类内部锁：

```text
commandMutex：控制任务级独占
sdkMutex：单次 SDK 函数调用串行化
```

`MoveAbsJ`、所有 `MoveL` 运动服务和初始化使用 `commandMutex`。同一只手臂一次
只允许一个控制任务，左右臂可以并行。状态发布和灵巧手只短暂使用 `sdkMutex`。

内部控制锁目前没有发布为 ROS topic，SDK 的 `operationState()` 也没有独立状态
topic。因此：

- `operationState=moving` 表示控制器在运动，不等价于 ROS 控制权归属。
- `commandMutex` 能阻止统一进程内的命令冲突，但无法阻止外部程序直接创建
  `ArRobot` 并绕过驱动。
- 不应在统一驱动运行时并行启动 `cxl/servol.cpp` 等直接 SDK 控制程序。

后续若增加 `control_state`，应同时表达 `occupied`、`owner`、SDK operation
state、实际运动状态、控制模式和错误，而不是只发布一个 `moving` 布尔值。

## 12. 未实现接口

以下能力存在于 xCoreSDK 或本地 `cxl` 示例，但尚未接入统一 ROS 驱动：

| 能力 | 当前状态 |
| --- | --- |
| ServoJ / 实时关节位置控制 | 未接入 |
| ServoL / 实时笛卡尔位置控制 | 未接入 |
| 关节阻抗 | 未接入 |
| 笛卡尔阻抗 | 未接入 |
| 实时力矩控制 | 未接入 |
| SDK 拖动模式 | 未接入 |
| MoveJ、MoveC、MoveCF、MoveSP | 未接入 |
| `/left_arm/control_state`、`/right_arm/control_state` | 尚未定义 |
| 对外控制权获取/释放服务 | 尚未定义 |

实时控制以后应作为独占的会话式接口接入统一驱动，不能把连续 ServoL 目标
简单转换成多次阻塞 MoveL 调用。

## 13. 上层视觉目标接口

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

## 14. 可选底盘桥接接口

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

## 15. 安全使用要求

1. 确认本机网卡已经配置 `192.168.4.10` 和 `192.168.2.10`，并确认左右臂 IP
   没有互换。
2. 第一次调用只使用单臂、小位移、低速度、`zone_mm=0`。
3. 清空工作空间，操作员保持急停可触及。
4. MoveAbsJ 前确认关节顺序、弧度单位和控制器软限位。
5. MoveL 前确认外部参考坐标系、TCP、工具负载、臂角和姿态约定。
6. 不要同时运行绕过 ROS 驱动的 SDK 控制程序。
7. YAML 中的限制值只是软件请求边界，不能替代控制器安全配置、碰撞检测或
   风险评估。

## 16. 实现位置

| 内容 | 源文件 |
| --- | --- |
| 共享 SDK 对象与互斥 | `src/rokae_driver/include/rokae_driver/shared_arm_hardware.hpp` |
| 统一进程入口 | `src/rokae_driver/src/ros_dual_arm_driver.cpp` |
| MoveAbsJ | `src/rokae_driver/src/ros_moveabsj_action_server.cpp` |
| MoveL 与笛卡尔状态 | `src/rokae_driver/src/ros_movel_service.cpp` |
| 状态与 Jacobian | `src/rokae_driver/src/ros_pos_publisher.cpp` |
| 初始化 | `src/rokae_driver/src/ros_robot_initializer_service.cpp` |
| 灵巧手 | `src/rokae_driver/src/ros_hand_service.cpp` |
| 自定义接口定义 | `src/rokae_interfaces/srv/` |
| 部署参数 | `src/rokae_bringup/config/dual_arm.yaml` |
| Launch | `src/rokae_bringup/launch/dual_arm.launch.py` |

接口字段或行为发生变化时，应同时更新本文件、对应 `.srv` 注释和部署参数。
