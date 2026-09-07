---
title: ⚙️ 参数参考
outline: [2, 4]
---

# ⚙️ 参数参考

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

以下“当前值”来自 `rokae_bringup/config/dual_arm.yaml`，不是经过安全认证的
控制器参数。

## 11.1 网络

| 参数 | 左臂当前值 | 右臂当前值 |
| --- | --- | --- |
| `robot_ip` | `192.168.4.160` | `192.168.2.160` |
| `local_ip` | `192.168.4.10` | `192.168.2.10` |

所有内部节点对同一手臂必须使用完全相同的 IP 参数，否则共享硬件注册表会拒绝
不一致配置。

## 11.2 MoveAbsJ 参数

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

## 11.3 MoveL 参数

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

## 11.4 状态发布参数

| 参数 | 左/右当前值 | 说明 |
| --- | --- | --- |
| `arm_name` | `left` / `right` | 关节名称前缀和共享硬件键 |
| `frame_id` | `left_external_ref` / `right_external_ref` | TCP 消息坐标系 |
| `rate_hz` | 20.0 | 发布频率，有效范围 `(0,200]` |
| `publish_jacobian` | `true` | 是否创建 Jacobian 发布器 |

## 11.5 灵巧手参数

| 参数 | 左手当前值 | 右手当前值 | 说明 |
| --- | ---: | ---: | --- |
| `can_id` | 40 (`0x28`) | 39 (`0x27`) | 标准 CAN ID |
| `receive_timeout_ms` | 200 | 200 | 单次接收超时 |
| `receive_attempts` | 3 | 3 | 接收重试次数 |
| `reply_delay_ms` | 1000 | 1000 | 发送后等待回复的时间 |

## 11.6 ServoJ 参数

| 参数 | 当前值 | 驱动约束 | 说明 |
| --- | ---: | ---: | --- |
| `period_s` | 0.02 | `[0.001,0.1]` | SDK 固定下发周期，默认 50 Hz |
| `lookahead_s` | 0.02 | `[period_s,1.0]` | SDK ServoJ 前瞻时间 |
| `gain` | 0.0 | `[0,1000]` | SDK ServoJ 控制增益 |
| `command_timeout_s` | 0.10 | `[2*period_s,2.0]` | ROS 目标断流看门狗 |
| `max_command_step_rad` | 0.02 | `(0,0.3]` | 首帧和相邻帧单关节最大变化 |
| `soft_limit_margin_rad` | 0.08 | `[0,0.3]` | 目标与控制器软限位的最小余量 |

## 11.7 回原参数

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
