---
title: Service Type · 服务类型
outline: [2, 3]
---

# Service Type · 服务类型

本页按服务类型汇总请求、响应与调用入口。运动命令执行前必须同时阅读对应功能页
中的限制和安全要求。

<a id="std-srvs-srv-trigger"></a>

## `std_srvs/srv/Trigger`

空请求，响应为 `success` 与 `message`。初始化、上电、回原和停止拖动示教均使用该类型，
但动作语义由服务名称决定。

```bash
ros2 interface show std_srvs/srv/Trigger
ros2 service call /aide/upperlimb/initialize/left_arm std_srvs/srv/Trigger '{}'
```

> 初始化会真实上电，回原会真实运动。网页或命令行调用前必须确认现场安全。

[查看初始化示例](/api/power) · [查看回原示例](/api/home)

<a id="rokae-interfaces-srv-movejbypath"></a>

## `rokae_interfaces/srv/MoveJByPath`

用展平数组表达 2–100 个七关节路点。单臂使用 `joint_positions`，双臂使用左右
对应数组；角度单位 rad，左右路径点数必须一致。

```bash
ros2 interface show rokae_interfaces/srv/MoveJByPath
```

[查看路径字段与调用示例](/api/moveabsj#_7-6-movej-by-path-service)

<a id="rokae-interfaces-srv-movel"></a>

## `rokae_interfaces/srv/MoveL`

绝对 TCP 目标服务。请求包含位置、完整 RPY 姿态、可选七轴臂角、速度和过渡半径；
响应为 `success/message`。

```bash
ros2 interface show rokae_interfaces/srv/MoveL
```

[查看 MoveL 调用示例](/api/movel#_8-1-绝对-movel)

<a id="rokae-interfaces-srv-movelrelative"></a>

## `rokae_interfaces/srv/MoveLRelative`

相对 TCP 平移服务。`translation` 单位 m；姿态通过 `orientation_override` 选择是否
覆盖对应的绝对 RPY 轴。

[查看相对 MoveL 示例](/api/movel#_8-2-相对-movel)

<a id="rokae-interfaces-srv-getcartesianstate"></a>

## `rokae_interfaces/srv/GetCartesianState`

空请求。成功响应中的 `pose` 为 `[x,y,z,rx,ry,rz]`，单位分别为 m 和 rad。

```bash
ros2 interface show rokae_interfaces/srv/GetCartesianState
```

[查看笛卡尔状态读取](/api/movel#_8-3-读取笛卡尔状态)

<a id="rokae-interfaces-srv-cartesianteach"></a>

## `rokae_interfaces/srv/CartesianTeach`

启动单臂笛卡尔拖动示教。请求只包含 `enable_drag_button`；默认为
`false`，拖动时必须按住末端按键。响应为 `success/message`。

```bash
ros2 interface show rokae_interfaces/srv/CartesianTeach
```

[查看拖动示教调用与安全说明](/api/cartesian-teach)

<a id="rokae-interfaces-srv-controlhand"></a>

## `rokae_interfaces/srv/ControlHand`

请求包含 `command` 和六电机 `values`；响应包含执行状态、说明和原始 CAN 回复。
支持 `open`、`half`、`close`、`position`、`motors`、`speed`、`pressure`。

```bash
ros2 interface show rokae_interfaces/srv/ControlHand
```

[查看灵巧手命令示例](/api/hand)

<a id="rokae-interfaces-srv-forwardkinematics"></a>

## `rokae_interfaces/srv/ForwardKinematics`

请求七个关节角，返回 TCP `Pose`、臂角和关节构型。该服务只计算，不发送运动。

```bash
ros2 service call /aide/upperlimb/fk/left_arm \
  rokae_interfaces/srv/ForwardKinematics \
  "{joints: [J1, J2, J3, J4, J5, J6, J7]}"
```

[查看 FK 字段说明](/api/kinematics#_18-1-forwardkinematics)

<a id="rokae-interfaces-srv-inversekinematics"></a>

## `rokae_interfaces/srv/InverseKinematics`

请求 TCP `Pose` 及可选臂角约束，返回七关节解、FK 回算位姿与误差。该服务只计算，
不发送运动。

[查看 IK 字段与联合测试](/api/kinematics#_18-2-inversekinematics)
