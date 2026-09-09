---
title: 📦 MoveL 服务
outline: [2, 4]
---

# 📦 MoveL 服务

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

所有 MoveL 服务是阻塞服务：调用在完成、失败或超时后返回。它们使用 SDK
`NrtCommand + MoveLCommand`，不是实时 ServoL。

## 8.1 绝对 MoveL

```text
/aide/upperlimb/move_l/left_arm
/aide/upperlimb/move_l/right_arm
Type: rokae_interfaces/srv/MoveL
```

请求：

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `position` | `float64[3]` | m | 外部参考系中的绝对 TCP XYZ |
| `orientation_rpy` | `float64[3]` | rad | 必填的绝对 XYZ Euler RPY 姿态 |
| `use_elbow` | `bool` | - | 是否应用请求中的七轴臂角 |
| `elbow` | `float64` | rad | 七轴臂角；`use_elbow=false` 时忽略 |
| `speed_mm_s` | `float64` | mm/s | TCP 线速度 |
| `zone_mm` | `float64` | mm | 过渡半径，0 表示精确停点 |

响应：`success` 和可诊断的 `message`。驱动复制当前完整 SDK
`CartesianPosition`，再设置请求中的绝对位置和完整姿态；默认保持臂角和构型。

```bash
ros2 service call /aide/upperlimb/move_l/right_arm rokae_interfaces/srv/MoveL \
  "{position: [0.280941, -0.314533, -0.489353], orientation_rpy: [-2.761879, 0.340152, -0.746153], use_elbow: false, elbow: 0.0, speed_mm_s: 50.0, zone_mm: 0.0}"
```

## 8.2 相对 MoveL

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

## 8.3 读取笛卡尔状态

```text
/aide/upperlimb/get_cartesian_state/left_arm
/aide/upperlimb/get_cartesian_state/right_arm
Type: rokae_interfaces/srv/GetCartesianState
```

请求为空。成功响应中的 `pose` 是 `[x,y,z,rx,ry,rz]`，单位分别为 m 和 rad。
为了返回同一控制阶段的一致数据，该服务也会尝试取得该手臂的控制锁；手臂被
其他运动命令占用时返回失败。持续监控请使用 `tcp_pose` topic。
