---
title: 🚙 底盘桥接
outline: [2, 4]
---

# 🚙 底盘桥接

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

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
