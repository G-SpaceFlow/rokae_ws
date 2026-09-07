---
title: 🖐️ 灵巧手
outline: [2, 4]
---

# 🖐️ 灵巧手

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

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
