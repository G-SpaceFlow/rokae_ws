---
title: 🤖 笛卡尔拖动示教
outline: [2, 4]
---

# 🤖 笛卡尔拖动示教

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

左右臂分别提供启动服务，并共用一个停止服务：

```text
/aide/upperlimb/cartesian_teach/left_arm
/aide/upperlimb/cartesian_teach/right_arm
Type: rokae_interfaces/srv/CartesianTeach

/aide/upperlimb/cartesian_teach/stop
Type: std_srvs/srv/Trigger
```

## 启动参数

| 字段 | 类型 | 默认值 | 说明 |
| --- | --- | ---: | --- |
| `enable_drag_button` | `bool` | `false` | `false` 时必须按住末端拖动按键；`true` 时可直接拖动 |

示教最长时间固定为 300 秒，不提供时间请求字段。`enable_drag_button=true`
会放宽物理按键保护，仅应在已完成现场风险确认时使用。

## 调用方式

```bash
# 默认模式：拖动时按住末端按键
ros2 service call /aide/upperlimb/cartesian_teach/left_arm \
  rokae_interfaces/srv/CartesianTeach "{}"

# 提前停止：在另一个终端执行
ros2 service call /aide/upperlimb/cartesian_teach/stop \
  std_srvs/srv/Trigger "{}"
```

启动服务是阻塞调用，直到停止、安全检查触发或 300 秒到期后返回。
服务会检查工具负载、相对起点位移、旋转、TCP 线速度和关节速度。

::: warning 结束后的机器人状态
服务按 SDK 拖动示教流程切换为手动模式并下电。结束时会关闭拖动，
但不会自动切回自动模式或重新上电。
:::
