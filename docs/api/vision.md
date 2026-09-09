---
title: 📷 视觉目标
outline: [2, 4]
---

# 📷 视觉目标

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

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
缓存目标，不直接调用 SDK；运动统一通过 `move_l` 驱动接口执行。

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
