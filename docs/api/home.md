---
title: 🏠 回原与兼容初始化
outline: [2, 4]
---

# 🏠 回原与兼容初始化

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 9.1 初始化

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

## 9.2 回原

```text
/left_arm/go_home
/right_arm/go_home
/dual_arm/go_home
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
ros2 service call /left_arm/go_home std_srvs/srv/Trigger
ros2 service call /right_arm/go_home std_srvs/srv/Trigger
ros2 service call /dual_arm/go_home std_srvs/srv/Trigger
```

`Trigger` 是固定的空请求类型，不接收关节角；服务内部直接使用上面的原点。
命令中的 `std_srvs/srv/Trigger` 是 ROS 2 通用 CLI 要求填写的接口类型，不是
运动参数。调用会产生真实运动。驱动不会自动上电，调用前应完成初始化并确认
工作空间、负载和急停条件。
