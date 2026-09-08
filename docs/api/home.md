---
title: 🏠 回原与兼容初始化
outline: [2, 4]
---

# 🏠 回原与兼容初始化

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 9.1 初始化

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

## 9.2 回原

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
