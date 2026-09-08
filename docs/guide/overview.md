---
title: 📘 概述
outline: [2, 4]
---

# 📘 概述

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

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
  ├── FK / IK Services
  ├── MoveAbsJ Action
  ├── MoveL Services
  ├── ServoJ Realtime Topics
  ├── ServoL Realtime Topics
  ├── State Topics
  ├── Initialization Service
  └── Linker Hand Services
          ↓ xCoreSDK
左臂 ArRobot                 右臂 ArRobot
```

当前驱动同时提供非实时 Move、FK/IK 运动学计算、ServoJ 实时关节位置流和
ServoL 实时笛卡尔位姿流接口。阻抗和力矩控制尚未作为 ROS 接口发布，参见
[未实现接口](/ROS2_INTERFACE_REFERENCE#_13-未实现接口)。
