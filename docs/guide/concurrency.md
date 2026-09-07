---
title: 🔒 控制权与并发
outline: [2, 4]
---

# 🔒 控制权与并发

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

统一驱动进程只创建两个 `ArRobot`：左、右臂各一个。每只手臂有两类内部锁：

```text
commandMutex：控制任务级独占
sdkMutex：单次 SDK 函数调用串行化
```

`MoveAbsJ`、所有 `MoveL`、ServoJ、回原和初始化使用 `commandMutex`。同一只
手臂一次只允许一个控制任务，左右臂可以并行；双臂 ServoJ 和双臂回原同时
取得两侧锁。状态发布和灵巧手只短暂使用 `sdkMutex`。

内部控制锁目前没有发布为 ROS topic，SDK 的 `operationState()` 也没有独立状态
topic。因此：

- `operationState=moving` 表示控制器在运动，不等价于 ROS 控制权归属。
- `commandMutex` 能阻止统一进程内的命令冲突，但无法阻止外部程序直接创建
  `ArRobot` 并绕过驱动。
- 不应在统一驱动运行时并行启动 `cxl/servol.cpp` 等直接 SDK 控制程序。

后续若增加 `control_state`，应同时表达 `occupied`、`owner`、SDK operation
state、实际运动状态、控制模式和错误，而不是只发布一个 `moving` 布尔值。
