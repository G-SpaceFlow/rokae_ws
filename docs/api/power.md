---
title: 🔌 初始化上电
outline: [2, 4]
---

# 🔌 初始化上电

[查看完整来源文档](/JOINT_TRAJECTORY_UPDATE)

## Initialize

| 属性 | 说明 |
| --- | --- |
| 接口类型 | `std_srvs/srv/Trigger` |
| 请求 | 空请求 `{}` |
| 返回 | `bool success`、`string message` |
| 本地状态 | 已实现并编译；本次未执行硬件测试 |
| 行为 | 完整初始化并上电；不发送运动指令 |

### 服务地址

| 服务名称 | 范围 |
| --- | --- |
| `/aide/upperlimb/initialize/left_arm` | 左臂 |
| `/aide/upperlimb/initialize/right_arm` | 右臂 |
| `/aide/upperlimb/initialize/dual_arm` | 双臂 |
| `/aide/upperlimb/initialize` | 原双臂兼容接口 |

### 执行顺序

对应 SDK 示例 `op_single.cpp` 和 `op.cpp`：

```text
控制占用及安全检查
  → 读取机器人信息
  → NrtCommand（非实时模式）
  → automatic（自动模式）
  → setPowerState(true)
  → powerState 验证并返回
```

服务额外检查控制占用、空闲及电源安全状态，复用驱动连接，不启动独立 SDK 进程。
不发送运动指令，不清除故障，不执行下电。

双臂初始化先取得两臂控制锁。实际初始化时一臂失败仍尝试另一臂，
响应分别报告结果，不自动下电回滚已成功的机械臂。

### 调用示例

> **以下命令会真实上电。** 确认现场安全后，仅调用需要的模式。

```bash
ros2 service call /aide/upperlimb/initialize/left_arm std_srvs/srv/Trigger '{}'
```

右臂或双臂分别替换末尾为 `right_arm` 或 `dual_arm`。
已实现这些服务的驱动需要重新编译、重启后才可用；不要并行启动多份驱动。

<a id="power-on"></a>

## PowerOn

| 属性 | 说明 |
| --- | --- |
| 接口类型 | `std_srvs/srv/Trigger` |
| 左臂地址 | `/aide/upperlimb/power_on/left_arm` |
| 右臂地址 | `/aide/upperlimb/power_on/right_arm` |
| 已上电时 | 直接返回，不重新设置模式 |
| 与 Initialize 的区别 | Initialize 执行完整初始化顺序，即使已经上电 |
| 使用位置 | Python 的 `POWER_ON_BEFORE_MOTION` 分支 |
