---
title: 🦾 关节轨迹接口契约
outline: [2, 4]
---

# 🦾 关节轨迹接口契约

[查看完整来源文档](/JOINT_TRAJECTORY_UPDATE)

> **运动服务端尚未实现。** 以下为接口契约，不代表可以直接调用运动。

## 🦾 Services · 关节轨迹

### JointTrajectory

| 属性 | 说明 |
| --- | --- |
| 接口类型 | `rokae_interfaces/srv/JointTrajectory` |
| 服务名称 | 尚未绑定 |
| 实现阶段 | 类型已生成，**运动服务端待实现** |
| 支持对象 | 左臂 / 右臂 / 双臂 |
| 单位 | 关节角 `rad`；时间 `s` |
| 是否隐含上电 | 否，需显式使用上电接口 |

#### 请求参数

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `path` | `Joints[]` | 2..100 个关节角路点，每点含 `positions` |
| `time` | `float64` | 总运动时间；大于 0 时覆盖 `timestamp` |
| `timestamp` | `float64[]` | 逐点到达时间，仅 `time=0` 时使用 |
| `is_async` | `bool` | `true` 异步提交；`false` 等待结束 |
| `arm_type` | `int8` | `1` 左臂、`2` 右臂、`3` 双臂 |

#### 返回结果

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `success` | `bool` | 请求 / 命令结果，不是独立的到位证明 |
| `message` | `string` | 执行提示或失败原因 |

<details>
<summary>查看完整 .srv / .msg 定义</summary>

```text
rokae_interfaces/Joints[] path
float64 time
float64[] timestamp
bool is_async
int8 arm_type
---
bool success
string message
```

`rokae_interfaces/msg/Joints`：

```text
float64[] positions
```

</details>

<a id="arm-modes"></a>

### 控制模式

| arm_type | 模式 | 每个路点的 positions |
| --- | --- | --- |
| 1 | 左臂 | 左 J1..J7，共 7 个角度 |
| 2 | 右臂 | 右 J1..J7，共 7 个角度 |
| 3 | 双臂 | 左 J1..J7、右 J1..J7，共 14 个角度 |

所有角度单位 rad，仅支持 1、2、3，不支持颈、腰、升降等其他组合。
路径包含 2..100 个路点，每点维度必须匹配模式，角度必须为有限值。

### 时间规则

- `time` 必须有限且非负，单位秒。
- `time > 0`：指定整段轨迹到达最后一个路点的时间，完全忽略 timestamp。
- `time = 0`：timestamp 必须与 path 等长，各项有限，首项 >0，严格递增。
- t=0 对应执行前测量的当前关节位置，path 中各点是后续目标。
- 等待服务、初始化上电、ServoJ 启动准备不计入轨迹总时间。
- 执行端仍需检查速度、加速度及硬件约束；总时间不可行时应拒绝，不能仅靠请求格式校验放行。

### 同步与异步

- `is_async=true`：约定为请求被接收后返回，不等待运动结束。
- `is_async=false`：约定为等待执行结束后返回。
- `success` 表示请求/命令结果，不能独立证明机械臂实际到位。
- 异步任务的完成、失败、取消和状态查询机制需要随服务端另行实现。
