# 关节轨迹与上电接口更新说明

更新日期：2026-09-07。

参考来源：[浙江人形机器人 ROS SDK — MoveJByPath](https://zj-humanoid.github.io/zj_humanoid_sdk_ros/zj_humanoid_types#movejbypath)。
本文仅借鉴字段设计，不表示与该 SDK 二进制兼容。原文 MoveJByPath 的
is_async 注释只写“是否同步运行”；下文明确规定本项目的布尔语义。
时间戳首项 >0、路点数量及双臂排列也是本项目约定，不是对参考文档的逐字复述。

本文记录本地开发工作区的接口设计和实现进度。本次文档提交不包含对应
功能代码；仅拉取本次文档不能保证目标机器已安装下述新增接口。

## 1. 实现状态

| 功能 | 本地状态 |
| --- | --- |
| Python 定时 ServoJ 轨迹 | 已实现，含启动、反馈、到位检查 |
| 单臂/双臂初始化上电服务 | 已实现并编译，未在本次修改中执行硬件测试 |
| `Joints.msg`、`JointTrajectory.srv` | 已定义并编译 |
| 新轨迹请求构造与参数校验 | 已实现，4 项离线测试通过 |
| 新 `JointTrajectory` 运动服务端 | **尚未实现** |
| 新接口逐点时间戳、同步/异步执行 | **仅定义契约，尚未接入执行端** |

原 `MoveJByPath` 服务及 `movej_by_path_client.py` 执行路径保持兼容。
不要把新服务类型生成成功理解为运动服务已经上线。

## 2. 统一轨迹接口契约

类型：`rokae_interfaces/srv/JointTrajectory`。服务名称尚未绑定。

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

## 3. 初始化上电服务

下列接口均为 `std_srvs/srv/Trigger`，请求为空，响应为 success/message。

| 服务名称 | 范围 |
| --- | --- |
| `/aide/upperlimb/initialize/left_arm` | 左臂 |
| `/aide/upperlimb/initialize/right_arm` | 右臂 |
| `/aide/upperlimb/initialize/dual_arm` | 双臂 |
| `/aide/upperlimb/initialize` | 原双臂兼容接口 |

对应 SDK 示例 `op_single.cpp` 和 `op.cpp` 的顺序：读取机器人信息 →
NrtCommand → automatic → setPowerState(true) → powerState 验证。
服务额外检查控制占用、空闲及电源安全状态，复用驱动连接，不启动独立 SDK 进程。
不发送运动指令，不清除故障，不执行下电。

双臂初始化先取得两臂控制锁。实际初始化时一臂失败仍尝试另一臂，
响应分别报告结果，不自动下电回滚已成功的机械臂。

以下命令会真实上电。确认现场安全后，仅调用需要的模式：

```bash
ros2 service call /aide/upperlimb/initialize/left_arm std_srvs/srv/Trigger '{}'
```

右臂或双臂分别替换末尾为 `right_arm` 或 `dual_arm`。
已实现这些服务的驱动需要重新编译、重启后才可用；不要并行启动多份驱动。

另有 `/aide/upperlimb/power_on/left_arm` 和 `right_arm`：已上电时直接返回，
不重新设置模式。它们与完整 initialize 顺序不同。

## 4. 现有 Python 轨迹配置

文件：`src/test/movej_by_path_client.py`。

| 配置 | 作用 |
| --- | --- |
| `ARM_SELECTION` | 1 左臂、2 右臂、3 双臂 |
| `EXECUTE` | 是否真实执行 |
| `POWER_ON_BEFORE_MOTION` | 是否在运动前显式请求所选臂上电 |
| `TOTAL_MOTION_TIME_S` | 正数使用定时 ServoJ；None 使用原非实时服务 |
| `CORNER_BLEND_RAD` | 拐角圆滑偏差，0 恢复逐点停顿 |

上电开关开启时使用 power_on 服务；失败或响应超时不执行轨迹。
程序不在轨迹结束或异常后自动补上电。

运行前仍需加载 ROS 和已编译工作区环境：

```bash
source /opt/ros/humble/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
```

## 5. 已知问题与验证边界

- 轨迹退出时一次 powerState=on 不能保证之后持续上电。
- 已观察到退出检查通过、下一次运行却未上电的情况；根因尚未确认，
  不能断言切换控制模式必然导致下电。
- 编译和离线测试不替代真实机器人安全验证。
- 曾出现多个进程处于内核 D 状态、等待 rtnl_lock/netlink_dump_start；
  这不等于内存耗尽。旧进程未退出时不要重复启动驱动。

原接口总览见 [ROS 2 接口参考](./ROS2_INTERFACE_REFERENCE.md)。
