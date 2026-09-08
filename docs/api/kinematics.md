---
title: 🧮 FK / IK 运动学计算
outline: [2, 4]
---

# 🧮 FK / IK 运动学计算

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

运动学服务分别提供左臂和右臂接口：

```text
/aide/upperlimb/fk/left_arm
/aide/upperlimb/fk/right_arm
/aide/upperlimb/ik/left_arm
/aide/upperlimb/ik/right_arm
```

服务使用 xCoreSDK `xMateModel<7>` 和控制器当前 `toolset`。位姿是 TCP 相对当前
外部参考坐标系的结果。服务只计算，不自动上电，也不发送运动指令；但需要驱动
已连接对应机器人，以读取机型模型和活动工具/工件坐标配置。

## 18.1 ForwardKinematics

类型：`rokae_interfaces/srv/ForwardKinematics`。

| 方向 | 字段 | 类型 | 单位/说明 |
| --- | --- | --- | --- |
| 请求 | `joints` | `float64[7]` | J1…J7，rad |
| 响应 | `success` | `bool` | SDK 计算是否成功 |
| 响应 | `message` | `string` | 结果或错误原因 |
| 响应 | `pose` | `geometry_msgs/Pose` | TCP 位姿，位置 m、姿态四元数 |
| 响应 | `elbow` | `float64` | 七轴臂角，rad |
| 响应 | `has_elbow` | `bool` | 臂角是否有效 |
| 响应 | `configuration` | `int32[]` | SDK 关节构型数据 |

```bash
ros2 service call /aide/upperlimb/fk/left_arm \
  rokae_interfaces/srv/ForwardKinematics \
  "{joints: [J1, J2, J3, J4, J5, J6, J7]}"
```

## 18.2 InverseKinematics

类型：`rokae_interfaces/srv/InverseKinematics`。

| 方向 | 字段 | 类型 | 单位/说明 |
| --- | --- | --- | --- |
| 请求 | `pose` | `geometry_msgs/Pose` | TCP 目标，外部参考系 |
| 请求 | `use_elbow` | `bool` | 是否使用指定臂角约束七轴选解 |
| 请求 | `elbow` | `float64` | 指定臂角，rad |
| 响应 | `joints` | `float64[7]` | IK 结果，rad |
| 响应 | `solved_pose` | `geometry_msgs/Pose` | IK 结果经 FK 回算的位姿 |
| 响应 | `position_error_m` | `float64` | FK 回算位置误差，m |
| 响应 | `orientation_error_rad` | `float64` | FK 回算姿态误差，rad |
| 响应 | `success/message` | `bool/string` | 校验结果或错误原因 |

IK 返回前会用同一个模型和 `toolset` 执行 FK 回算；误差超过 11.10 节配置的
阈值时返回失败。七轴机械臂通常存在多组逆解，因此输出关节角不保证与生成目标
位姿时使用的原始关节角完全相同。

## 18.3 Python 联合测试

测试程序先执行 FK，再把输出位姿传给 IK，最后对 IK 结果再次执行 FK：

```bash
source /opt/ros/humble/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
python3 /home/niic/rokae_ws/src/test/test_fk_ik.py
```

程序顶部使用 `ARM_TYPE=1` 选择左臂，`ARM_TYPE=2` 选择右臂。测试不会上电或
控制机械臂运动。
