---
title: 🚀 快速开始
outline: [2, 4]
---

# 🚀 快速开始

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

## 4.1 启动完整系统

```bash
source /opt/ros/humble/setup.bash
source /home/niic/dbstest_ws/install/setup.bash
source /home/niic/rokae_ws/install/local_setup.bash
ros2 launch rokae_bringup dual_arm.launch.py
```

启动驱动不会自动上电，也不会发送运动或灵巧手命令。运动前显式调用：

```bash
ros2 service call /aide/upperlimb/initialize std_srvs/srv/Trigger "{}"
```

## 4.2 Launch 开关

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `params_file` | `rokae_bringup/config/dual_arm.yaml` | 参数文件 |
| `start_state_publisher` | `true` | 关节、TCP 和 Jacobian 发布 |
| `start_move_server` | `true` | MoveAbsJ Action |
| `start_movel_service` | `true` | MoveL 与笛卡尔状态服务 |
| `start_hand_service` | `true` | 灵巧手服务 |
| `start_cartesian_teach_service` | `true` | 左右臂笛卡尔拖动示教服务 |
| `start_initializer_service` | `true` | 双臂初始化服务 |
| `start_go_home_service` | `true` | 左、右和双臂回原服务 |
| `start_kinematics_service` | `true` | 左、右臂 FK/IK 计算服务 |
| `start_servoj` | `true` | 左、右和双臂 ServoJ 订阅接口 |
| `start_servol` | `true` | 左、右和双臂 ServoL 订阅接口 |
| `start_movej_by_path_service` | `true` | 左、右和双臂 MoveJ 路径服务 |
| `start_vision_target_server` | `true` | 视觉目标缓存服务 |
| `start_chassis_navigation` | `false` | Seer 底盘桥接 |

## 4.3 检查接口

```bash
ros2 topic list
ros2 service list
ros2 action list
ros2 node list
```
