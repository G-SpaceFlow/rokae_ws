---
title: 🗂️ 实现位置
outline: [2, 4]
---

# 🗂️ 实现位置

[查看完整来源文档](/ROS2_INTERFACE_REFERENCE)

| 内容 | 源文件 |
| --- | --- |
| 共享 SDK 对象与互斥 | `src/rokae_driver/include/rokae_driver/shared_arm_hardware.hpp` |
| 统一进程入口 | `src/rokae_driver/src/ros_dual_arm_driver.cpp` |
| 接口命名常量 | `src/rokae_driver/include/rokae_driver/interface_names.hpp` |
| MoveAbsJ | `src/rokae_driver/src/ros_moveabsj_action_server.cpp` |
| MoveJ By Path | `src/rokae_driver/src/ros_movej_by_path_service.cpp` |
| MoveL 与笛卡尔状态 | `src/rokae_driver/src/ros_movel_service.cpp` |
| ServoJ 实时关节控制 | `src/rokae_driver/src/ros_servoj_subscriber.cpp` |
| ServoL 实时笛卡尔控制 | `src/rokae_driver/src/ros_servol_subscriber.cpp` |
| FK/IK 运动学服务 | `src/rokae_driver/src/ros_kinematics_service.cpp` |
| 单臂与双臂回原 | `src/rokae_driver/src/ros_go_home_service.cpp` |
| 状态与 Jacobian | `src/rokae_driver/src/ros_pos_publisher.cpp` |
| 初始化 | `src/rokae_driver/src/ros_robot_initializer_service.cpp` |
| 灵巧手 | `src/rokae_driver/src/ros_hand_service.cpp` |
| 自定义接口定义 | `src/rokae_interfaces/msg/`、`src/rokae_interfaces/srv/` |
| 部署参数 | `src/rokae_bringup/config/dual_arm.yaml` |
| Launch | `src/rokae_bringup/launch/dual_arm.launch.py` |

接口字段或行为发生变化时，应同时更新本文件、对应 `.msg`/`.srv` 注释和部署参数。
