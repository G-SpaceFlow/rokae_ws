# rokae_ws 架构与 xCoreSDK 引用关系

面向接口使用者的完整参考文档见
[`docs/ROS2_INTERFACE_REFERENCE.md`](docs/ROS2_INTERFACE_REFERENCE.md)。

## 1. 工作空间边界

当前机器人电脑中，厂家 SDK 位于统一的硬件目录：

```text
/home/niic/
|-- aide/hardware/arm/
|   `-- xCoreSDK-v0.7.1.ar_6/  # 厂家头文件、Eigen、库和 cxl 示例
`-- rokae_ws/                  # 自己维护的 ROS 2 工程
```

`rokae_ws` 不复制厂家 SDK 的 `include`、`external` 和 `lib`。这样升级 SDK
时只需要更换 SDK 目录并重新编译，不需要把厂家文件混入 ROS 源码。

## 2. 工作空间目录

```text
rokae_ws/
|-- src/
|   |-- rokae_interfaces/   # 自定义消息与服务定义
|   |
|   |-- rokae_driver/       # C++，连接机器人并把 SDK 转换成 ROS 接口
|   |   |-- CMakeLists.txt
|   |   |-- package.xml
|   |   |-- include/rokae_driver/
|   |   |   |-- node_factories.hpp
|   |   |   `-- shared_arm_hardware.hpp
|   |   `-- src/
|   |       |-- ros_dual_arm_driver.cpp
|   |       |-- ros_pos_publisher.cpp
|   |       |-- ros_moveabsj_action_server.cpp
|   |       |-- ros_movel_service.cpp
|   |       |-- ros_servoj_subscriber.cpp
|   |       |-- ros_go_home_service.cpp
|   |       |-- ros_hand_service.cpp
|   |       `-- ros_robot_initializer_service.cpp
|   |
|   |-- rokae_motion/       # Python，调用 Action 并编排动作
|   |   |-- package.xml
|   |   |-- setup.py
|   |   |-- setup.cfg
|   |   |-- behavior_trees/
|   |   |   `-- examples/
|   |   |-- resource/
|   |   |   `-- rokae_motion
|   |   `-- rokae_motion/
|   |       |-- __init__.py
|   |       |-- moveabsj_action_client.py
|   |       |-- bt_executor.py
|   |       |-- executor_services.py
|   |       |-- vision_motion.py
|   |       `-- vision_target_server.py
|   |
|   `-- rokae_bringup/      # Launch 和现场参数
|       |-- CMakeLists.txt
|       |-- package.xml
|       |-- config/
|       |   `-- dual_arm.yaml
|       `-- launch/
|           `-- dual_arm.launch.py
|
|-- build/                   # colcon 生成，不提交
|-- install/                 # colcon 生成，ros2 run/launch 从这里找包
`-- log/                     # colcon 生成
```

## 3. Rokae 核心包的责任

### rokae_interfaces

这是接口定义层，保存 MoveL、灵巧手、视觉目标等自定义 ROS 2 服务定义。
驱动与动作编排包都依赖它，接口变更后必须重新构建工作空间。

### rokae_driver

这是硬件适配层，直接创建 `rokae::ArRobot` 并调用 xCoreSDK：

- `ros_dual_arm_driver` 是唯一安装和启动的底层驱动进程。进程内只有左右臂
  各一个共享 `ArRobot` 对象；下列 ROS 节点接口都由该进程内部创建并复用这
  两个连接。
- `SharedArmHardware` 分别为左右臂提供 SDK 调用锁和运动控制锁。SDK 调用锁
  保护同一个 `ArRobot` 对象，运动控制锁防止 MoveAbsJ、MoveL 和初始化同时
  控制同一机械臂；ServoJ 也使用同一把锁，左右臂仍可独立工作。

- `ros_pos_publisher` 调用 `jointPos()`、`posture()` 和模型库
  `jacobian()`，发布 `/left_arm|right_arm/joint_states`、`tcp_pose` 和
  `jacobian`。雅可比计算默认开启，但没有订阅者时跳过计算。
- `ros_moveabsj_action_server` 接收
  `/left_arm|right_arm/move_absj` Action，检查目标和限位，然后调用
  `MoveAbsJCommand`、`moveAppend()` 和 `moveStart()`。
- `ros_movel_service` 提供左右臂绝对与相对 MoveL 服务。
- `ros_servoj_subscriber` 提供左臂、右臂及双臂 ServoJ Topic。ROS 回调更新
  最新关节目标，专用线程按固定周期调用 SDK 实时接口；断流时自动停止。
- `ros_go_home_service` 提供左臂、右臂和双臂回原服务，以配置中的固定关节
  原点执行受监控的 MoveAbsJ；双臂模式同时占用两侧控制锁。
- `ros_hand_service` 提供左右灵巧手末端 CAN 控制服务。
- `ros_robot_initializer_service` 提供 `/initialize_robots`，严格按
  `op.cpp` 的顺序切换非实时指令模式、自动模式并给双臂上电。启动节点本身
  不会上电，只有收到服务请求才执行初始化。

上层程序不应绕过这个包同时直接控制同一机械臂。

### rokae_motion

这是动作编排层，不直接包含 xCoreSDK 头文件，也不链接 SDK 动态库。它通过
ROS 2 Action 调用 `rokae_driver`：

```text
bt_runner / bt_control
        |
        | FollowJointTrajectory Action
        v
ros_moveabsj_action_server
```

视觉部分订阅现有 YOLO JSON 话题，并在内存中缓存命名点与标量值：

```text
/yolo_vision/* JSON
  -> vision_target_server
  -> /bt_target_server/trigger_detect|get_target|set_offset
  -> move_l_vision motion_mode 1..6
  -> /left_arm|right_arm/move_l_target
```

`move_l_target` 在 C++ 驱动中复制控制器当前完整笛卡尔构型，再覆盖视觉计算
出的绝对位置和指定 RPY 轴，因此不需要为视觉目标伪造 elbow。全身 IK、腰部
运动和腰部坐标旋转不在当前 Rokae 范围内。

### rokae_bringup

这是部署层：

- `config/dual_arm.yaml` 保存 IP、速度比例、软限位余量和话题坐标系。
- `launch/dual_arm.launch.py` 启动 driver 中已经安装的可执行程序。

Launch 不编译源码，也不直接查找 `.cpp` 文件。

## 4. CMake 如何找到外部 SDK

`rokae_driver/CMakeLists.txt` 先确定 SDK 根目录：

```bash
export ROKAE_SDK_ROOT=/home/niic/aide/hardware/arm/xCoreSDK-v0.7.1.ar_6
```

如果没有设置环境变量，当前工程默认从
`/home/niic/rokae_ws/src/rokae_driver`回到 `/home/niic`，再进入
`aide/hardware/arm/xCoreSDK-v0.7.1.ar_6`。

CMake随后检查并使用：

```text
$ROKAE_SDK_ROOT/include/rokae/robot.h
$ROKAE_SDK_ROOT/external/
$ROKAE_SDK_ROOT/lib/Linux/<architecture>/libxCoreSDK.so 或 libxCoreSDK.a
```

它建立了一个导入目标：

```cmake
add_library(Rokae::xCoreSDK UNKNOWN IMPORTED)
```

这个目标同时携带：

- `IMPORTED_LOCATION`：选中的 `libxCoreSDK.so` 或 `libxCoreSDK.a` 路径。
- `INTERFACE_INCLUDE_DIRECTORIES`：SDK头文件和Eigen路径。

统一 C++ 驱动目标再通过：

```cmake
target_link_libraries(<node> PRIVATE Rokae::xCoreSDK)
```

获得SDK依赖。

这里并不是运行SDK目录下的 `moveabsj`、`pos` 等示例程序，而是让ROS节点
直接调用选中的 xCoreSDK 库中实现的函数。当前 SDK 会选用 ARM64 静态库
`libxCoreSDK.a`。

## 5. 编译阶段

```text
colcon build
    |
    |-- 读取每个 package.xml，识别 ament_cmake 或 ament_python
    |
    |-- rokae_driver/CMakeLists.txt
    |     |-- 编译 src/*.cpp
    |     |-- 读取 SDK include/external
    |     `-- 链接匹配目标架构的 xCoreSDK 库
    |
    |-- rokae_motion/setup.py
    |     `-- 安装 Python 模块和 console_scripts
    |
    `-- rokae_bringup/CMakeLists.txt
          `-- 安装 launch 和 config
```

安装后的重要位置：

```text
install/rokae_driver/lib/rokae_driver/ros_dual_arm_driver
install/rokae_motion/lib/rokae_motion/bt_runner
install/rokae_motion/lib/rokae_motion/bt_control
install/rokae_motion/lib/rokae_motion/vision_target_server
install/rokae_bringup/share/rokae_bringup/launch/
```

## 6. 运行阶段

状态发布链路：

```text
机器人控制器
  -> 以太网
  -> xCoreSDK 库
  -> ArRobot::jointPos()/posture()/xMateModel::jacobian()
  -> ros_dual_arm_driver 内部状态发布节点
  -> ROS 2 JointState/PoseStamped/Float64MultiArray 话题
```

MoveAbsJ动作链路：

```text
rokae_motion Python程序
  -> ROS 2 FollowJointTrajectory Action
  -> ros_dual_arm_driver 内部 MoveAbsJ Action 节点
  -> SDK MoveAbsJCommand
  -> moveAppend()/moveStart()
  -> 以太网
  -> 机器人控制器
  -> 关节驱动和电机
```

`bt_control` 初始化链路：

```text
bt_control
  -> /initialize_robots (std_srvs/Trigger)
  -> ros_dual_arm_driver 内部初始化服务节点
  -> NrtCommand -> automatic -> power on -> verify
  -> 初始化成功后才开始执行行为树动作
```

`bt_control --dry-run` 和通用的 `bt_runner` 不调用该初始化服务。

运行时不会读取 `.cpp`，而是执行 `install/` 中的程序。当前 ARM64 构建将
xCoreSDK 静态链接进驱动程序；如果以后选择共享库，则由动态加载器装载。

## 7. 构建与启动

```bash
cd /home/niic/rokae_ws
source /opt/ros/humble/setup.bash
source /home/niic/dbstest_ws/install/setup.bash
export ROKAE_SDK_ROOT=/home/niic/aide/hardware/arm/xCoreSDK-v0.7.1.ar_6

colcon build --symlink-install
source install/local_setup.bash

ros2 launch rokae_bringup dual_arm.launch.py
```

工作区在没有显式设置 `RMW_IMPLEMENTATION` 时默认使用
`rmw_cyclonedds_cpp`，以避开当前机器人电脑上 Fast DDS 创建完整客户端集合时
出现的卡死。部署网络参数与 SDK 的 `cxl/op.cpp` 一致：左臂控制器/本机地址为
`192.168.4.160/192.168.4.10`，右臂为
`192.168.2.160/192.168.2.10`。启动驱动前，两张对应网卡都必须处于 UP 状态。

该 Launch 会连接两台控制器，但启动本身不会给机器人上电，也不会发送动作
目标。真实运行 `bt_control` 时，它会在首个动作前请求初始化并给双臂上电。
首次发送目标必须使用单臂、小关节变化量、低速、空旷环境，并保持急停可触及。
