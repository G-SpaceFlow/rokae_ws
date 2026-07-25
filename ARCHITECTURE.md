# rokae_ws 架构与 xCoreSDK 引用关系

## 1. 工作空间边界

当前机器人电脑中，`cxl` 与 ROS 2 工作空间同级，厂家 SDK 位于 `cxl`
内部：

```text
/home/niic/
|-- cxl/
|   `-- xCoreSDK-v0.7.1.ar_6/  # 厂家头文件、Eigen、动态库和示例
`-- rokae_ws/                  # 自己维护的 ROS 2 工程
```

`rokae_ws` 不复制厂家 SDK 的 `include`、`external` 和 `lib`。这样升级 SDK
时只需要更换 SDK 目录并重新编译，不需要把厂家文件混入 ROS 源码。

## 2. 工作空间目录

```text
rokae_ws/
|-- src/
|   |-- rokae_driver/       # C++，连接机器人并把 SDK 转换成 ROS 接口
|   |   |-- CMakeLists.txt
|   |   |-- package.xml
|   |   `-- src/
|   |       |-- ros_pos_publisher.cpp
|   |       `-- ros_moveabsj_action_server.cpp
|   |
|   |-- rokae_motion/       # Python，调用 Action 并编排动作
|   |   |-- package.xml
|   |   |-- setup.py
|   |   |-- setup.cfg
|   |   |-- config/
|   |   |   `-- dual_moveabsj_program.json
|   |   |-- resource/
|   |   |   `-- rokae_motion
|   |   `-- rokae_motion/
|   |       |-- __init__.py
|   |       |-- ros_dual_moveabsj_client.py
|   |       `-- run_moveabsj_program.py
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

## 3. 三个包的责任

### rokae_driver

这是硬件适配层，直接创建 `rokae::ArRobot` 并调用 xCoreSDK：

- `ros_pos_publisher` 调用 `jointPos()` 和 `posture()`，发布
  `/left_arm|right_arm/joint_states` 和 `tcp_pose`。
- `ros_moveabsj_action_server` 接收
  `/left_arm|right_arm/move_absj` Action，检查目标和限位，然后调用
  `MoveAbsJCommand`、`moveAppend()` 和 `moveStart()`。

上层程序不应绕过这个包同时直接控制同一机械臂。

### rokae_motion

这是动作编排层，不直接包含 xCoreSDK 头文件，也不链接 SDK 动态库。它通过
ROS 2 Action 调用 `rokae_driver`：

```text
dual_arm_program
        |
        | FollowJointTrajectory Action
        v
ros_moveabsj_action_server
```

### rokae_bringup

这是部署层：

- `config/dual_arm.yaml` 保存 IP、速度比例、软限位余量和话题坐标系。
- `launch/dual_arm.launch.py` 启动 driver 中已经安装的可执行程序。

Launch 不编译源码，也不直接查找 `.cpp` 文件。

## 4. CMake 如何找到外部 SDK

`rokae_driver/CMakeLists.txt` 先确定 SDK 根目录：

```bash
export ROKAE_SDK_ROOT=/home/niic/cxl/xCoreSDK-v0.7.1.ar_6
```

如果没有设置环境变量，当前工程默认从
`/home/niic/rokae_ws/src/rokae_driver`回到 `/home/niic`，再进入
`cxl/xCoreSDK-v0.7.1.ar_6`。

CMake随后检查并使用：

```text
$ROKAE_SDK_ROOT/include/rokae/robot.h
$ROKAE_SDK_ROOT/external/
$ROKAE_SDK_ROOT/lib/**/libxCoreSDK.so
```

它建立了一个导入目标：

```cmake
add_library(Rokae::xCoreSDK UNKNOWN IMPORTED)
```

这个目标同时携带：

- `IMPORTED_LOCATION`：`libxCoreSDK.so` 的实际路径。
- `INTERFACE_INCLUDE_DIRECTORIES`：SDK头文件和Eigen路径。

两个C++节点再通过：

```cmake
target_link_libraries(<node> PRIVATE Rokae::xCoreSDK)
```

获得SDK依赖。

这里并不是运行SDK目录下的 `moveabsj`、`pos` 等示例程序，而是让ROS节点
直接调用 `libxCoreSDK.so` 中实现的函数。

## 5. 编译阶段

```text
colcon build
    |
    |-- 读取每个 package.xml，识别 ament_cmake 或 ament_python
    |
    |-- rokae_driver/CMakeLists.txt
    |     |-- 编译 src/*.cpp
    |     |-- 读取 SDK include/external
    |     `-- 链接 libxCoreSDK.so
    |
    |-- rokae_motion/setup.py
    |     `-- 安装 Python 模块和 console_scripts
    |
    `-- rokae_bringup/CMakeLists.txt
          `-- 安装 launch 和 config
```

安装后的重要位置：

```text
install/rokae_driver/lib/rokae_driver/ros_pos_publisher
install/rokae_driver/lib/rokae_driver/ros_moveabsj_action_server
install/rokae_motion/lib/rokae_motion/moveabsj_client
install/rokae_motion/lib/rokae_motion/dual_arm_program
install/rokae_bringup/share/rokae_bringup/launch/
```

## 6. 运行阶段

状态发布链路：

```text
机器人控制器
  -> 以太网
  -> libxCoreSDK.so
  -> ArRobot::jointPos()/posture()
  -> ros_pos_publisher
  -> ROS 2 JointState/PoseStamped 话题
```

MoveAbsJ动作链路：

```text
rokae_motion Python程序
  -> ROS 2 FollowJointTrajectory Action
  -> ros_moveabsj_action_server
  -> SDK MoveAbsJCommand
  -> moveAppend()/moveStart()
  -> 以太网
  -> 机器人控制器
  -> 关节驱动和电机
```

运行时不会读取 `.cpp`，而是执行 `install/` 中的程序；动态加载器再装载
`libxCoreSDK.so`。

## 7. 构建与启动

```bash
cd /home/niic/rokae_ws
source /opt/ros/humble/setup.bash
export ROKAE_SDK_ROOT=/home/niic/cxl/xCoreSDK-v0.7.1.ar_6

colcon build --symlink-install
source install/setup.bash

ros2 launch rokae_bringup dual_arm.launch.py
```

该 Launch 会连接两台控制器，但不会给机器人上电，也不会发送动作目标。
首次发送目标必须使用单臂、小关节变化量、低速、空旷环境，并保持急停可触及。
