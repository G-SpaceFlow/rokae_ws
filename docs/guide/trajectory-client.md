---
title: 🐍 Python 轨迹配置
outline: [2, 4]
---

# 🐍 Python 轨迹配置

[查看完整来源文档](/JOINT_TRAJECTORY_UPDATE)

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
