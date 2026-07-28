#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger

# ===================== 全局话题常量（和调度、Seer驱动对齐） =====================
# 调度下发指令
TOPIC_CMD_CHASSIS = "/scheduler/cmd/chassis"
# 底盘状态反馈给调度
TOPIC_STATE_CHASSIS = "/chassis/state"
# Seer底盘导航下发目标站点
SEER_GO_TARGET = "/seer/go_target"
# Seer导航状态
SEER_NAV_STATUS = "/seer/navigation_status"
# Seer取消导航服务
SRV_CANCEL_NAV = "/seer/cancel_navigation"

# 调度指令
CMD_GO_A_POINT = "GO_A_POINT"
CMD_GO_B_POINT = "GO_B_POINT"
CMD_GO_HOME = "GO_HOME"

# 上报调度的到达状态
STATE_ARRIVE_A = "ARRIVE_A"
STATE_ARRIVE_B = "ARRIVE_B"
STATE_ARRIVE_HOME = "ARRIVE_HOME"

# Seer导航状态码
NAV_IDLE = 0
NAV_RUNNING = 1
NAV_ARRIVE = 2
NAV_FAIL = 3

# ========== 修改：适配你现有LM1/LM2/LM3点位 ==========
TARGET_MAP = {
    CMD_GO_A_POINT: "LM2",
    CMD_GO_B_POINT: "LM3",
    CMD_GO_HOME: "LM1"
}
STATE_MAP = {
    "LM2": STATE_ARRIVE_A,
    "LM3": STATE_ARRIVE_B,
    "LM1": STATE_ARRIVE_HOME
}

class ChassisDriverNode(Node):
    def __init__(self):
        super().__init__("chassis_driver_node")
        self.get_logger().info("===== 仙工Seer真实底盘驱动节点启动 =====")

        # 1. 订阅调度运动指令
        self.cmd_sub = self.create_subscription(
            String,
            TOPIC_CMD_CHASSIS,
            self.cmd_callback,
            10
        )
        # 2. 发布到达状态给调度
        self.state_pub = self.create_publisher(String, TOPIC_STATE_CHASSIS, 10)
        # 3. 下发导航目标给Seer底盘
        self.seer_target_pub = self.create_publisher(String, SEER_GO_TARGET, 10)
        # 4. 监听Seer导航实时状态
        self.nav_status_sub = self.create_subscription(
            Int32,
            SEER_NAV_STATUS,
            self.nav_status_callback,
            10
        )
        # 5. 取消导航客户端
        self.cancel_nav_cli = self.create_client(Trigger, SRV_CANCEL_NAV)
        while not self.cancel_nav_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("等待取消导航服务 /seer/cancel_navigation 上线...")

        # 运行状态缓存
        self.is_nav_running = False
        self.current_target_station = None  # 当前要去的站点 LM1/LM2/LM3
        self.seer_nav_status = NAV_IDLE

    def nav_status_callback(self, msg: Int32):
        """监听Seer底盘导航状态变化"""
        self.seer_nav_status = msg.data

        if self.is_nav_running:
            if self.seer_nav_status == NAV_ARRIVE:
                # 到达目标，上报状态给调度
                self.publish_arrive_state()
                self.is_nav_running = False
                self.current_target_station = None
                self.get_logger().info(f"底盘抵达站点，任务完成")
            elif self.seer_nav_status == NAV_FAIL:
                self.get_logger().error(f"导航失败！障碍物/定位丢失")
                self.is_nav_running = False
                self.current_target_station = None

    def cmd_callback(self, msg: String):
        """接收调度下发点位指令"""
        cmd = msg.data.strip()
        self.get_logger().info(f"收到调度底盘指令: {cmd}")

        # 校验指令合法性
        if cmd not in TARGET_MAP.keys():
            self.get_logger().error(f"未知底盘指令: {cmd}")
            return

        # 底盘正在运动，拒绝新指令
        if self.is_nav_running:
            self.get_logger().warn(f"底盘正在运动，忽略新指令 {cmd}")
            return

        # 转换为地图现有站点ID
        station_id = TARGET_MAP[cmd]
        self.current_target_station = station_id
        self.is_nav_running = True

        # 下发导航目标给Seer底盘
        target_msg = String()
        target_msg.data = station_id
        self.seer_target_pub.publish(target_msg)
        self.get_logger().info(f"下发导航指令至Seer底盘，目标站点: {station_id}")

    def publish_arrive_state(self):
        """到达点位后上报完成状态给调度"""
        if self.current_target_station not in STATE_MAP:
            return
        msg = String()
        msg.data = STATE_MAP[self.current_target_station]
        self.state_pub.publish(msg)
        self.get_logger().info(f"上报调度到达状态: {msg.data}")

    def cancel_all_navigation(self):
        """外部触发取消导航（急停/复位预留）"""
        req = Trigger.Request()
        future = self.cancel_nav_cli.call_async(req)
        self.get_logger().info("发送取消导航请求")

def main(args=None):
    rclpy.init(args=args)
    node = ChassisDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

