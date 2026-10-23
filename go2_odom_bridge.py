#!/usr/bin/env python3
"""
go2_odom_bridge.py — Go2 四足解算位姿桥 v2（Orin 上跑，纯 rclpy/FastDDS 单栈）
订阅固件 /lf/sportmodestate (unitree_go/SportModeState, best_effort)
→ 发布 /go2/odom (nav_msgs/Odometry, best_effort)，供工作站定位节点做
  四足航位推算融合（LIO 健康裁判 + 退化死推）。

为什么不用 SDK2/CycloneDDS 双栈：固件话题在 FastDDS 域里可发现（ros2 topic list
可见），rclpy 直接订阅即可；SDK2 ChannelSubscriber 路线此前始终收不到回调。
注意：狗趴下/趴窝（非 sport 状态）时固件对该话题静默属正常——把狗唤醒才有数据。

帧名: dog_odom / go2_base（独立于 LIO 栈的 map/camera_init/body）。
时间戳: 用 Orin 本机 now() 重打（LIO 消息戳同样源自 Orin 时钟，下游按各自流的
  内部差分对窗口，跨钟偏差不敏感）。
Unitree 四元数顺序 [w,x,y,z] → ROS [x,y,z,w]。
野值防护: 相邻样本速度超 dog_jump_speed 视为狗里程计复位（standup/recovery 会
  重置原点），清空基准重新开始——下游只用相对差分，复位不影响。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState

DOG_JUMP_SPEED = 1.5  # m/s，超过视为狗里程计复位/野值（巡检速度上限 ~0.5）
WARN_QUIET = 3.0      # 秒，无数据静默告警间隔
LOG_EVERY = 30.0      # 秒，例行心跳日志间隔


class Go2OdomBridge(Node):
    def __init__(self):
        super().__init__("go2_odom_bridge")
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(
            SportModeState, "/lf/sportmodestate", self.on_state, qos_sub)
        self.pub = self.create_publisher(Odometry, "/go2/odom", qos_pub)
        self.n = 0
        self.resets = 0
        self.last_pos = None
        self.last_warn = 0.0
        self.last_log = time.time()
        self.last_rx = time.time()
        self.t_start = time.time()
        self.get_logger().info("go2_odom_bridge v2 就绪（等 /lf/sportmodestate 数据；"
                               "狗趴着时静默属正常）")

    def on_state(self, msg):
        now = time.time()
        pos = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
        # 野值/复位检测：瞬移按速度算超限 → 丢样本并清基准
        if self.last_pos is not None:
            dp = ((pos[0] - self.last_pos[0]) ** 2 +
                  (pos[1] - self.last_pos[1]) ** 2) ** 0.5
            dt = now - self.last_rx if self.last_rx else 1.0
            if dt > 1e-3 and dp / dt > DOG_JUMP_SPEED:
                self.resets += 1
                self.last_pos = None
                self.get_logger().warn(
                    "狗里程计疑似复位/野值(%.2fm/%.0fms)，重建基准(第%d次)"
                    % (dp, dt * 1e3, self.resets))
                self.last_rx = now
                return
        self.last_pos = pos
        self.last_rx = now
        self.n += 1

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "dog_odom"
        odom.child_frame_id = "go2_base"
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        # Unitree [w,x,y,z] → ROS [x,y,z,w]
        q = msg.imu_state.quaternion
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.pose.pose.orientation.w = float(q[0])
        # 速度按机体系理解（Unitree 惯例），角速度只取 yaw_speed
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self.pub.publish(odom)

        if now - self.last_log > LOG_EVERY:
            self.last_log = now
            rate = self.n / LOG_EVERY if self.n else 0.0
            self.get_logger().info(
                "回传中 pos=(%.2f,%.2f) 速度=(%.2f,%.2f) ~%.0fHz 复位%d次"
                % (pos[0], pos[1], msg.velocity[0], msg.velocity[1], rate,
                   self.resets))
            self.n = 0

    def tick(self):
        """无数据告警（狗唤醒前后状态切换时最容易误判"没配好"）"""
        now = time.time()
        silent = (now - self.last_rx) if self.last_pos is not None else (now - self.t_start)
        if silent > 5.0 and now - self.last_warn > WARN_QUIET:
            self.last_warn = now
            self.get_logger().warn(
                "sportmodestate 静默 %.0fs（狗趴着/未唤醒时固件不发，唤醒站立即有数据）"
                % silent)


def main():
    rclpy.init()
    node = Go2OdomBridge()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            node.tick()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
