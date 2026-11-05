#!/usr/bin/env python3
"""
go2_odom_bridge.py — Go2 四足解算位姿桥 v3（Orin 上跑，两段接力第 1 段）
订阅固件 /lf/sportmodestate (unitree_go/SportModeState, best_effort)
→ 发布 /go2/odom_src (nav_msgs/Odometry, best_effort)，
由 go2_odom_republish.py（第 2 段，默认 FastDDS 栈）透传为 /go2/odom 给工作站，
供定位节点做四足航位推算融合（LIO 健康裁判 + 退化死推）。

为什么必须两段、且本段必须挂 rmw_cyclonedds（2026-10-31 实测定论）:
  主板 DDS 是 Cyclone 品格、只认 192.168.123.x 内网可达的 locator——
  rclpy 默认 FastDDS 参与者无论如何配 XML 都完不成与主板的发现
  （publisher count 恒 0，收不到任何数据）；
  而 FastDDS 的默认 locator 走默认路由网卡(wlan0)，主板也无法反向完成发现。
  实测唯一能收到数据的方式 = rmw_cyclonedds + CYCLONEDDS_URI 钉在 eth1
  （unitree 自带的 /home/unitree/cyclonedds_ws/cyclonedds.xml，20Hz 稳定）。
  但钉死 eth1 的参与者只在 eth1 广播，工作站（wlan0 网段）永远发现不了它，
  所以再由默认栈的第 2 段在本机跨栈透传一次。
启动: bash /home/unitree/go2_odom_start.sh（两段一起，环境变量都在里面）。
狗趴下/未唤醒时固件对该话题静默属正常——唤醒站起来立即有数据。

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
        self.pub = self.create_publisher(Odometry, "/go2/odom_src", qos_pub)
        self.n = 0
        self.resets = 0
        self.last_pos = None
        self.last_warn = 0.0
        self.last_log = time.time()
        self.last_rx = time.time()
        self.t_start = time.time()
        self.get_logger().info("go2_odom_bridge v3 就绪（等 /lf/sportmodestate 数据；"
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
