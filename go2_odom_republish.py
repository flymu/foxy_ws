#!/usr/bin/env python3
"""
go2_odom_republish.py — Orin 侧四足航位推算两段接力第 2 段（默认 FastDDS 栈）
订阅第 1 段 go2_odom_bridge（挂在 rmw_cyclonedds+eth1 上的）发出的 /go2/odom_src，
原样透传为 /go2/odom 供工作站定位节点订阅。

为什么需要两段（详见 go2_odom_bridge.py 头注释）:
  主板 DDS 只认 123.x 内网 locator，FastDDS 无论如何配都收不到主板数据；
  唯一可行解 = cyclone 钉 eth1 收数据，但那个参与者工作站又发现不了。
  所以本机跨栈接力一次：cyclone(eth1) → FastDDS(默认) → 工作站(wlan0)。
消息体已在第 1 段用 Orin 时钟重打、四元数已转换，这里只透传不加工。
启动: bash /home/unitree/go2_odom_start.sh（两段一起，无需单独起）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class Repub(Node):
    def __init__(self):
        super().__init__("go2_odom_republish")
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.n = 0
        self.sub = self.create_subscription(Odometry, "/go2/odom_src",
                                            self.on_odom, qos_sub)
        self.pub = self.create_publisher(Odometry, "/go2/odom", qos_pub)
        self.t0 = rclpy.clock.Clock().now().nanoseconds / 1e9
        self.get_logger().info("go2_odom_republish 就绪（等 /go2/odom_src）")

    def on_odom(self, msg):
        self.n += 1
        self.pub.publish(msg)
        if self.n % 600 == 0:
            self.get_logger().info("透传中 累计%d条" % self.n)


def main():
    rclpy.init()
    node = Repub()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
