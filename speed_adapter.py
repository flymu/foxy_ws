#!/usr/bin/env python3
"""speed_adapter.py — 运动适配器：狗可执行速度域的地板/停步语义（2026-11-05 G1）

为什么存在（评审 R2 + 实弹教训）：狗的速度空间物理上只有两档——起步阈值
实测 0.20~0.25m/s，低于阈值狗原地罚站；接近目标的减速段一旦跌进
(0, 0.25) 区间，狗当场假死（2026-11-04 creep_test 实锤）。旧版把这个
地板塞在 veto 里靠"抬升残速"实现——那是放大速度、越权（G1 定案：最终
否决层只能抑制或停止）。正确分工：

  规划器(/cmd_vel_intent) → 本适配器 → /cmd_vel_raw → veto(Orin) → 桥
                          ↑ 起步/停步地板在这里
                            否决层此后只做纯抑制

规则（平移速度模长 sp = hypot(vx,vy)）：
  sp <  v_stop(0.05)     → 原样转发（规划器要停就停，含零）
  v_stop ≤ sp < v_min    → 等比抬到 v_min_walk（保方向）——想走就走得起
  sp ≥ v_min             → 原样转发（不削峰，限幅是桥的职责）
  角速度 vyaw 永远原样（原地转向无起步阈值问题）。

注意：本节点不是安全件——它挂了 cmd 流就断，桥 0.5s 看门狗自然停车；
它输出的速度仍要过 veto 的几何否决。跑在工作站（与规划器同机，零额外
WiFi：/cmd_vel_intent 本地传递，跨 WiFi 的还是那一路 /cmd_vel_raw 小报文）。
用法（foxy-live 容器内，live_stack_start.sh 已带）: python3 speed_adapter.py
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class SpeedAdapter(Node):
    def __init__(self):
        super().__init__("speed_adapter")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.v_min = p("v_min_walk", 0.25)   # 狗起步阈值（11-05 实测 0.20~0.25）
        self.v_stop = p("v_stop", 0.05)      # 低于此视为"要停"，不抬
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Twist, "/cmd_vel_raw", q)
        self.create_subscription(Twist, "/cmd_vel_intent", self.on_cmd, q)
        self.n_in = self.n_floor = self.n_stop = 0
        self.t0 = self.get_clock().now()
        self.create_timer(10.0, self.stats)
        self.get_logger().info(
            "speed_adapter 就绪: /cmd_vel_intent → /cmd_vel_raw, "
            "地板 %.2f 停步死区 %.2f" % (self.v_min, self.v_stop))

    def on_cmd(self, m):
        vx, vy, wz = float(m.linear.x), float(m.linear.y), float(m.angular.z)
        sp = math.hypot(vx, vy)
        self.n_in += 1
        if self.v_stop <= sp < self.v_min:  # 起步死区内的"幽灵速度"：等比抬到地板
            s = self.v_min / sp
            vx, vy = vx * s, vy * s
            self.n_floor += 1
        elif sp < self.v_stop:
            self.n_stop += 1
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.pub.publish(out)

    def stats(self):
        dt = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if dt > 0.5 and self.n_in:
            self.get_logger().info(
                "适配: %.1fHz 抬地板 %d 停步 %d 原样 %d"
                % (self.n_in / dt, self.n_floor, self.n_stop,
                   self.n_in - self.n_floor - self.n_stop))
        self.n_in = self.n_floor = self.n_stop = 0
        self.t0 = self.get_clock().now()


def main():
    rclpy.init(args=sys.argv)
    try:
        rclpy.spin(SpeedAdapter())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()