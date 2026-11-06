#!/usr/bin/env python3
"""bbs_frame_probe.py — 量 /cloud_registered 或指定话题每帧点数+真实到包率+戳龄（诊断用）

rclpy 读大点云有 ~5Hz 天花板（D2 教训），所以本探针报的"到包率"是读侧下界；
每帧点数才是本探针的目的（判断帧是否被截小/解析是否正确）。

用法: python3 bbs_frame_probe.py [topic] [duration_s]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Probe(Node):
    def __init__(self, topic):
        super().__init__("frame_probe")
        self.n = 0
        self.npts = []
        self.first_wall = None
        self.last_wall = None
        self.ages = []
        self.create_subscription(PointCloud2, topic, self.cb,
                                  QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, m):
        w = time.time()
        if self.first_wall is None:
            self.first_wall = w
        self.last_wall = w
        self.n += 1
        self.npts.append(m.width * m.height)
        self.ages.append(w - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
        self.get_logger().info(
            f"frame {self.n}: {m.width * m.height} pts "
            f"(fields={len(m.fields)} step={m.point_step}) "
            f"stamp_age={self.ages[-1]:.3f}")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/cloud_registered"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    p = Probe(topic)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < dur:
        rclpy.spin_once(p, timeout_sec=0.5)
    span = (p.last_wall - p.first_wall) if p.n >= 2 else 0.0
    if p.n:
        print(f"[probe] {topic}: {p.n} 帧 / {span:.2f}s = 读侧 {p.n / span if span else 0:.2f}Hz "
              f"(下界) | 每帧点数 min={min(p.npts)} med={sorted(p.npts)[len(p.npts) // 2]} "
              f"max={max(p.npts)} | 戳龄 med={sorted(p.ages)[len(p.ages) // 2]:.3f}s", flush=True)
    else:
        print(f"[probe] {topic}: 0 帧", flush=True)
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
