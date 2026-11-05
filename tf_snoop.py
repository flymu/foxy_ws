#!/usr/bin/env python3
"""tf_snoop.py — 抓 livox_frame→veto_dog 的一条 TF（Orin daemon 瞎，CLI 看不见）。
用法: python3 tf_snoop.py [等待秒数=6]
"""
import sys

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def main():
    wait_s = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Node("tf_snoop")
    got = []

    def cb(m):
        for t in m.transforms:
            if t.header.frame_id == "livox_frame" and t.child_frame_id == "veto_dog":
                got.append(t)

    node.create_subscription(TFMessage, "/tf", cb, 10)
    t0 = node.get_clock().now()
    while not got and (node.get_clock().now() - t0).nanoseconds < wait_s * 1e9:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    if got:
        t = got[-1]
        tr, q = t.transform.translation, t.transform.rotation
        print("livox_frame→veto_dog: t=(%.4f, %.4f, %.4f) q=(%.4f, %.4f, %.4f, %.4f)"
              % (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w))
        return 0
    print("NO_TF")
    return 1


if __name__ == "__main__":
    sys.exit(main())
