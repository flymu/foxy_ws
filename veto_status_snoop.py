#!/usr/bin/env python3
"""veto_status_snoop.py — 一次性拉取 /veto/status（Orin daemon 瞎，CLI 看不见本机话题）。
用法: python3 veto_status_snoop.py [等待秒数=8]
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    wait_s = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Node("veto_status_snoop")
    got = []

    def cb(m):
        got.append(m.data)

    node.create_subscription(String, "/veto/status", cb, 5)
    t0 = node.get_clock().now()
    while not got and (node.get_clock().now() - t0).nanoseconds < wait_s * 1e9:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    if got:
        print(got[-1])
        return 0
    print("NO_STATUS（%0.1fs 内没等到——veto 没在跑？）" % wait_s)
    return 1


if __name__ == "__main__":
    sys.exit(main())
