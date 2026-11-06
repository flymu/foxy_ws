#!/usr/bin/env python3
"""bbs_probe_topics.py — 摸清 /livox/lidar 与 /Odometry 的字段与密度, 供稠密 BBS 扫描用。
容器内跑: python3 /home/mu/foxy_ws/bbs_probe_topics.py"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node("bbs_probe_topics")
    res = {}

    def on_lidar(m):
        if "lidar" in res:
            return
        res["lidar"] = {
            "n": m.width * m.height,
            "step": m.point_step,
            "frame": m.header.frame_id,
            "fields": [(f.name, f.offset, f.datatype) for f in m.fields],
        }

    def on_odo(m):
        if "odo" in res:
            return
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        res["odo"] = {
            "frame_id": m.header.frame_id,
            "child": m.child_frame_id,
            "t": (round(p.x, 4), round(p.y, 4), round(p.z, 4)),
            "q": (round(q.x, 4), round(q.y, 4), round(q.z, 4), round(q.w, 4)),
        }

    node.create_subscription(PointCloud2, "/livox/lidar", on_lidar, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/Odometry", on_odo, 20)
    t0 = time.time()
    while time.time() - t0 < 8 and len(res) < 2:
        rclpy.spin_once(node, timeout_sec=0.4)
    for k in ("lidar", "odo"):
        print(f"[{k}] {res.get(k, 'TIMEOUT 未收到')}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
