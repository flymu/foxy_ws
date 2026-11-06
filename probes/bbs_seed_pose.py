#!/usr/bin/env python3
"""bbs_seed_pose.py — BBS 全局重定位种子的 /initialpose 发布器（容器内跑）

与 r5_seed_pose.py 的区别：种子不是"当前 loc 位姿+N"，而是 BBS 全局搜索给的
T_map_odom（map←odom），需在发布时刻与最新 /Odometry（odom←body）合成
T_map_body = T_mo(BBS) · T_ob(now) 再发——狗在 BBS 采样后挪过也不怕戳龄。

预期效果（健康追踪期）：loc 收到后重设 T_mo ← seed 并由 NDT 当帧精修回真值，
位姿不应可见跳变（R5 回归 Run1 已验证该语义）。日志应出现
  [reseed] T_mb=(x, y, yaw deg° …
与 seed 打印值一致（±0.05）。

用法:
  python3 bbs_seed_pose.py --tmo "m00,m01,m02,tx,m10,m11,m12,ty,m20,m21,m22,tz"
      （BBS bbs_pose 输出的 3x4 矩阵，逗号分隔 12 个数）
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def quat_from_matrix(R):
    """旋转矩阵(3x3) → 四元数 (x,y,z,w)。"""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class Seeder(Node):
    def __init__(self, tmo):
        super().__init__("bbs_seed_pose")
        self.tmo = tmo
        self.tob = None
        self.create_subscription(Odometry, "/Odometry", self.on_odom,
                                  QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        R = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w),
             p.x],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w),
             p.y],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y),
             p.z],
            [0, 0, 0, 1]])
        self.tob = R

    def have_odom(self):
        return self.tob is not None


def main():
    tmo_vals = None
    for i, a in enumerate(sys.argv):
        if a == "--tmo":
            tmo_vals = [float(v) for v in sys.argv[i + 1].split(",")]
    if tmo_vals is None or len(tmo_vals) != 12:
        raise SystemExit("需要 --tmo \"3x4 矩阵 12 个数\"")
    tmo = np.array(tmo_vals).reshape(3, 4)
    tmo = np.vstack([tmo, [0, 0, 0, 1]])

    rclpy.init()
    node = Seeder(tmo)
    t0 = time.time()
    while rclpy.ok() and not node.have_odom() and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.have_odom():
        print("[bbs_seed_pose] !! 5s 没收到 /Odometry（point_lio 未起?）", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    tmb = node.tmo @ node.tob
    x, y, z = tmb[0, 3], tmb[1, 3], tmb[2, 3]
    qx, qy, qz, qw = quat_from_matrix(tmb[:3, :3])
    yaw = math.degrees(math.atan2(tmb[1, 0], tmb[0, 0])) % 360
    print(f"[bbs_seed_pose] seed T_mb=({x:.3f},{y:.3f}) yaw={yaw:.1f}° (z={z:.3f})", flush=True)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    t0 = time.time()
    while pub.get_subscription_count() < 1 and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() < 1:
        print("[bbs_seed_pose] !! 无订阅者(loc 未起?)", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    pub.publish(msg)
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print("[bbs_seed_pose] 已发", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
