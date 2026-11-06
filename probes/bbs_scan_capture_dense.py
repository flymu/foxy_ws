#!/usr/bin/env python3
"""bbs_scan_capture_dense.py — 稠密 BBS 扫描抓取 (修 /cloud_registered 特征子集过稀 1153 voxel 的问题)

源 = /livox/points2 (livox_relay 从 CustomMsg 转的稠密 PointCloud2, 传感器系, 桅杆倾斜 ~39°)
姿态 = /Odometry (point_lio, camera_init 重力对齐系) 最新位姿; 狗静止 → 常姿态足够
变换 p_odom = R_odom_body · (p_lidar + T_extrinsic) + t_odom_body
  (mid360.yaml extrinsic_R=I, T=(0.011,0.023,-0.044), 仅 4cm 量级)
然后同配方: 剥地面带 (5%分位地面 +0.15~+2.8) + 15m 截断 → 二进制 XYZ PCD

容器内跑: python3 /home/mu/foxy_ws/bbs_scan_capture_dense.py [帧数=12] [输出=/home/mu/maps/bbs_dense_scan.pcd]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

N_FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 12
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/mu/maps/bbs_dense_scan.pcd"
T_EX = np.array([0.011, 0.02329, -0.04412])  # mid360.yaml extrinsic_T (IMU->雷达, R=I)


def quat_to_R(x, y, z, w):
    R = np.empty((3, 3))
    R[0, 0] = 1 - 2 * (y * y + z * z); R[0, 1] = 2 * (x * y - z * w); R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w);     R[1, 1] = 1 - 2 * (x * x + z * z); R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w);     R[2, 1] = 2 * (y * z + x * w);     R[2, 2] = 1 - 2 * (x * x + y * y)
    return R


class Cap(Node):
    def __init__(self):
        super().__init__("bbs_scan_capture_dense")
        self.pts = None
        self.n = 0
        self.odo = None  # (R, t)
        self.n_odo = 0
        self.create_subscription(PointCloud2, "/livox/points2", self.cb_pts, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/Odometry", self.cb_odo, 20)

    def cb_odo(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.odo = (quat_to_R(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))
        self.n_odo += 1

    def cb_pts(self, m):
        if self.odo is None:
            self.get_logger().info("等 /Odometry ...", throttle_duration_sec=2.0)
            return
        step = m.point_step
        raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, step)
        offs = {f.name: f.offset for f in m.fields}

        def f32(name):
            o = offs[name]
            return raw[:, o:o + 4].copy().view(np.float32).ravel()
        xyz = np.stack([f32("x"), f32("y"), f32("z")], axis=1).astype(np.float64)
        R, t = self.odo
        xyz = (R @ (xyz + T_EX).T).T + t  # p_odom = R·(p_lidar+T_ex)+t
        self.pts = xyz if self.pts is None else np.concatenate([self.pts, xyz], axis=0)
        self.n += 1


def write_xyz_pcd(path, pts):
    arr = np.ascontiguousarray(pts.astype(np.float32))
    n = len(arr)
    with open(path, "wb") as f:
        f.write(b"# .PCD v0.7 - Point Cloud Data file format\n")
        f.write(b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode())
        f.write(arr.tobytes())


def main():
    rclpy.init()
    node = Cap()
    t0 = time.time()
    while node.n < N_FRAMES and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.5)
    print(f"[capture] 点云帧={node.n} odometry帧={node.n_odo}")
    pts = node.pts
    if pts is None:
        print("无帧! /livox/points2 未发布 (检查 livox_relay 是否在 Orin 上活着)")
        sys.exit(1)
    print(f"[capture] 原始 {len(pts)} pts "
          f"bbox x[{pts[:,0].min():.1f},{pts[:,0].max():.1f}] "
          f"y[{pts[:,1].min():.1f},{pts[:,1].max():.1f}] z[{pts[:,2].min():.1f},{pts[:,2].max():.1f}]")
    # 地面平直度自检: 变换正确则地面点应集中在薄层
    gz = np.percentile(pts[:, 2], 5)
    low = pts[pts[:, 2] < gz + 0.15]
    if len(low):
        print(f"[check] 地面层z={gz:.2f} 低带 {len(low)} pts, 厚度(std)={low[:,2].std():.3f}m (应<0.05)")
    m = (pts[:, 2] > gz + 0.15) & (pts[:, 2] < gz + 2.8)
    keep = pts[m]
    print(f"[strip] 剥除后 {len(keep)} pts ({len(keep)/len(pts)*100:.0f}%)")
    d2 = (keep ** 2).sum(axis=1)
    keep = keep[d2 < 15.0 ** 2]
    print(f"[range] 15m 内 {len(keep)} pts")
    write_xyz_pcd(OUT, keep)
    print(f"[done] {OUT} {len(keep)} pts")


if __name__ == "__main__":
    main()
