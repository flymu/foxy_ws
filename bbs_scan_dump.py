#!/usr/bin/env python3
"""bbs_scan_dump.py — 回放期从 /cloud_registered 采 BBS 子图扫描（容器内跑）

设计要点：
- 子图取 odom 系（点云已配准免畸变；odom 系=重力对齐，BBS 无需 IMU 对齐）
- 1.0s 窗 + 0.15m 体素降采样 + 按窗内 z 5%分位剥地面带（室内退化教训）
- 每窗写 scan_<t>.pcd（二进制 XYZ）+ meta.json（窗末 /Odometry 位姿，留作 GT 备用）
- 本图自建自放 → 期望 BBS T≈I；加已知扰动的副本做判别力测试在主机侧另做

用法（e2e_bbs_dump.sh 一键；手动）:
  ROS_DOMAIN_ID=77 python3 bbs_scan_dump.py --windows 20,50,80,110,140,170,200
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

WIN_DUR = 1.0
LEAF = 0.15
STRIP_M = 0.15          # 地面带厚度（z5%分位之上保留）
Z_KEEP = 4.0            # 剥天花板之上保留


class ScanDumper(Node):
    def __init__(self, outdir, windows, wlen):
        super().__init__("bbs_scan_dump")
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self.windows = list(windows)          # 相对首朵云的秒数
        self.wlen = wlen                     # 每窗采集时长（狗静止时可加长换密度）
        self.win_idx = 0
        self.t0 = None
        self.buf = []
        # BEST_EFFORT 与 loc/planner 消费者一致；RELIABLE 会让慢 rclpy 读者触发
        # 重传/背压，实测出现 3s+ 无回调（11-06 live dump 事故：1s 窗只收到 2 帧）
        self.qr = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "/cloud_registered", self.on_cloud, self.qr)
        self.create_subscription(Odometry, "/Odometry", self.on_odom, self.qr)
        self.last_odom = None
        self.done = 0

    def on_odom(self, msg):
        self.last_odom = msg

    def on_cloud(self, msg):
        if self.win_idx >= len(self.windows):
            return
        # 通用字段偏移解析（x,y,z float32）
        off = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                off[f.name] = f.offset
        if len(off) < 3:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        pts = np.stack([raw[:, off["x"]:off["x"] + 4].copy().view(np.float32).ravel(),
                        raw[:, off["y"]:off["y"] + 4].copy().view(np.float32).ravel(),
                        raw[:, off["z"]:off["z"] + 4].copy().view(np.float32).ravel()], axis=1)
        pts = pts[np.isfinite(pts).all(axis=1)]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        rel = stamp - self.t0
        w = self.windows[self.win_idx]
        if rel < w:
            return
        if rel <= w + self.wlen:
            self.buf.append(pts)
            return
        # 窗关——落盘
        cloud = np.concatenate(self.buf) if self.buf else np.empty((0, 3))
        self.buf = [pts]  # 当前帧进下一窗缓冲（无害：下一窗起点靠 rel 判定重置）
        # 下一窗若与当前帧已过界，直接顺延处理由后续回调推进
        self.flush(w, rel, cloud)
        self.win_idx += 1
        # 当前帧可能已属于更晚的窗——递进检查交给下帧；窗口不重叠则无碍
        if self.win_idx < len(self.windows) and stamp - self.t0 > self.windows[self.win_idx] + self.wlen:
            # 稀疏窗口漏采（帧间跳变）：空窗跳过
            self.get_logger().warn(f"窗口 {self.windows[self.win_idx]} 无帧，跳过")
            self.win_idx += 1

    def flush(self, w, rel, cloud):
        name = f"scan_t{int(round(w)):03d}"
        if len(cloud) < 2000:
            self.get_logger().warn(f"{name} 仅 {len(cloud)} 点，丢弃")
            return
        # 体素降采样（首点代表）
        key = np.floor(cloud / LEAF).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        cloud = cloud[idx]
        # 剥地面带：窗内 z 5% 分位视作地面
        z5 = np.percentile(cloud[:, 2], 5)
        keep = (cloud[:, 2] > z5 + STRIP_M) & (cloud[:, 2] < z5 + Z_KEEP)
        cloud = cloud[keep]
        path = os.path.join(self.outdir, name + ".pcd")
        arr = np.ascontiguousarray(cloud.astype(np.float32))
        n = len(arr)
        with open(path, "wb") as f:
            f.write(b"# .PCD v0.7 - Point Cloud Data file format\n")
            f.write(b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode())
            f.write(arr.tobytes())
        meta = {"window_s": w, "n_points": n, "rel_end": rel}
        if self.last_odom is not None:
            p = self.last_odom.pose.pose.position
            q = self.last_odom.pose.pose.orientation
            meta["odom_end"] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                 self.last_odom.header.stamp.sec + self.last_odom.header.stamp.nanosec * 1e-9]
        mf = os.path.join(self.outdir, "meta.json")
        m = {}
        if os.path.exists(mf):
            with open(mf) as f:
                m = json.load(f)
        m[name] = meta
        with open(mf, "w") as f:
            json.dump(m, f, indent=1)
        self.done += 1
        self.get_logger().info(f"{name}: {n} pts 落盘 (odometry {'有' if self.last_odom else '无'})")

    def finished(self):
        return self.win_idx >= len(self.windows)


def main():
    outdir = "/home/mu/bags/bbs_eval"
    windows = [20, 50, 80, 110, 140, 170, 200]
    wlen = WIN_DUR
    for i, a in enumerate(sys.argv):
        if a == "--windows":
            windows = [float(x) for x in sys.argv[i + 1].split(",")]
        elif a == "--outdir":
            outdir = sys.argv[i + 1]
        elif a == "--wlen":
            wlen = float(sys.argv[i + 1])
    rclpy.init()
    node = ScanDumper(outdir, windows, wlen)
    print(f"[bbs_scan_dump] outdir={outdir} windows={windows} wlen={wlen}", flush=True)
    while rclpy.ok() and not node.finished():
        rclpy.spin_once(node, timeout_sec=1.0)
    print(f"[bbs_scan_dump] 完成 {node.done}/{len(windows)} 窗", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.done > 0 else 2)


if __name__ == "__main__":
    main()
