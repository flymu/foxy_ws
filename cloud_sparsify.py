#!/usr/bin/env python3
"""
cloud_sparsify.py — Orin 侧点云稀疏中继（2026-11-05，RViz 远程 3D 观景用）
/livox/points2 (全量 ~20k点/帧) → /livox/points_sparse (体素 0.05m 降采样)

为什么存在: RViz 在工作站、点云在 Orin——WiFi 上每个订阅者一份单播
(D1 教训)，全量 320KB/帧×10Hz≈26Mbps 一路就把 WiFi 打进棘轮。
体素降采样后 ~60KB/帧≈5Mbps，且 Mid360 非重复扫描在 RViz Decay 累积
下稀疏帧拼成完整 3D 场景，观感不损失。

实现: np.frombuffer 零拷贝吃 D2 格式(16B/点=x,y,z,intensity)；
体素哈希(每轴21bit打包int64)取每体素首点；强度原样保留；
header 原样保留(戳+livox_frame)。QoS: 订 BE depth10，发 BE depth5。

用法: python3 cloud_sparsify.py [体素米=0.05]     (需 OPENBLAS_NUM_THREADS=1 环境)
"""
import array
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudSparsify(Node):
    def __init__(self, voxel):
        super().__init__("cloud_sparsify")
        self.voxel = float(voxel)
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PointCloud2, "/livox/points_sparse", 5)
        self.create_subscription(PointCloud2, "/livox/points2", self.on_cloud, qb)
        self.n_in = 0
        self.last_in_pts = 0
        self.last_out_pts = 0
        self.t0 = time.monotonic()
        self.create_timer(10.0, self.stats)
        self.get_logger().info(
            "cloud_sparsify 就绪: %.2fm 体素, /livox/points2 → /livox/points_sparse"
            % self.voxel)

    def on_cloud(self, m):
        if m.point_step != 16 or m.width < 64:
            return  # 异格式/病云不转发（防把驱动病传染给观景端）
        arr = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
        # 体素哈希: 每轴 21bit 打包 int64（±2048 体素 = ±100m 余量）
        k = np.floor(arr[:, :3] / self.voxel).astype(np.int64)
        k += 1 << 11
        key = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
        _, idx = np.unique(key, return_index=True)
        out = arr[np.sort(idx)]
        m.width = len(out)
        m.row_step = len(out) * m.point_step
        m.data = array.array("B", out.tobytes())
        self.pub.publish(m)
        self.n_in += 1
        self.last_in_pts, self.last_out_pts = len(arr), len(out)

    def stats(self):
        dt = time.monotonic() - self.t0
        if dt > 0.5:
            self.get_logger().info(
                "稀疏: %.2f Hz (10s窗) 上帧 %d→%d 点"
                % (self.n_in / dt, self.last_in_pts, self.last_out_pts))
        self.n_in = 0
        self.t0 = time.monotonic()


def main():
    voxel = sys.argv[1] if len(sys.argv) > 1 else "0.05"
    rclpy.init()
    node = CloudSparsify(voxel)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
