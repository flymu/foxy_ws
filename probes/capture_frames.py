#!/usr/bin/env python3
"""capture_frames.py — 订阅 /livox/lidar 采集原始帧存 npz（标定/测试数据用）

BEST_EFFORT 订阅（与驱动/回放发布匹配）。存单文件：pts(M,3 float32) + counts(N)。
用法（foxy-live 容器内，另开 ros2 bag play 或真机雷达）:
  source /opt/ros/foxy/setup.bash && source ~/foxy_ws/install_x86/setup.bash
  python3 ~/foxy_ws/capture_frames.py 输出.npz 跳过帧数 采集帧数
"""
import sys
import time

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/veto_frames.npz"
SKIP = int(sys.argv[2]) if len(sys.argv) > 2 else 0
N = int(sys.argv[3]) if len(sys.argv) > 3 else 40
STRIDE = int(sys.argv[4]) if len(sys.argv) > 4 else 0  # >0: 每采1帧隔 STRIDE 帧（时间采样）
WARMUP = 8.0  # 同容器发现一般 1-3s，跨 WiFi 更久；只影响跳过计数起点


def main():
    rclpy.init()
    node = Node("capture_frames")
    q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST, depth=10)
    st = {"frames": [], "skipped": 0, "first_t": None}

    def on_msg(m):
        if st["first_t"] is None:
            st["first_t"] = time.monotonic()
        if st["skipped"] < SKIP:
            st["skipped"] += 1
            return
        if len(st["frames"]) >= N:
            return
        if st["frames"] and STRIDE > 0 and st["skipped"] < SKIP + 1 + STRIDE * len(st["frames"]):
            st["skipped"] += 1
            return
        st["frames"].append(np.array([[p.x, p.y, p.z] for p in m.points],
                                      dtype=np.float32))

    node.create_subscription(CustomMsg, "/livox/lidar", on_msg, q)
    t0 = time.time()
    need_s = WARMUP + (SKIP + N * (STRIDE + 1)) * 0.12 + 30
    while len(st["frames"]) < N and time.time() - t0 < need_s:
        rclpy.spin_once(node, timeout_sec=0.5)
    frames = st["frames"]
    print("收到帧 %d (跳过 %d)" % (len(frames), st["skipped"]))
    if frames:
        counts = np.array([len(f) for f in frames], dtype=np.int32)
        pts = np.vstack(frames) if frames else np.zeros((0, 3), np.float32)
        np.savez(OUT, pts=pts, counts=counts)
        print("已存 %s: %d 点 / %d 帧, 每帧 min/avg/max = %d/%.0f/%d"
              % (OUT, len(pts), len(counts), counts.min(), counts.mean(), counts.max()))
    else:
        print("未收到任何帧")
        sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
