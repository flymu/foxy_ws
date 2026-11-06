#!/usr/bin/env python3
"""bbs_hold_watch.py — 重种子后守护验证: 每2s采样 /loc/pose + /loc/health 共N秒(默认60),
打印位姿轨迹并统计稳定性(应钉在新盆地, 不再滑回旧盆地)。
容器内跑: python3 /home/mu/foxy_ws/bbs_hold_watch.py [秒=60]"""
import sys, time
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

rclpy.init()
n = Node("bbs_hold_watch")
buf = []
health = ["无"]

n.create_subscription(Odometry, "/loc/pose", lambda m: buf.append(
    (time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
     m.pose.pose.position.z, m.pose.pose.orientation)), 20)
n.create_subscription(String, "/loc/health", lambda m: health.__setitem__(0, m.data), 10)

t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.4)

if not buf:
    print("TIMEOUT: /loc/pose 无流!")
    sys.exit(1)
arr = np.array([(x, y, z) for _, x, y, z, _ in buf])
yaw = [np.degrees(2*np.arctan2(q.z, q.w)) for *_, q in buf]
print(f"[watch] {len(buf)} 个位姿样本, 最新健康: {health[0]}")
print(f"[watch] 首样本 t={buf[0][0]-t0:.0f}s: ({arr[0,0]:.3f},{arr[0,1]:.3f},{arr[0,2]:.3f}) yaw={yaw[0]:.1f}°")
print(f"[watch] 末样本 t={buf[-1][0]-t0:.0f}s: ({arr[-1,0]:.3f},{arr[-1,1]:.3f},{arr[-1,2]:.3f}) yaw={yaw[-1]:.1f}°")
print(f"[watch] 中段60%范围: x[{np.percentile(arr[:,0],20):.3f},{np.percentile(arr[:,0],80):.3f}] "
      f"y[{np.percentile(arr[:,1],20):.3f},{np.percentile(arr[:,1],80):.3f}] "
      f"yaw[{np.percentile(yaw,20):.1f}°,{np.percentile(yaw,80):.1f}°]")
print(f"[watch] 首末位移: {np.linalg.norm(arr[-1,:2]-arr[0,:2])*1000:.0f} mm (新盆地理应 <~50mm)")
