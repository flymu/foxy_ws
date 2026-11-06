#!/usr/bin/env python3
"""bbs_pose_verify.py — 候选 T_map_odom 的客观裁决: 把稠密扫描(odom系)按各候选位姿变换进地图系,
对清洗后地图做最近邻距离统计。中位/分位数最小者胜 — 与 BBS 的体素打分完全独立的验证。
宿主跑: python3 bbs_pose_verify.py
候选:
  A) BBS 稠密扫描结果 (0.15 全套配置)
  B) 当前 NDT 跟踪 T_mo (loc_state.txt)
  C) 11-05 信念 T_mb=(-1.109, 0.014, yaw 7.7°) 换算的 T_mo (需当前 /Odometry)
"""
import numpy as np
from scipy.spatial import cKDTree

MAP = "/tmp/bbs_selftest/map_clean.pcd"
SCAN = "/home/mu/maps/bbs_dense_scan.pcd"


def read_bin_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if line.startswith("DATA"):
                fmt = line.split()[1]
                break
            k, *v = line.split()
            hdr[k] = v
        n = int(hdr["POINTS"][0])
        assert fmt == "binary", fmt
        fields = hdr["FIELDS"]
        if fields[:3] == ["x", "y", "z"] and len(fields) == 3:
            arr = np.fromfile(f, dtype=np.float32, count=n * 3).reshape(n, 3).astype(np.float64)
        else:
            step = sum(int(s) for s in hdr["SIZE"])
            raw = np.fromfile(f, dtype=np.uint8, count=n * step).reshape(n, step)
            off = [sum(int(s) for s in hdr["SIZE"][:i]) for i in range(len(fields))]
            xi, yi = fields.index("x"), fields.index("y")
            zi = fields.index("z")
            arr = np.stack([raw[:, off[i]:off[i] + 4].copy().view(np.float32).ravel()
                            for i in (xi, yi, zi)], axis=1).astype(np.float64)
    return arr


def yaw_rot(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    R = np.eye(3)
    R[0, 0], R[0, 1], R[1, 0], R[1, 1] = c, -s, s, c
    return R


def quat_R(x, y, z, w):
    R = np.empty((3, 3))
    R[0, 0] = 1 - 2 * (y * y + z * z); R[0, 1] = 2 * (x * y - z * w); R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w);     R[1, 1] = 1 - 2 * (x * x + z * z); R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w);     R[2, 1] = 2 * (y * z + x * w);     R[2, 2] = 1 - 2 * (x * x + y * y)
    return R


# 当前 /Odometry (T_ob, odom←body): 探针实测
T_ob_R = quat_R(0.0028, 0.3364, 0.0001, 0.9417)   # 俯仰≈39.3° 桅杆倾斜
T_ob_t = np.array([0.0041, -0.002, 0.004])

# A) BBS 稠密结果 (bbs_pose 输出的 map<-odom 4x4)
A_R = np.array([
    [0.967462, -0.252366, 0.018142],
    [0.252322, 0.967632, 0.004732],
    [-0.018749, 0.000000, 0.999824]])
A_t = np.array([-1.200000, 0.000000, -0.300000])

# B) loc_state.txt 的 t_mo (行优先 3x4 前3列/第4列)
B_R = np.array([
    [0.998204, -0.0590215, -0.0102451],
    [0.0588299, 0.998104, -0.0180955],
    [0.0112937, 0.0174603, 0.999784]])
B_t = np.array([-0.922071, -0.153659, -0.178106])

# C) 11-05 信念: T_mb=(-1.109, 0.014, yaw 7.7°), z/rpy 沿用当前 T_mb
#    当前 T_mb = T_mo_B · T_ob → 取其 z/roll/pitch; T_mo_C = T_mb_C·T_ob⁻¹
mb_cur_R = B_R @ T_ob_R
mb_cur_t = B_R @ T_ob_t + B_t
# 从当前 T_mb 提取 z、roll、pitch, 换 x/y/yaw
z_cur = mb_cur_t[2]
roll_cur = np.arctan2(mb_cur_R[2, 1], mb_cur_R[2, 2])
pitch_cur = np.arctan2(-mb_cur_R[2, 0], np.hypot(mb_cur_R[2, 1], mb_cur_R[2, 2]))
C_mb_R = yaw_rot(7.7) @ np.array([  # R_z(yaw)·R_y(pitch)·R_x(roll)
    [np.cos(pitch_cur), 0, np.sin(pitch_cur)],
    [0, 1, 0],
    [-np.sin(pitch_cur), 0, np.cos(pitch_cur)]]) @ np.array([
    [1, 0, 0],
    [0, np.cos(roll_cur), -np.sin(roll_cur)],
    [0, np.sin(roll_cur), np.cos(roll_cur)]])
C_mb_t = np.array([-1.109, 0.014, z_cur])
T_ob_inv_R = T_ob_R.T
T_ob_inv_t = -T_ob_R.T @ T_ob_t
C_R = C_mb_R @ T_ob_inv_R
C_t = C_mb_R @ T_ob_inv_t + C_mb_t

print(f"[info] 当前 T_mb: t=({mb_cur_t[0]:.3f},{mb_cur_t[1]:.3f},{mb_cur_t[2]:.3f}) "
      f"rpy=({np.degrees(roll_cur):.1f},{np.degrees(pitch_cur):.1f},"
      f"{np.degrees(np.arctan2(mb_cur_R[1,0],mb_cur_R[0,0])):.1f})°")

mp = read_bin_pcd(MAP)
sp = read_bin_pcd(SCAN)
print(f"[load] map={len(mp)} scan={len(sp)}")
tree = cKDTree(mp)
sp_s = sp[::3]  # 66k 采样

print(f"{'候选':<4} {'中位':>8} {'75%':>8} {'90%':>8} {'<10cm':>7}  T_mo(t, yaw)")
for name, (R, t) in [("A BBS", (A_R, A_t)), ("B NDT", (B_R, B_t)), ("C 1105", (C_R, C_t))]:
    q = (sp_s @ R.T) + t
    d, _ = tree.query(q, workers=-1)
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f"{name:<4} {np.median(d):8.3f} {np.percentile(d,75):8.3f} "
          f"{np.percentile(d,90):8.3f} {(d<0.1).mean()*100:6.1f}%  "
          f"t=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) yaw={yaw:.2f}°")

# A 的局部微调扫描: 在 A 附近 ±0.3m/±5° 网格找 NN 中位最小者 (复核 A 是否真在谷底)
best = (np.median(d), A_t, A_R, "A")
grids = [(-0.2, -0.1, 0, 0.1, 0.2)]
for dx in grids[0]:
    for dy in grids[0]:
        for dyaw in (-4, -2, 0, 2, 4):
            Rg = yaw_rot(dyaw) @ A_R
            qg = (sp_s @ Rg.T) + (A_t + [dx, dy, 0])
            dg, _ = tree.query(qg, workers=-1)
            md = np.median(dg)
            if md < best[0]:
                best = (md, A_t + [dx, dy, 0], Rg, f"A+({dx},{dy},{dyaw}°)")
print(f"[refine] A 邻域最优: {best[3]} 中位={best[0]:.3f} t=({best[1][0]:.2f},{best[1][1]:.2f},{best[1][2]:.2f})")
