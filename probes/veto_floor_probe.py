#!/usr/bin/env python3
"""veto_floor_probe.py — 诊断：频带内"障碍"点是不是地板泄漏

用 veto 同款管线（先验 n + 实弹精修 h）处理实弹帧，回答三个问题：
  1. 频带点（h∈band）的计算高度分布——若大簇挤在 band_lo 附近 = 地板壳
  2. "拦截点"（dh<radius）里 h<0.20 的占比——若占大头 = 拦的全是地板
  3. 拦截点的 dh-az 图（az 之前被 status 打印丢掉了）
用法: python3 veto_floor_probe.py 帧npz [h_sensor]
"""
import sys

import numpy as np

NPZ = sys.argv[1] if len(sys.argv) > 1 else "/tmp/floor_diag.npz"
H_SENSOR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.506  # 实弹 auto6s 精修值
N_UP = np.array([-0.7330, 0.0001, 0.6802])
N_UP /= np.linalg.norm(N_UP)
BAND = (0.05, 0.75)
BODY_ALONG = 0.05
NEAR_KEEP = 1.25
SECTOR_HALF = np.deg2rad(60.0)
SECTOR_R, RING_R, TOUCH_R = 1.0, 0.55, 0.30

f = np.array([1.0, 0.0, 0.0])
f = f - (f @ N_UP) * N_UP
f /= np.linalg.norm(f)
l = np.cross(N_UP, f)
body_c = BODY_ALONG * f

d = np.load(NPZ)
pts, counts = d["pts"].astype(np.float64), d["counts"]
off = 0
all_band, all_blk = [], []
for c in counts:
    P = pts[off:off + c]
    off += c
    h = P @ N_UP + H_SENSOR
    m = (h > BAND[0]) & (h < BAND[1])
    m &= ~((np.abs(P[:, 0]) < 0.25) & (np.abs(P[:, 1]) < 0.15)
           & (P[:, 2] > -0.30) & (P[:, 2] < 0.35))
    Q = P[m] - body_c
    vh = Q - (Q @ N_UP)[:, None] * N_UP
    dh = np.hypot(vh[:, 0], vh[:, 1])
    az = np.arctan2(vh @ l, vh @ f)
    k = dh < NEAR_KEEP
    hh, dd, aa = h[m][k], dh[k], az[k]
    all_band.append(hh)
    r = np.where(np.abs(aa) < SECTOR_HALF, SECTOR_R, RING_R)
    blk = dd < r
    all_blk.append(np.column_stack([dd[blk], np.degrees(aa[blk]), hh[blk]]))

H = np.concatenate(all_band)
B = np.vstack(all_blk)
print("帧数 %d, 频带内近点(可用) %d, 拦截点 %d" % (len(counts), len(H), len(B)))
print("\n== 频带点计算高度直方图(5cm bins, 0-0.5m) ==")
hist, edges = np.histogram(H, bins=np.arange(0.0, 0.51, 0.05))
for cnt, lo in zip(hist, edges[:-1]):
    bar = "#" * min(50, int(cnt / max(1, hist.max()) * 50))
    print("  h %.2f-%.2f: %7d %s" % (lo, lo + 0.05, cnt, bar))
print("\n== 拦截点高度分布 ==")
for lo in (0.0, 0.1, 0.2, 0.3, 0.5):
    n = int(((B[:, 2] >= lo) & (B[:, 2] < lo + (0.1 if lo < 0.3 else 0.25))).sum())
    print("  h %.2f-%.2f: %d (%.0f%%)" % (
        lo, lo + (0.1 if lo < 0.3 else 0.25), n, 100.0 * n / max(1, len(B))))
low = int((B[:, 2] < 0.20).sum())
print("  → 拦截点中 h<0.20m 占 %.0f%%  (%s)" % (
    100.0 * low / max(1, len(B)), "地板泄漏实锤" if low > 0.5 * len(B) else "不像地板"))
print("\n== 拦截点 dh-az (h<0.20 的低点, 每帧平均) ==")
lowb = B[B[:, 2] < 0.20]
if len(lowb):
    for lo in range(-180, 180, 30):
        sel = lowb[(lowb[:, 1] >= lo) & (lowb[:, 1] < lo + 30)]
        if len(sel):
            print("  az [%4d,%4d)°: dh %.2f-%.2f 中位 %.2f, 密度 %d/帧"
                  % (lo, lo + 30, sel[:, 0].min(), sel[:, 0].max(),
                     np.median(sel[:, 0]), len(sel) // len(counts)))
