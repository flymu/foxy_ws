#!/usr/bin/env python3
"""veto_selfmask_probe.py — 构建狗体自回波掩膜（obstacle_veto 依赖）

雷达在尾部桅杆顶、朝前下 47° 看，自己的背/头/桅杆在频带内产生 ~13 万点/50帧的
恒定回波。用"时间采样法"分离：走行段每隔 ~20s 抽一帧（狗动了、环境在传感器系里
平移），只有刚性连在狗身上的东西恒留同一批 5cm 体素里。

持久体素（≥80% 帧命中）+ 26 邻域膨胀（±5cm 抗行走抖动）→ 掩膜。
用静止段数据验证：掩膜应解释正前方 <1.2m 的频带点大头。
输出：veto_selfmask.npz (voxels Nx3 int32, vox=0.05, dilate=1)
用法（foxy-live 容器内）:
  python3 ~/foxy_ws/veto_selfmask_probe.py 行走采样.npz 静止段.npz [输出npz]
"""
import sys

import numpy as np

WALK = sys.argv[1] if len(sys.argv) > 1 else "/tmp/veto_walk.npz"
STILL = sys.argv[2] if len(sys.argv) > 2 else "/tmp/veto_frames.npz"
OUT = sys.argv[3] if len(sys.argv) > 3 else "/home/mu/foxy_ws/veto_selfmask.npz"

VOX = 0.05
PERSIST = 0.8
DILATE = 1

N_UP = np.array([-0.7330, 0.0001, 0.6802])  # veto_calib_probe 标定结果
H_SENSOR = 0.486
BODY_CENTER = np.array([0.20, 0.0, 0.22])  # 传感器系，狗身轴心（水平前移~0.3m）


def vox_ids(pts):
    return np.floor(pts / VOX).astype(np.int32)


def frame_voxels(counts, pts):
    """按帧切分返回每帧的唯一体素坐标列表。"""
    out, off = [], 0
    for c in counts:
        f = pts[off:off + c]
        off += c
        out.append(np.unique(vox_ids(f), axis=0))
    return out


def dilate(V):
    """V: (K,3) 体素坐标 → 26 邻域膨胀后去重。"""
    offs = [(i, j, k) for i in (-DILATE, 0, DILATE)
            for j in (-DILATE, 0, DILATE) for k in (-DILATE, 0, DILATE)]
    parts = [V + np.array(o, np.int32) for o in offs]
    return np.unique(np.vstack(parts), axis=0)


def apply_mask_keys(P, mask_vox):
    """P: (M,3) 点 → 返回命中掩膜的布尔数组。体素 key 编码进 int64 二分查找。"""
    ox, oy, oz = mask_vox.T
    keys = (ox.astype(np.int64) + 2048) * (1 << 42) + \
           (oy.astype(np.int64) + 2048) * (1 << 21) + (oz.astype(np.int64) + 2048)
    keys.sort()
    ix, iy, iz = vox_ids(P).T
    q = (ix.astype(np.int64) + 2048) * (1 << 42) + \
        (iy.astype(np.int64) + 2048) * (1 << 21) + (iz.astype(np.int64) + 2048)
    idx = np.searchsorted(keys, q)
    idx = np.minimum(idx, len(keys) - 1) if len(keys) else idx
    hit = np.zeros(len(P), bool) if len(keys) else np.zeros(len(P), bool)
    if len(keys):
        hit = keys[idx] == q
    return hit


def band_stats(P, hit_self):
    """掩膜前后的频带内近点统计。"""
    n = N_UP / np.linalg.norm(N_UP)
    h = P @ n + H_SENSOR
    band = (h > 0.05) & (h < 0.75)
    rel = P - BODY_CENTER
    vh = rel - (rel @ n)[:, None] * n
    dh = np.hypot(vh[:, 0], vh[:, 1])
    near = band & (dh < 1.2)
    return near.sum(), (near & ~hit_self).sum()


def main():
    dw = np.load(WALK)
    fws = frame_voxels(dw["counts"], dw["pts"])
    nf = len(fws)
    print("行走采样 %d 帧 / %d 点" % (nf, len(dw["pts"])))
    allv = np.vstack(fws)
    uv, cnt = np.unique(allv, axis=0, return_counts=True)
    thr = int(np.ceil(PERSIST * nf))
    pers = uv[cnt >= thr]
    print("体素命中≥%d/%d 的持久体素: %d" % (thr, nf, len(pers)))
    mask = dilate(pers) if len(pers) else pers
    print("膨胀 ±%d 体素后: %d" % (DILATE, len(mask)))
    if len(mask):
        mmin, mmax = mask.min(0) * VOX, mask.max(0) * VOX
        print("掩膜范围(传感器系 m): x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]"
              % (mmin[0], mmax[0], mmin[1], mmax[1], mmin[2], mmax[2]))

    ds = np.load(STILL)
    Ps = ds["pts"].astype(np.float64)
    hit = apply_mask_keys(Ps, mask) if len(mask) else np.zeros(len(Ps), bool)
    before, after = band_stats(Ps, hit)
    print("---- 验证（静止段 %d 点）----" % len(Ps))
    print("频带内 <1.2m 近点: 掩膜前 %d → 掩膜后 %d（消除 %.0f%%）"
          % (before, after, 100.0 * (before - after) / max(1, before)))
    # 掩膜后近点的方位分布（应不再有正前方巨型簇）
    n = N_UP / np.linalg.norm(N_UP)
    h = Ps @ n + H_SENSOR
    bandm = (h > 0.05) & (h < 0.75) & ~hit
    rel = Ps[bandm] - BODY_CENTER
    vh = rel - (rel @ n)[:, None] * n
    dh = np.hypot(vh[:, 0], vh[:, 1])
    near = dh < 1.2
    if near.sum():
        ang = np.degrees(np.arctan2(vh[near][:, 1], vh[near][:, 0]))
        print("掩膜后 <1.2m 近点 %d, 最近 %.2fm, 方位:" % (near.sum(), dh[near].min()))
        for lo in range(-180, 180, 30):
            c = int(((ang >= lo) & (ang < lo + 30)).sum())
            print("  [%4d,%4d)°: %d %s" % (lo, lo + 30, c, "#" * min(40, c)))
    np.savez(OUT, voxels=mask.astype(np.int32), vox=VOX, dilate=DILATE,
             n_up=N_UP, h_sensor=H_SENSOR)
    print("已存 %s" % OUT)


if __name__ == "__main__":
    main()
