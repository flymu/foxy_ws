#!/usr/bin/env python3
"""veto_calib_probe.py — 离线标定 Mid360→狗身静态几何（obstacle_veto 的先验来源）

对 capture_frames.py 采集的静止段原始帧（传感器系），用"双候选先验 +
百分位取地板 + SVD 精修"拟合地面，输出：
  - n_up：传感器系里的"重力向上"单位向量（46° 斜装 → 落在 x-z 平面）
  - h_sensor：传感器离地高
  - sensor-x 仰角（判定 46° 是朝前上还是前下）
  - 频带 [0.05,0.75]m 内最近点距离分布 → 校核 veto 环/扇区参数基线
用法（foxy-live 容器内）:
  source /opt/ros/foxy/setup.bash && source ~/foxy_ws/install_x86/setup.bash
  python3 ~/foxy_ws/veto_calib_probe.py [帧npz] [先验倾角deg]
"""
import sys

import numpy as np

NPZ = sys.argv[1] if len(sys.argv) > 1 else "/tmp/veto_frames.npz"
PITCH = np.radians(float(sys.argv[2]) if len(sys.argv) > 2 else 46.0)

CANDIDATES = [
    ("+x倾", np.array([np.sin(PITCH), 0.0, np.cos(PITCH)])),
    ("-x倾", np.array([-np.sin(PITCH), 0.0, np.cos(PITCH)])),
]


def fit_ground(P, prior, tol=0.08):
    """以 prior 为初猜在点云 P 里找地板：h=P@prior 低分位取地板候选，SVD 精修。"""
    h = P @ prior
    h_lo = np.percentile(h, 5)
    sel = np.abs(h - h_lo) < tol
    if sel.sum() < 200:
        return None
    Q = P[sel]
    for _ in range(3):
        c = Q.mean(axis=0)
        _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
        n = vt[2]
        if n @ prior < 0:
            n = -n
        sel = np.abs((P - c) @ n) < tol
        if sel.sum() < 200:
            return None
        Q = P[sel]
    return dict(n=n, h=float(-c @ n), inliers=int(sel.sum()),
                dev_deg=float(np.degrees(np.arccos(min(1.0, abs(n @ prior))))),
                centroid=c)


def main():
    d = np.load(NPZ)
    P = d["pts"].astype(np.float64)
    counts = d["counts"]
    print("载入 %d 帧 / %d 点, 每帧 min/avg/max = %d/%.0f/%d"
          % (len(counts), len(P), counts.min(), counts.mean(), counts.max()))

    print("---- 地面拟合（双候选, 先验 %.0f°）----" % np.degrees(PITCH))
    best = None
    for name, prior in CANDIDATES:
        r = fit_ground(P, prior)
        if r is None:
            print("[%s] 拟合失败(候选点不足)" % name)
            continue
        ok = r["dev_deg"] < 15 and 0.15 <= r["h"] <= 1.2
        print("[%s] n=[%.4f,%.4f,%.4f] h_sensor=%.3fm 内点=%d 与先验偏=%.1f° %s"
              % (name, r["n"][0], r["n"][1], r["n"][2], r["h"], r["inliers"],
                 r["dev_deg"], "✓通过" if ok else "✗拒绝"))
        if ok and best is None:
            best = r
    if best is None:
        print("!! 两候选均未过验证")
        return 1
    n, hg = best["n"], best["h"]

    fx = np.array([1.0, 0.0, 0.0])
    elev_x = float(np.degrees(np.arcsin(np.clip(fx @ n, -1, 1))))
    print("传感器x轴相对水平面仰角 = %+.1f°（正=朝上）" % elev_x)
    print("FOV垂直边相对水平面: %+.1f° / %+.1f°" % (elev_x - 7.0, elev_x + 52.0))
    if elev_x - 7.0 < 0:
        blind = hg / max(0.01, -np.tan(np.radians(elev_x - 7.0)))
        print("下盲锥半径(低于下FOV边的东西看不到) ≈ %.2fm" % blind)
    else:
        print("!! 下FOV边在水平面之上：看不到地面——先验倾角或拟合有问题")

    print("---- 频带内点分布（veto 参数校核）----")
    h_pts = (P - best["centroid"]) @ n  # 离地高
    band = (h_pts > 0.05) & (h_pts < 0.75)
    print("总点 %d, 频带内 %d (%.0f%%)" % (len(P), band.sum(), 100.0 * band.sum() / len(P)))
    B = P[band]
    if len(B):
        # 水平距离：投影到地面后离传感器水平位置
        vh = B - (B @ n)[:, None] * n
        dh = np.hypot(vh[:, 0], vh[:, 1])
        for r in (0.55, 1.0, 1.5, 2.0):
            print("  水平距<%.2fm 的频带点: %d" % (r, (dh < r).sum()))
        if (dh < 1.5).sum():
            print("  1.5m内最近频带点: %.2fm" % dh[dh < 1.5].min())
        # 方位分布（0°=传感器x=狗前向）
        ang = np.degrees(np.arctan2(vh[:, 1], vh[:, 0]))
        near = dh < 1.2
        if near.sum():
            for lo in range(-180, 180, 30):
                cnt = ((ang[near] >= lo) & (ang[near] < lo + 30)).sum()
                print("  方位[%4d,%4d)°: %d %s" % (lo, lo + 30, cnt, "#" * min(40, cnt)))
    print("---- 建议参数（obstacle_veto.py prior_up / h_sensor）----")
    print("prior_up = [%.4f, %.4f, %.4f]" % (n[0], n[1], n[2]))
    print("h_sensor = %.3f" % hg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
