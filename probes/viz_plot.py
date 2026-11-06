#!/usr/bin/env python3
"""viz_plot.py — 定位效果俯视图（宿主跑，matplotlib 出 PNG）

地图(灰) + 当前扫描(蓝, 已按 T_mo 变换进 map 系) + 定位轨迹(橙) + 当前位姿(橙箭头)。
扫描蓝点落进地图结构上 = 定位可信的视觉证据。

用法: python3 viz_plot.py <viz_cap.npz> [--map /tmp/bbs_selftest/map_clean.pcd]
                              [--out out.png] [--title "补充说明"]
"""
import sys

import matplotlib
matplotlib.use("Agg")
# CJK 字体: DejaVu Sans 无汉字字形(标题/图例会变豆腐块)，按宿主 fc-list :lang=zh 实测可用的顺序回退
matplotlib.rcParams["font.sans-serif"] = [
    "AR PL UMing CN", "AR PL UKai CN", "Noto Sans CJK SC",
    "WenQuanYi Zen Hei", "Droid Sans Fallback", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False  # CJK 字体常缺 U+2212 负号
import matplotlib.pyplot as plt
import numpy as np

MAP_GRAY = "#9AA5B1"
ACCENT_SCAN = "#2F6BD8"     # 蓝：当前扫描
ACCENT_POSE = "#E8833A"     # 橙：轨迹/位姿（蓝橙=色盲安全对比对）
GRID = "#E3E7EB"
INK = "#1F2429"


def read_xyz_pcd(path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode(errors="ignore").strip()
            k = line.split()[0] if line else ""
            header[k] = line
            if k == "DATA":
                break
        n = int(header["POINTS"].split()[1])
        return np.frombuffer(f.read(n * 12), dtype=np.float32).reshape(n, 3).astype(np.float64)


def main():
    npz_path = sys.argv[1]
    map_path = "/tmp/bbs_selftest/map_clean.pcd"
    out = None
    title_extra = ""
    args = sys.argv[2:]
    for i, a in enumerate(args):
        if a == "--map":
            map_path = args[i + 1]
        elif a == "--out":
            out = args[i + 1]
        elif a == "--title":
            title_extra = args[i + 1]
    d = np.load(npz_path, allow_pickle=True)
    scan_map = d["scan_map"]
    loc = d["loc_pose"]
    t_mo = d["t_mo"]
    path = d["path"]
    try:
        m = read_xyz_pcd(map_path)
    except Exception as e:
        print(f"[viz_plot] 地图读失败({e})，只画扫描")
        m = np.zeros((0, 3))
    yaw = np.degrees(np.arctan2(t_mo[1, 0], t_mo[0, 0])) % 360

    if out is None:
        out = npz_path.replace(".npz", ".png").replace("viz_cap_", "loc_viz_")

    fig, ax = plt.subplots(figsize=(11, 9), dpi=150)
    if len(m):
        ax.scatter(m[:, 0], m[:, 1], s=0.12, c=MAP_GRAY, alpha=0.35, linewidths=0,
                   rasterized=True, label=f"校园地图 ({len(m)//1000}k 点)")
    ax.scatter(scan_map[:, 0], scan_map[:, 1], s=1.6, c=ACCENT_SCAN, alpha=0.9,
               linewidths=0, rasterized=True, label=f"当前扫描·map系 ({len(scan_map)} 点)")
    if len(path):
        ax.plot(path[:, 0], path[:, 1], c=ACCENT_POSE, lw=2.0, alpha=0.85,
                label=f"定位轨迹 ({len(path)} 点)", zorder=5)
    # 位姿箭头（按轴范围缩放，保证可见）
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    L = 0.07 * min(x1 - x0, y1 - y0)
    ax.annotate("", xy=(loc[0] + L * np.cos(np.radians(yaw)), loc[1] + L * np.sin(np.radians(yaw))),
                xytext=(loc[0], loc[1]),
                arrowprops=dict(arrowstyle="-|>", color=ACCENT_POSE, lw=2.6, mutation_scale=18),
                zorder=6, label="当前位姿")
    ax.plot(loc[0], loc[1], "o", ms=5, c=ACCENT_POSE, zorder=7)

    ax.set_aspect("equal")
    ax.grid(True, color=GRID, lw=0.5)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=INK, labelsize=9)
    ax.set_xlabel("x / m", fontsize=10, color=INK)
    ax.set_ylabel("y / m", fontsize=10, color=INK)
    ax.set_title("Go2 定位效果·俯视  —  当前扫描(蓝)应严丝合缝落在地图结构上", fontsize=13,
                 color=INK, pad=12)
    import time
    sub = (f"位姿 map=({loc[0]:+.2f}, {loc[1]:+.2f}) m   yaw={yaw:.1f}°   "
           f"T_mo 平移=({t_mo[0, 3]:+.2f}, {t_mo[1, 3]:+.2f}, {t_mo[2, 3]:+.2f}) m"
           + (f"   {title_extra}" if title_extra else "")
           + f"   {time.strftime('%Y-%m-%d %H:%M:%S')}")
    ax.text(0.5, 1.015, sub, transform=ax.transAxes, ha="center", fontsize=9.5, color="#5B6470")
    leg = ax.legend(loc="upper right", fontsize=9, framealpha=0.9, edgecolor=GRID)
    for t in leg.get_texts():
        t.set_color(INK)
    fig.tight_layout()
    fig.savefig(out, facecolor="white", bbox_inches="tight")
    print(f"[viz_plot] 已存 {out}  (地图 {len(m)} 点 / 扫描 {len(scan_map)} 点 / "
          f"轨迹 {len(path)} 点)", flush=True)


if __name__ == "__main__":
    main()
