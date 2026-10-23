#!/usr/bin/env python3
# plot_map.py — campus_map 建图结果俯视图（暗色表面，符合 dataviz 规范）
# 图层: 点云=蓝色单色渐变(高度z) 背景 + 轨迹=橙色线(2px, 深色描边环)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.lines import Line2D

# ---- 色板 token（dataviz reference palette, dark mode）----
SURFACE = '#1a1a19'; PAGE = '#0d0d0d'
INK = '#f0efec'; INK2 = '#c3c2b7'; MUTED = '#898781'
GRID = '#2c2c2a'; AXIS = '#383835'
TRAJ = '#d95926'          # categorical slot 2 (dark)
RAMP = ['#184f95', '#3987e5', '#86b6ef']   # 蓝色顺序渐变: 低->高

# ---- CJK 字体 ----
cjk = next((f.name for f in font_manager.fontManager.ttflist
            if any(k in f.name for k in ('Noto Sans CJK', 'WenQuanYi', 'Source Han'))), None)
plt.rcParams['font.family'] = [cjk or 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

# ---- 数据 ----
MAP = '/home/mu/maps/lio_1792684457_global_map.pcd'
with open(MAP, 'rb') as f:
    while True:
        line = f.readline().decode().strip()
        if line.startswith('POINTS'):
            n = int(line.split()[1])
        if line == 'DATA binary':
            break
    mp = np.fromfile(f, dtype=np.float32, count=n * 4).reshape(n, 4)
traj = np.loadtxt('/home/mu/maps/lio_1792684457_trajectory.txt', skiprows=1)
print('点云 %d 点, 轨迹 %d 位姿' % (len(mp), len(traj)))

# ---- 画布 ----
fig = plt.figure(figsize=(10, 7.5), facecolor=PAGE, dpi=200)
ax = fig.add_axes([0.06, 0.08, 0.76, 0.82])
ax.set_facecolor(SURFACE)
ax.grid(color=GRID, lw=0.6)
for s in ax.spines.values():
    s.set_color(AXIS)
ax.tick_params(colors=MUTED, labelsize=9)
ax.set_xlabel('x (m)', color=INK2)
ax.set_ylabel('y (m)', color=INK2)

zlo, zhi = -0.7, 2.2
cmap = LinearSegmentedColormap.from_list('height', RAMP)
sc = ax.scatter(mp[:, 0], mp[:, 1], c=mp[:, 2], cmap=cmap,
                norm=Normalize(zlo, zhi), s=1.2, alpha=0.45,
                linewidths=0, rasterized=True, zorder=2)

cb = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.015)
cb.set_label('点云高度 z (m)', color=INK2, fontsize=10)
cb.ax.tick_params(colors=MUTED, labelsize=9)
cb.outline.set_edgecolor(AXIS)

# 轨迹: 深色描边环 + 橙色主线
ax.plot(traj[:, 1], traj[:, 2], color=SURFACE, lw=3.4, zorder=5)
ax.plot(traj[:, 1], traj[:, 2], color=TRAJ, lw=2.0, zorder=6)

# 起点/终点: 8px+ 环形标记 + 直接标注（文字用 ink，不用系列色）
for (px, py), lab, dx, dy in [((traj[0, 1], traj[0, 2]), '起点', 10, -16),
                              ((traj[-1, 1], traj[-1, 2]), '终点', 12, 8)]:
    ax.scatter([px], [py], s=130, facecolor=INK, edgecolor=SURFACE, lw=2.0, zorder=7)
    ax.annotate(lab, (px, py), xytext=(dx, dy), textcoords='offset points',
                color=INK, fontsize=11, fontweight='bold')

legend_items = [
    Line2D([], [], color=TRAJ, lw=2, label='行走轨迹（24.9 m）'),
    Line2D([], [], marker='s', color='none', markerfacecolor='#3987e5',
           markersize=8, label='建图点云（颜色 = 高度）'),
]
ax.legend(handles=legend_items, loc='upper left', facecolor=PAGE,
          edgecolor=AXIS, labelcolor=INK2, fontsize=10, framealpha=0.9)

ax.set_title('campus_map 建图结果 · 俯视图（Point-LIO）', color=INK,
             fontsize=14, loc='left', pad=16)
fig.text(0.06, 0.925,
         '路程 24.9 m · 起终点闭合误差 0.27 m（1.1%）· 2258 帧 @10 Hz 零丢帧 · 地图 77.6 万点',
         color=INK2, fontsize=10)
ax.set_aspect('equal')

out = '/home/mu/maps/campus_map_topdown.png'
fig.savefig(out, facecolor=PAGE)
print('saved:', out)
