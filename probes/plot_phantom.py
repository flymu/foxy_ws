#!/usr/bin/env python3
"""plot_phantom.py — 幽灵簇诊断图：狗顶视图 + 拦截区 + 簇高亮（转前/转后 两版）
用法: python3 plot_phantom.py 转前.npz 转后.npz 输出.png
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

N_UP = np.array([-0.7330, 0.0001, 0.6802]); N_UP /= np.linalg.norm(N_UP)
H = 0.486
BAND = (0.20, 0.75)
SELF_BOX = (0.25, 0.15, -0.30, 0.35)
NEAR_KEEP = 1.25
SECTOR_HALF = np.deg2rad(60.0)

# 类别色（dataviz 固定次序）: 蓝=右簇, 橙=左簇, 绿=前扇区带内点; 灰=上下文
C_RIGHT = '#3b6fd4'
C_LEFT = '#e08214'
C_FRONT = '#2e9e5b'
C_CTX = '#c9c9c9'

f_ax = np.array([1.0, 0.0, 0.0]); f_ax = f_ax - (f_ax @ N_UP) * N_UP; f_ax /= np.linalg.norm(f_ax)
l_ax = np.cross(N_UP, f_ax)
body_c = 0.05 * f_ax


def load(npz):
    d = np.load(npz)
    return d['pts'].astype(np.float64), d['counts']


def frame_geo(P):
    """返回 (vh_x前, vh_y左, dh, az, h, band_mask)"""
    h = P @ N_UP + H
    band = (h > BAND[0]) & (h < BAND[1])
    band &= ~((np.abs(P[:, 0]) < SELF_BOX[0]) & (np.abs(P[:, 1]) < SELF_BOX[1])
              & (P[:, 2] > SELF_BOX[2]) & (P[:, 2] < SELF_BOX[3]))
    Q = P - body_c
    vh = Q - (Q @ N_UP)[:, None] * N_UP
    dh = np.hypot(vh[:, 0], vh[:, 1])
    az = np.arctan2(vh @ l_ax, vh @ f_ax)
    return vh[:, 0], vh[:, 1], dh, az, h, band


def main():
    npz_a, npz_b, out = sys.argv[1], sys.argv[2], sys.argv[3]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.4), sharex=True, sharey=True)
    fig.patch.set_facecolor('white')

    for ax, npz, title in zip(axes, (npz_a, npz_b), ('转前', '转后')):
        pts, counts = load(npz)
        off = 0
        ctx = []
        R_x, R_y, L_x, L_y, F_x, F_y = [], [], [], [], [], []
        for i, c in enumerate(counts):
            P = pts[off:off + c]; off += c
            vx, vy, dh, az, h, band = frame_geo(P)
            if i == 0:
                k = dh < 4.5
                ctx.append(np.column_stack([-vy[k], vx[k]]))  # 地图视角: 东=狗右
            m = band & (dh < NEAR_KEEP)
            if m.any():
                east, north = -vy[m], vx[m]
                a = az[m]
                kr = m & (a < np.deg2rad(-20)) & (dh < 1.1)[m] if m.any() else None
                # 逐类拆
                aa = a
                sel_r = aa < np.deg2rad(-20)
                sel_l = aa > np.deg2rad(20)
                sel_f = ~sel_r & ~sel_l
                sel_r &= dh[m] < 1.1
                sel_l &= dh[m] < 1.1
                R_x.append(east[sel_r]); R_y.append(north[sel_r])
                L_x.append(east[sel_l]); L_y.append(north[sel_l])
                F_x.append(east[sel_f]); F_y.append(north[sel_f])
        ctx = np.concatenate(ctx)
        ax.scatter(ctx[:, 0], ctx[:, 1], s=1.5, c=C_CTX, alpha=0.5, linewidths=0)
        for xx, yy, col, lab in ((F_x, F_y, C_FRONT, None),
                                 (L_x, L_y, C_LEFT, '左簇'),
                                 (R_x, R_y, C_RIGHT, '右簇(密)')):
            if xx:
                ax.scatter(np.concatenate(xx), np.concatenate(yy), s=3, c=col,
                           alpha=0.8, linewidths=0)
        # 狗 + 拦截区
        ax.add_patch(plt.Rectangle((-0.10, -0.19), 0.20, 0.38, fill=True,
                                   color='#333333', zorder=5))
        wedge = matplotlib.patches.Wedge((0, 0), 1.0, 90 - 60, 90 + 60,
                                         fill=True, color='#d62728', alpha=0.10, zorder=1)
        ax.add_patch(wedge)
        circ = plt.Circle((0, 0), 0.55, fill=False, color='#d62728',
                          alpha=0.35, linewidth=1.2, zorder=1)
        ax.add_patch(circ)
        ax.annotate('狗(朝上)', (0.02, 0.05), fontsize=9, color='white', zorder=6)
        if R_x:
            rx = np.concatenate(R_x); ry = np.concatenate(R_y)
            ax.annotate('右簇(密) %d点/帧\ndh≈%.2f h≈0.5-0.6' % (len(rx) // 40, np.median(np.hypot(rx, ry))),
                        (np.median(rx), np.median(ry)), textcoords='offset points',
                        xytext=(24, -6), fontsize=10, color=C_RIGHT,
                        arrowprops=dict(arrowcolor=C_RIGHT, arrowstyle='->'))
        if L_x:
            lx = np.concatenate(L_x); ly = np.concatenate(L_y)
            ax.annotate('左簇 %d点/帧' % (len(lx) // 40),
                        (np.median(lx), np.median(ly)), textcoords='offset points',
                        xytext=(18, 10), fontsize=10, color=C_LEFT,
                        arrowprops=dict(arrowcolor=C_LEFT, arrowstyle='->'))
        ax.set_title('%s（狗朝上，右=狗右侧）' % title, fontsize=12)
        ax.set_xlabel('狗右 → （米）'); ax.set_ylabel('← 狗左 / 狗前 → （米）')
        ax.set_xlim(-3.2, 3.2); ax.set_ylim(-3.2, 3.2)
        ax.set_aspect('equal')
        ax.grid(alpha=0.15, linewidth=0.5)
        ax.axhline(0, color='#888888', linewidth=0.6); ax.axvline(0, color='#888888', linewidth=0.6)

    fig.suptitle('幽灵障碍诊断：频带内(离地0.20-0.75m)近场点 — 蓝=右侧簇 橙=左侧簇 绿=前扇区 | 红楔/圈=veto拦截区',
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor='white')
    print('saved', out)


if __name__ == '__main__':
    main()
