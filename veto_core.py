#!/usr/bin/env python3
"""veto_core.py — obstacle_veto 的纯决策核心（无 ROS 依赖，2026-11-05 G1）

为什么存在：G1 修复批要求"几何、速度、时效、停车优先级的无 ROS 回归"
（DEVELOPMENT_PLAN.md）。原实现把决策逻辑焊死在 rclpy Node 里，离开容器
没法测。本模块只依赖 numpy，obstacle_veto.py 负责接线，本模块负责对错。

G1 三处修复落点（评审 docs/REVIEW_2026-11-05.md）：
  R1 距离压缩：dh 原来算 hypot(vh.x, vh.y)——vh 已投到水平面但坐标仍在
     倾斜的传感器系里，前向被压缩 ×cos(47°)≈0.68（实测 0.62 vs 真值 0.91，
     胶带复核）。修正为 hypot(vh·f, vh·l)，正交水平基下各向同性。
     半径语义随之校正：sector_r=1.0 现在就是真 1.0m（旧代码实际 1.47m）。
  R2 提权违规：否决层原来把残速"抬回"v_min_walk——放大速度越权（残速
     0.21 被抬成 0.25 是输入的数倍）。修正为纯抑制：消解过的残速低于起步
     阈值 → 归零（保守停车）；未消解的指令原样放行。起步/停步地板归上游
     speed_adapter.py 管（适配在前、否决在后）。
  R3 时效盲区：原来"刚收到"当"刚采集"，旧帧/重复帧能无限续命。修正为
     双闸：采集年龄(now-stamp) + 时间戳严格前进（冻结/回退即拒）。
"""
import math

import numpy as np

# ---------------- 先验（veto_calib_probe.py 从 campus_map 静止段标定）----------------
PRIOR_N = np.array([-0.7330, 0.0001, 0.6802])
PRIOR_H = 0.486


class Geom:
    """安装几何：传感器系里的重力对齐狗身基（n=上, f=前, l=左）+ 体轴心。"""
    __slots__ = ("n", "f", "l", "body_c", "h")

    def __init__(self, n, h, body_along=0.05, body_lat=0.0):
        self.rebase(n)
        self.h = float(h)
        self.body_c = (body_along * self.f + body_lat * self.l)

    def rebase(self, n):
        """法线更新后重导 f/l/体心（IMU 增量跟踪用；h 不动）。

        体轴心的 along/lat 分量必须先在【旧基】下分解——回归测试抓过
        先换基再分解的顺序 bug（体心会冻结在旧向量上不随地面估计转）。
        """
        ba = bl = None
        if hasattr(self, "body_c"):
            ba = float(self.body_c @ self.f)
            bl = float(self.body_c @ self.l)
        self.n = np.asarray(n, float) / np.linalg.norm(n)
        f = np.array([1.0, 0.0, 0.0])
        f = f - (f @ self.n) * self.n
        self.f = f / np.linalg.norm(f)
        self.l = np.cross(self.n, self.f)
        if ba is not None:
            self.body_c = ba * self.f + bl * self.l

    def to_sensor(self, x, y=None, z=None):
        """狗身坐标 → 传感器系（与 veto_viz 同式）。标量三元组或 (N,3) 数组。"""
        if y is None:                      # (N,3) 整块输入
            pts = np.asarray(x, float)
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        x, y, z = np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)
        if x.ndim == 0:
            return x * self.f + y * self.l + (z - self.h) * self.n
        return (x[:, None] * self.f + y[:, None] * self.l
                + (z - self.h)[:, None] * self.n)


class Params:
    """决策参数（obstacle_veto 用 ROS 参数覆盖默认值后传入）。"""
    def __init__(self, **kw):
        self.band = (0.20, 0.75)              # 离地检查频带 (m)
        self.self_half = (0.25, 0.15)         # 自体掩膜半宽 (x,y)
        self.self_z = (-0.30, 0.35)           # 自体掩膜 z 范围（传感器系）
        self.sector_half = math.radians(60.0)
        self.sector_r = 1.0                   # 前扇区拦截半径（真·水平米，R1 语义）
        self.ring_r = 0.55                    # 侧后环拦截半径
        self.touch_r = 0.30                   # 极近全消平动
        self.near_keep = 1.25                 # 入库水平距上限
        self.vx_dead = 0.02                   # 分量消解死区
        self.v_min_walk = 0.25                # 狗起步阈值（残速低于此→归零，不抬升）
        self.stamp_check = True               # R3：采集时效/前进性闸门
        self.stamp_max_age = 2.5              # 采集年龄上限(s)；实测健康基线 0.15s
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError("未知参数: %s" % k)
            setattr(self, k, v)


def ingest(P, g, prm):
    """一帧点云（传感器系 N×3）→ 频带内、近场、非自体的 (dh, az)。

    dh=水平距离（R1 修正：正交水平基下度量，各向同性），
    az=方位角（rad，0=正前，+左）。空结果返回 (0,2)。
    """
    P = np.asarray(P, float)
    if len(P) == 0:
        return np.zeros((0, 2), np.float32)
    h = P @ g.n + g.h
    m = (h > prm.band[0]) & (h < prm.band[1])
    m &= ~((np.abs(P[:, 0]) < prm.self_half[0]) &
           (np.abs(P[:, 1]) < prm.self_half[1]) &
           (P[:, 2] > prm.self_z[0]) & (P[:, 2] < prm.self_z[1]))
    if not m.any():
        return np.zeros((0, 2), np.float32)
    rel = P[m] - g.body_c
    vh = rel - (rel @ g.n)[:, None] * g.n
    # R1 修正核心：在重力正交基下取水平距离。旧式 hypot(vh.x, vh.y) 把
    # 倾斜系 z 分量丢了——47° 斜装下前向压缩 ×0.68（实测 0.62 vs 真值 0.91）
    dh = np.hypot(vh @ g.f, vh @ g.l)
    keep = dh < prm.near_keep
    if not keep.any():
        return np.zeros((0, 2), np.float32)
    v = vh[keep]
    az = np.arctan2(v @ g.l, v @ g.f)
    return np.column_stack([dh[keep], az]).astype(np.float32)


def radius(az, prm):
    """方位相关的拦截半径：前扇区远、侧后环近。az 可为标量或数组(rad)。"""
    return np.where(np.abs(az) < prm.sector_half, prm.sector_r, prm.ring_r)


def apply_veto(vx, vy, pts, prm):
    """对速度指令执行否决。pts = 窗口内全部 (dh, az)。

    返回 (vx, vy, state, blk)：
      state: PASS / BLOCK_SECTOR / BLOCK_RING
      blk:   最近拦截者 (dh, az°)，无拦截为 None
    语义（G1 定案）：只抑制、不放大——输出速度模长 ≤ 输入。
    消解过的残速低于狗起步阈值时归零（保守停车）；未被消解触碰的指令
    原样放行（起步地板是上游 speed_adapter 的职责，否决层不越权）。
    角速度不在此函数管辖区（原地转向永远放行）。
    """
    vx, vy = float(vx), float(vy)
    if len(pts) == 0:
        return vx, vy, "PASS", None
    pts = np.asarray(pts, float)
    blockers = pts[pts[:, 0] < radius(pts[:, 1], prm)]
    if len(blockers) == 0:
        return vx, vy, "PASS", None
    blockers = blockers[np.argsort(blockers[:, 0])][:3]  # 最近3个
    state = "BLOCK_RING" if abs(blockers[0, 1]) > prm.sector_half \
            else "BLOCK_SECTOR"
    if blockers[0, 0] < prm.touch_r:
        vx = vy = 0.0  # 极近：全消平动
    else:
        cancelled = False
        for dh, az in blockers:
            u = np.array([math.cos(az), math.sin(az)])
            v = np.array([vx, vy])
            toward = v @ u
            if toward > prm.vx_dead:
                v = v - toward * u
                vx, vy = float(v[0]), float(v[1])
                cancelled = True
        if cancelled:
            # R2（G1）：残速低于起步阈值 → 归零（旧版抬回 v_min_walk 是放大，
            # 越权；滑行需要上游给足速度让残速留在阈值之上）。后退残速同样归零
            # ——宁停不倒车。
            if math.hypot(vx, vy) < prm.v_min_walk:
                vx = vy = 0.0
    blk = (float(blockers[0, 0]), float(math.degrees(blockers[0, 1])))
    return vx, vy, state, blk


def stamp_ok(stamp, last_stamp, now, prm):
    """R3 采集时效闸门。stamp/last_stamp/now 都是同一时钟域的秒。

    返回 (ok, reason)。拒绝理由：'age'（采集年龄超限——延迟到达的旧帧
    不能刷新有效期）、'frozen'（时间戳不前进——驱动冻结/重复帧回放）。
    回放模式（bag 旧时间戳）用 prm.stamp_check=False 显式关门，别靠放宽
    年龄上限混过去。
    """
    if not prm.stamp_check:
        return True, "off"
    if last_stamp is not None and stamp <= last_stamp:
        return False, "frozen"
    if now - stamp > prm.stamp_max_age:
        return False, "age"
    return True, "ok"