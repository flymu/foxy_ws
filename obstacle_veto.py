#!/usr/bin/env python3
"""obstacle_veto.py — 独立避障否决层（最后防线，2026-11-03 P0-S）

背景：撞车事故核实——规划器障碍图唯一数据源是实时点云，但传感器病时静默冻结、
定位游走时障碍图错位、EMERGENCY 循环蠕行，且狗自带 AI 避障被 sport RPC 旁路。
本节点是"最后一道闸"：独立于 LIO/定位/规划器，只看原始雷达 + 静态安装几何。

设计要点（全部有数据依据，见 veto_calib_probe.py 标定）：
  1. 只依赖 Mid360→狗身静态几何：n_up=[-0.733,0.0001,0.680]（46° 前下斜装，
     h_sensor=0.486m）。传感器x轴=狗前向（建图轨迹直行段验证 φ≈0）。
  2. 狗体自回波只有一个紧簇：桅杆座/背在传感器正下方（传感器系
     x∈[-0.04,+0.14] y≈0 z∈[-0.17,+0.08]，即离地高 0.38-0.52m；URDF 证实
     传感器地面投影≈躯干中心，躯干全长 0.376m 恰对应可见顶面跨
     s∈[-0.15,+0.15]）——用传感器系小方盒掩掉，不搞体素掩膜（时间采样法
     会混入起点环境回波，51% 消除率不够）。侧腹/腿被自身遮挡无回波，
     正前向环境从 s≈+0.6 起（建图起点数据）。
  3. 检查几何：离地高 h∈[band_lo,band_hi] 的点，前方 ±60° 扇区 <sector_r
     拦截，侧后 <ring_r 拦截；拦截只消速度"朝向障碍"的分量（保留沿墙滑行
     与脱离分量），原地转向永远放行（逃生通道）。极近(<0.3m)才全消平动。
     G1（2026-11-05，评审 R1/R2）定案的决策语义在 veto_core.py：
       R1 dh=正交水平基下真距离（旧式前向压缩 ×0.68 已修）→ 半径就是真米数；
       R2 只抑制不放大——消解过的残速低于起步阈值(v_min_walk)→归零（保守
          停车），未消解的指令原样放行；起步/停步地板归上游 speed_adapter.py；
       R3 采集时效双闸：stamp 年龄超限或不前进 → 不计新鲜（防旧帧/重复帧
          无限续命）；回放模式用 -p stamp_check:=false 显式关门。
  4. 断供即熔断：>cloud_timeout 无有效云（点数/散布/戳三判据，防驱动病空云
     假新鲜）→ 全零输出。无数据=不动。节点死=cmd流断=桥看门狗停车，天然失效安全。
  5. Mid360 内置 IMU 重力对齐（/livox/imu，g 单位，静止读数=+g 向上）：
     开机静止窗直接给出法线初值——趴着也给得对，不依赖地板可见性；
     地板 SVD 精修当"锚"（定 h、定坡道上的真地板方向），只在 IMU 判
     "准静止"时采帧/精修（"开机=静止"推广成"任何静止时刻"）；锚后 IMU
     增量跟踪倾斜（坡道/步态俯仰实时补偿）；倾斜累计>5°（坐/趴/搬动）或
     静置>120s 自动重修。IMU 缺席/断流优雅回退纯雷达行为。
  6. /veto/cmd "recalib" 手动重标、"prior" 回退先验。

接线（D1+D2 落地 2026-11-05，本节点驻 Orin；speed_adapter 2026-11-05 G1 加入）：
  scan_planner(WS) -r /cmd_vel:=/cmd_vel_intent → speed_adapter(WS，起步地板)
  →(WiFi 小报文) /cmd_vel_raw → 本节点(Orin) → /go2/cmd_vel(本地链路) → Orin 桥。
  点云来源 = /livox/points2（Orin 上 livox_relay 的输出：同为原始雷达点，
  仅 CustomMsg→PointCloud2 格式转换，中继与驱动同机走 SHM 不占 WiFi）。
  /livox/lidar 的 WiFi 订阅者从此只剩 point_lio 一个（D1 写侧棘轮根治）。
离线测试 remap：--ros-args -r /go2/cmd_vel:=/veto/cmd_out -p stamp_check:=false
用法（Orin 上，unitree 用户）:
  bash /home/unitree/patrol_safety_start.sh    # relay + veto 一键启动
"""
import math
import sys
import time
from collections import deque

import numpy as np
import rclpy
import veto_core
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ObstacleVeto(Node):
    def __init__(self):
        super().__init__("obstacle_veto")
        # ---- 参数（全部可 -p 覆盖）----
        p = lambda n, d: self.declare_parameter(n, d).value
        # 决策参数打包给 veto_core（G1：决策逻辑无 ROS 化，可离线回归）
        self.prm = veto_core.Params(
            band=(p("band_lo", 0.20), p("band_hi", 0.75)),
            self_half=(p("self_half_x", 0.25), p("self_half_y", 0.15)),
            self_z=(p("self_z_lo", -0.30), p("self_z_hi", 0.35)),
            # band_lo=0.20：2026-11-03 实弹诊断——场地坡度+标定残差让地板壳漏进
            # h 0.05-0.15（拦截点 51% 是地板，净空处也永远 BLOCK），0.20 能盖住
            # 1.25m 近场内 ~5° 坡度+标定误差；代价是 <20cm 低矮物不可见
            # （11-03 撞的桌/沙发均 >30cm，可接受）
            sector_half=math.radians(p("sector_half_deg", 60.0)),
            # R1 语义：sector_r 现在是真·水平米。旧度量前向压缩 ×0.68，实测
            # sector_r=1.0 等效 1.47m 真距（狗 0.9m 外对无害物刹停的过度保守
            # 一半来自这里）。修正后 1.0 就是 1.0。高速巡航时应改为速度相关。
            sector_r=p("sector_r", 1.0),
            ring_r=p("ring_r", 0.55),
            touch_r=p("touch_r", 0.30),                    # 极近全消平动
            near_keep=p("near_keep", 1.25),                # 入库水平距上限
            vx_dead=p("vx_dead", 0.02),                    # 分量消解死区
            # 残速地板（11-05 蠕行实弹）：狗起步阈值实测 0.20~0.25，残速
            # 0.19~0.22 狗原地罚站。G1 定案：消解过的残速低于此→归零；
            # 抬升越权已删（R2），滑行由上游 speed_adapter 给足速度实现
            v_min_walk=p("v_min_walk", 0.25),
            stamp_check=p("stamp_check", True),            # R3 采集时效闸
            stamp_max_age=p("stamp_max_age", 2.5),         # 健康基线 0.15s
        )
        # ---- 安装几何（狗身轴心离传感器的水平距离：URDF 实测传感器地面投影
        # ≈躯干中心（可见躯干顶面跨 s∈[-0.15,+0.15] 恰为 0.376m 躯干全长），取外轮廓中心 +0.05）
        self.geom = veto_core.Geom(veto_core.PRIOR_N, veto_core.PRIOR_H,
                                   body_along=p("body_along", 0.05),
                                   body_lat=p("body_lat", 0.0))
        self.window_s = p("cloud_window", 0.5)
        self.timeout_s = p("cloud_timeout", 1.0)
        self.min_pts = p("min_pts", 2000)                    # 有效帧下限(健康~2万)
        self.min_spread = p("min_spread", 1.0)               # 有效帧散布下限(m)
        self.calib_after_s = p("calib_after_s", 6.0)
        # ---- IMU 重力对齐（Mid360 内置 IMU，/livox/imu ~181Hz，g 单位）----
        self.imu_quiet_g = p("imu_quiet_g", 0.05)        # |‖a‖-1g|<此判准静止
        self.imu_tau = p("imu_tau", 2.0)                  # 重力EMA时间常数(s)
        self.imu_track = p("imu_track", True)             # 锚定后 IMU 增量跟踪倾斜
        self.posture_rearm_deg = p("posture_rearm_deg", 5.0)  # 倾斜累计超此→重修
        self.recheck_s = p("recheck_s", 120.0)            # 静置复检周期(s)

        # ---- 状态 ----
        self.win = deque()            # (t, (M,2) float32: dh, az)
        self.last_fresh_t = None
        self.last_stamp = None        # R3：最近一次通过时效闸的采集戳
        self.last_stamp_warn = 0.0
        self.raw = deque()            # 标定用原始帧（仅准静止时采集）
        self.t_first = time.monotonic()
        self.calib_done = False
        self.calib_attempts = 0   # 重试上限，防坏地面无限刷日志
        self.next_calib_t = self.t_first + self.calib_after_s
        self.calib_frames = p("calib_frames", 55)   # 收满静止帧即精修(≈5.5s@10Hz)
        # ---- IMU 重力对齐状态 ----
        self.g = self.geom.n.copy()   # 重力"向上"方向估计（传感器系）
        self.g_at_fit = self.geom.n.copy()  # 上次锚定（精修/对齐）时的 g
        self.last_fit_t = self.t_first
        self.imu_n = 0
        self.quiet_n = 0
        self.a_mag = 0.0
        self.imu_quiet = None         # None=尚无IMU；True/False=准静止与否
        self.last_imu_t = None
        self.imu_sign = None          # 符号约定（实测 +1：读数即向上）
        self.boot_aligned = False
        self.calib_src = "prior"
        self.stats = {"block_sector": 0, "block_ring": 0, "pass": 0,
                      "stale": 0, "frames": 0, "fresh_frames": 0,
                      "stamp_rej": 0}
        self.last_state = "INIT"
        self.last_blk = None

        # ---- 话题 ----
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        # 速度指令深度1（终于和注释一致了，评审 R3 顺带修）：纯"最新指令赢"，
        # 丢旧无所谓；深度10 会把过期指令排队重放，RELIABLE 更有楔死前科
        qcmd = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        qr = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
        # D2 修复：吃中继的 PointCloud2（np.frombuffer 零拷贝），不再是
        # rclpy ctypes 逐点反序列化 ~190ms/帧 的 CustomMsg 直订受害者
        self.create_subscription(PointCloud2, "/livox/points2", self.on_cloud, qb)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, qb)
        self.create_subscription(Twist, "/cmd_vel_raw", self.on_cmd, qcmd)
        self.create_subscription(String, "/veto/cmd", self.on_cmd_string, qr)
        # BE 输出：桥 0.5s 断流看门狗兜底，丢一拍无妨；RELIABLE 有楔死风险
        self.pub_cmd = self.create_publisher(
            Twist, "/go2/cmd_vel",
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.pub_status = self.create_publisher(String, "/veto/status", 5)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            "obstacle_veto 就绪(G1): n=[%.3f,%.3f,%.3f] h=%.3f, 前扇±%.0f°/%.2fm(真距) "
            "环%.2fm, 超时%.1fs, 戳闸%s(%.1fs), 残速地板%.2f(低于即归零,不抬升)" % (
                self.geom.n[0], self.geom.n[1], self.geom.n[2], self.geom.h,
                math.degrees(self.prm.sector_half), self.prm.sector_r,
                self.prm.ring_r, self.timeout_s,
                "开" if self.prm.stamp_check else "关", self.prm.stamp_max_age,
                self.prm.v_min_walk))

    # ---------------- 回调 ----------------
    def on_cloud(self, m):
        # D2 修复核心：PointCloud2 的 data 是 array.array('B')，np.frombuffer
        # 走 buffer 协议零拷贝（旧版 CustomMsg 每帧 ~190ms ctypes 逐点反序列
        # 化把所有 python 订阅者钉死在 5.3Hz；此路彻底绕开）
        self.stats["frames"] += 1
        # R3 采集时效闸：刚收到≠刚采集。旧帧延迟到达/重复帧无限续命是评审
        # 实锤的盲区；驱动冻结(戳不前进)同样拒。回放模式显式 -p stamp_check:=false
        stamp_s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ok, why = veto_core.stamp_ok(stamp_s, self.last_stamp, time.time(),
                                     self.prm)
        if not ok:
            self.stats["stamp_rej"] += 1
            now = time.monotonic()
            if now - self.last_stamp_warn > 10.0:
                self.last_stamp_warn = now
                self.get_logger().warn(
                    "[R3] 拒帧(%s): 戳龄 %.2fs, 连拒 %d — 驱动冻结/时钟域异常?"
                    % (why, time.time() - stamp_s, self.stats["stamp_rej"]))
            return
        self.last_stamp = stamp_s
        if m.width < self.min_pts or m.point_step != 16:
            return  # 空云/病云/异格式不计新鲜
        arr = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
        P = arr[:, :3]
        if np.ptp(P[:, 0]) + np.ptp(P[:, 1]) < self.min_spread:
            return  # 塌缩成团的病云不计新鲜
        self.stats["fresh_frames"] += 1
        now = time.monotonic()
        self.last_fresh_t = now
        sl = veto_core.ingest(P, self.geom, self.prm)
        if len(sl):
            self.win.append((now, sl))
        # 建议保留窗口
        while self.win and self.win[0][0] < now - self.window_s:
            self.win.popleft()
        # 标定采集：IMU 在手时只在"准静止"采帧（"开机=静止"推广成"任何静止
        # 时刻"——步态/搬动中的帧不采，地板拟合才不被运动抹脏）；IMU 断流/
        # 缺席则回退纯雷达时序。收满即精修；失败 15s 后重采重试，至多 6 次
        imu_fresh = self.last_imu_t is not None and now - self.last_imu_t < 3.0
        quiet_ok = (self.imu_quiet is None) or (not imu_fresh) or self.imu_quiet
        if (not self.calib_done and self.calib_attempts < 6
                and now >= self.next_calib_t and quiet_ok):
            self.raw.append(P)
            if len(self.raw) > 80:
                self.raw.popleft()
            if len(self.raw) >= self.calib_frames:
                if self._calibrate("auto"):
                    self.calib_done = True
                else:
                    self.calib_attempts += 1
                    self.next_calib_t = now + 15.0
                    self.raw.clear()

    def on_cmd(self, m):
        vx, vy, wz = float(m.linear.x), float(m.linear.y), float(m.angular.z)
        now = time.monotonic()
        out = Twist()
        if self.last_fresh_t is None or now - self.last_fresh_t > self.timeout_s:
            state, blk = "STALE", None
            vx = vy = wz = 0.0  # 熔断：无有效云=禁走（设计4，撞车事故教训①）
            self.stats["stale"] += 1
        else:
            while self.win and self.win[0][0] < now - self.window_s:
                self.win.popleft()
            if self.win:
                pts = np.vstack([s for _, s in self.win])
            else:
                pts = np.zeros((0, 2), np.float32)
            # 决策在 veto_core（G1 无 ROS 核心，可回归）：R1 真距离 + R2 只抑制
            # 不放大（残速低于起步阈值→归零，不再抬升）
            vx, vy, state, blk = veto_core.apply_veto(vx, vy, pts, self.prm)
            if state == "PASS":
                self.stats["pass"] += 1
            else:
                self.stats["block_ring" if "RING" in state else "block_sector"] += 1
        self.last_state, self.last_blk = state, blk
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.pub_cmd.publish(out)

    def on_cmd_string(self, m):
        if m.data == "recalib":
            self.raw.clear()
            self.calib_done = False       # 否则自动标定永不触发，采的帧没人消费
            self.calib_attempts = 0
            self.next_calib_t = time.monotonic()
            self.get_logger().info("重标定：采 %d 帧静止帧后精修..." % self.calib_frames)
        elif m.data == "prior":
            self._set_ground(veto_core.PRIOR_N, veto_core.PRIOR_H, "prior")
            self.get_logger().info("已回退先验")

    # ---------------- IMU 重力对齐 ----------------
    def on_imu(self, m):
        a = np.array([m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z])
        self.imu_n += 1
        now = time.monotonic()
        dt = 0.005 if self.last_imu_t is None else min(0.5, now - self.last_imu_t)
        self.last_imu_t = now
        am = float(np.linalg.norm(a))
        self.a_mag = am
        # Mid360 内置 IMU 输出 g 单位（实测静止 |a|≈0.997）：模长贴 1g 判准静止
        self.imu_quiet = abs(am - 1.0) < self.imu_quiet_g
        if not self.imu_quiet:
            return  # 步态/搬动等动态中不更新重力方向
        if self.imu_sign is None:
            self.imu_sign = 1.0 if a @ self.geom.n > 0 else -1.0
        up = self.imu_sign * a / max(am, 1e-6)
        self.quiet_n += 1
        # EMA：对齐前快收敛（50帧≈0.3s 残差<1%），对齐后慢跟踪
        tau = 0.05 if not self.boot_aligned else self.imu_tau
        alpha = dt / (tau + dt)
        g_new = self.g + alpha * (up - self.g)
        g_new /= np.linalg.norm(g_new)
        # 锚后增量跟踪：地板锚定的法线随重力增量旋转
        # （地板定绝对，IMU 跟变化——坡道/步态俯仰实时补偿）
        if self.imu_track and self.calib_src != "prior":
            R = self._rot_between(self.g, g_new)
            if R is not None:
                self._apply_ground(R @ self.geom.n)
        self.g = g_new
        # 开机静止窗重力对齐（启动=静止假设）：趴着也给得出正确法线，
        # 不用等 6s 地板拟合，也不怕地板被挡
        if not self.boot_aligned and self.quiet_n >= 50 and now - self.t_first > 2.0:
            self.boot_aligned = True
            dev = math.degrees(math.acos(min(1.0, abs(self.g @ veto_core.PRIOR_N))))
            if dev < 15.0:
                self._set_ground(self.g, veto_core.PRIOR_H, "imu-boot")
                self.get_logger().info(
                    "[imu-boot] 重力对齐: 与先验夹角 %.1f°，法线即刻可用，"
                    "h 暂用先验 %.3f" % (dev, veto_core.PRIOR_H))
            else:
                self.get_logger().warn(
                    "[imu-boot] 重力与先验差 %.1f°>15°（坡道/被搬动?），"
                    "保持先验等地板精修" % dev)
            return
        # 姿态自愈：有锚才谈得上"变了"。倾斜累计超阈值（坐/趴/搬动）或
        # 静置超周期 → 待静止自动重修；6 连败后也周期性再给一轮
        # （狗趴一阵再站起，回到站立域就能自愈，不用手动 recalib）
        tilt = math.degrees(math.acos(min(1.0, abs(self.g @ self.g_at_fit))))
        since_fit = now - self.last_fit_t
        if self.calib_src != "prior":
            if self.calib_done and tilt > self.posture_rearm_deg:
                self._rearm(now, "IMU 倾斜累计 %.1f°>%.0f°（坐/趴/搬动）"
                            % (tilt, self.posture_rearm_deg))
            elif self.calib_done and since_fit > self.recheck_s:
                self._rearm(now, "静置复检（上次锚定 %.0fs 前）" % since_fit)
            elif self.calib_attempts >= 6 and since_fit > self.recheck_s:
                self._rearm(now, "精修 6 连败后定期重试（等狗回站立域）")

    def _rearm(self, now, why):
        self.calib_done = False
        self.calib_attempts = 0
        self.raw.clear()
        self.next_calib_t = now + 1.0
        self.last_fit_t = now
        self.get_logger().warn("%s：待静止后自动重标定" % why)

    @staticmethod
    def _rot_between(u, v):
        """把单位向量 u 转到 v 的最小旋转（Rodrigues）。反平行返回 None。"""
        c = float(np.clip(u @ v, -1.0, 1.0))
        w = np.cross(u, v)
        s = float(np.linalg.norm(w))
        if s < 1e-9:
            return None
        k = w / s
        K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        th = math.atan2(s, c)
        return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)

    # ---------------- 标定 ----------------
    def _calibrate(self, tag):
        if len(self.raw) < 5:
            self.get_logger().warn("[%s] 标定帧不足(%d)，保持先验" % (tag, len(self.raw)))
            return False
        P = np.vstack(list(self.raw))
        prior = self.geom.n
        h = P @ prior
        sel = np.abs(h - np.percentile(h, 5)) < 0.08
        if sel.sum() < 500:
            self.get_logger().warn("[%s] 地板候选点不足，保持先验" % tag)
            return False
        Q = P[sel]
        for _ in range(3):
            c = Q.mean(axis=0)
            _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
            nn = vt[2]
            if nn @ prior < 0:
                nn = -nn
            sel = np.abs((P - c) @ nn) < 0.08
            if sel.sum() < 500:
                self.get_logger().warn("[%s] 精修发散，保持先验" % tag)
                return False
            Q = P[sel]
        hh = float(-c @ nn)
        dev = math.degrees(math.acos(min(1.0, abs(nn @ prior))))
        # 校验门收紧到"站立姿态"合理域：坐/趴姿态下传感器掉 ~0.18m，
        # 误锁会把 0.38m 以下障碍全弄瞎（2026-11-03 实弹 h=0.309 教训）
        if dev < 12 and 0.35 <= hh <= 0.68:
            self._set_ground(nn, hh, "refit-%s" % tag)
            self.get_logger().info("[%s] 地板精修: dev=%.1f° h=%.3fm（先验 %.3f）"
                                   % (tag, dev, hh, veto_core.PRIOR_H))
            return True
        self.get_logger().warn("[%s] 精修未过校验(dev=%.1f° h=%.2f)，保持先验，15s 后重采重试"
                               % (tag, dev, hh))
        return False

    def _apply_ground(self, nn):
        """轻量更新几何（n/f/l/体心）——IMU 增量跟踪用，不清窗口、不动锚。"""
        self.geom.rebase(nn)

    def _set_ground(self, nn, hh, src):
        self._apply_ground(nn)
        self.geom.h = float(hh)
        self.calib_src = src
        self.g_at_fit = self.g.copy()   # 锚定当前重力：姿态变化检测的基准
        self.last_fit_t = time.monotonic()
        self.win.clear()  # 几何变了，窗口数据重灌

    # ---------------- 状态 ----------------
    def publish_status(self):
        now = time.monotonic()
        age = -1.0 if self.last_fresh_t is None else (now - self.last_fresh_t) * 1000
        s = self.stats
        if self.imu_n == 0:
            imu, q, slope = "none", "-", -1.0
        elif self.last_imu_t is not None and now - self.last_imu_t < 3.0:
            imu = "ok"
            q = str(int(bool(self.imu_quiet)))
            slope = math.degrees(math.acos(min(1.0, abs(self.g @ self.geom.n))))
        else:
            imu, q, slope = "dead", "-", -1.0
        msg = String()
        msg.data = ("state=%s age_ms=%d win=%d blk=%s calib=%s "
                    "frames=%d fresh=%d pass=%d sector=%d ring=%d stale=%d "
                    "stamprej=%d imu=%s a=%.2f q=%s slope=%.1f try=%d/6"
                    % (self.last_state, age, len(self.win),
                       ("dh=%.2f az=%.0f" % self.last_blk) if self.last_blk else "none",
                       self.calib_src, s["frames"], s["fresh_frames"], s["pass"],
                       s["block_sector"], s["block_ring"], s["stale"],
                       s["stamp_rej"],
                       imu, self.a_mag, q, slope, self.calib_attempts))
        self.pub_status.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = ObstacleVeto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()