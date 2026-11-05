#!/usr/bin/env python3
"""bridge_core.py — go2_bridge 的指令仲裁核心（无 ROS/SDK 依赖，2026-11-05 G1）

R4 修复（docs/REVIEW_2026-11-05.md）：停车指令和速度指令原来共用一个可覆盖
槽位——工作线程取走停车前若下一帧速度先到，停车就被顶掉；即使 StopMove 已
调用，持续速度流也能立刻重启运动。纯复现：入队 ('cmd','stop') 后紧跟一帧
速度，槽里只剩 ('move',...)。

本类把仲裁拆成两条通道 + 一个锁存：
  hi 通道  ：/go2/cmd 单条指令（stop/balance/devon/...），FIFO，最多 8 条，
             永远先于速度通道出队——停车不会被速度顶掉。
  vel 通道 ：/go2/cmd_vel 最新一条赢（latest-slot），harness 本来就是这语义。
  halt 锁存：'stop' 即刻锁存（入队时刻就生效，不等 RPC 执行完）；
             锁存期间速度一律丢弃并计数。恢复靠显式指令：'resume'（新增，
             纯解锁无 RPC）或姿态/模式类指令（balance/standup/recovery/devon
             ——操作员主动下姿态指令即视为接管解除）。
  看门狗  ：速度断流自动零速停车，瞬态不锁存（规划器重规划暂停 >0.5s 后
             恢复是正常工况，不能因此冻死）。

线程约定：本类不带锁，调用方（go2_bridge）须在同一把锁内调用全部方法。
mock 测试见 test_veto_regression.py 的 R4 组。
"""
from collections import deque

# 'stop' 之外这些指令解除 halt（操作员主动接管姿态/模式 = 解除停车锁存）
RESUME_CMDS = ("resume", "balance", "standup", "recovery", "devon")


class CmdArbiter:
    def __init__(self, watchdog_s=0.5, hi_depth=8):
        self.watchdog_s = float(watchdog_s)
        self._hi = deque(maxlen=hi_depth)  # 单条指令 FIFO，防爆队列截断最旧
        self._vel = None                   # 最新速度槽（latest-slot）
        self._halted = False
        self._moving = False
        self._last_vel_t = None
        self.dropped_vel = 0               # halt 期间丢弃的速度条数（状态可见）

    # ---- 入队（回调侧，锁内调用）----
    def push_vel(self, vx, vy, vyaw, now):
        if self._halted:
            self.dropped_vel += 1
            return
        self._vel = ("move", vx, vy, vyaw)
        self._last_vel_t = now

    def push_cmd(self, cmd):
        if cmd == "stop":
            self._halted = True
            self._vel = None               # 锁存同时清掉在途速度
        elif cmd in RESUME_CMDS:
            self._halted = False
        self._hi.append(("cmd", cmd))

    # ---- 出队（工作线程侧，锁内调用）----
    def next_item(self, now):
        """取下一条待执行项；无则 None。顺序：hi → vel → 看门狗零速。"""
        if self._hi:
            item = self._hi.popleft()
            self._moving = False           # 单条指令接管，退出连续速度模式
            if item[1] != "resume":
                self._vel = None           # 姿态/停车接管：作废排队中的旧速度
            return item                    # （resume 只解锁，不清其后新速度）
        if self._vel is not None:
            item = self._vel
            self._vel = None
            self._moving = True
            return item
        if self._moving and now - self._last_vel_t > self.watchdog_s:
            self._moving = False           # 瞬态停车，不锁存
            return ("move", 0.0, 0.0, 0.0)
        return None

    # ---- 状态 ----
    @property
    def halted(self):
        return self._halted

    @property
    def moving(self):
        return self._moving