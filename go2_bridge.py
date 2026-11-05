#!/usr/bin/env python3
"""
go2_bridge.py v4 — Go2 控制桥（跑在 Orin Nano 上）
工作站 WiFi DDS ──► rclpy 本桥 ──► unitree_sdk2py RPC（eth1 狗内部网）──► Go2 sport 服务

2026-10-22 实测结论:
- 狗新固件默认跑 mcf 模式(App/AI 控制), 此时 sport RPC 一律被拒(code=-1)
- 正确进入开发者模式的路径: motion_switcher 服务 SelectMode("sport")
  (robot_state 的 ServiceSwitch 会 5201 并回退; /api/sport/request 话题在 mcf 下不受理)
- SelectMode("mcf") 交还给 App/遥控

2026-11-05 G1（评审 R4）: 停车与速度的槽位竞争修复——仲裁逻辑抽到
bridge_core.CmdArbiter（hi 指令通道永远先于速度、'stop' 即刻锁存 halt、
看门狗瞬态不锁存），本文件只做 ROS 接线 + RPC 执行。

话题（ROS2 domain 0, 走 WiFi）:
  订阅 /go2/cmd_vel  geometry_msgs/Twist   连续速度指令（限幅 |vx|,|vy|≤0.5m/s, |vyaw|≤1rad/s）
  订阅 /go2/cmd      std_msgs/String       balance/damp/stop/standup/standdown/recovery/
                                            resume/switchjoystick_on/off/devon/devoff
  发布 /go2/status   std_msgs/String       心跳 1Hz: 当前模式 + halt/丢速计数 + 最近指令结果

安全:
  - 看门狗: 连续速度模式 0.5s 无新 cmd_vel → 自动 Move(0,0,0)（瞬态，不锁存）
  - stop 锁存: 'stop' 后速度一律丢弃，直到显式 'resume'（或姿态/模式类指令
    balance/standup/recovery/devon——操作员主动接管即解除）。评审 R4 实锤：
    旧版 stop 和速度共用一个可覆盖槽位，持续速度流能立刻覆盖停车。
  - 速度先限幅; 所有 RPC 结果码进 status, 出错即告警日志
"""
import json
import threading
import time

import bridge_core
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.rpc.client import Client

MAX_VXY = 0.5     # m/s 速度限幅
MAX_VYAW = 1.0    # rad/s 角速度限幅
WATCHDOG_S = 0.5  # 速度指令失联超时
MODE_CHECK_S = 5.0

MCF = 'mcf'    # App/AI 模式(默认)
SPORT = 'sport'  # 开发者模式(SDK 控制)


def clamp(v, lim):
    return max(-lim, min(lim, v))


class MotionSwitcherClient(Client):
    """motion_switcher 服务（api 与 B2 版一致: 1001 CheckMode / 1002 SelectMode / 1003 ReleaseMode）"""

    def __init__(self):
        super().__init__('motion_switcher', False)

    def Init(self):
        self._SetApiVerson('1.0.0.1')
        for api_id in (1001, 1002, 1003):
            self._RegistApi(api_id, 0)

    def CheckMode(self):
        return self._Call(1001, '{}')

    def SelectMode(self, name):
        return self._Call(1002, json.dumps({'name': name}))


class Go2Bridge(Node):
    def __init__(self):
        super().__init__('go2_bridge')

        # ---- SDK2 RPC 侧: eth1, domain 0 ----
        ChannelFactoryInitialize(0, 'eth1')

        self.sport = SportClient()
        self.sport.Init()
        self.ms = MotionSwitcherClient()
        self.ms.Init()

        self.mode = self._check_mode()
        self.get_logger().info('motion mode: %s' % self.mode)

        # ---- 指令仲裁 + RPC 工作线程（G1/R4：仲裁在 bridge_core，锁在本类）----
        self._cv = threading.Condition()
        self.arb = bridge_core.CmdArbiter(WATCHDOG_S)
        self._stop_thread = False
        self._last_result = 'idle'
        self._last_mode_check = time.monotonic()
        threading.Thread(target=self._work, daemon=True).start()

        # cmd_vel 用 BEST_EFFORT：上游 veto 输出 BE（RELIABLE 重传机制在
        # 执行器被雷达反序列化拖住时会楔死流）；丢拍由 0.5s 断流看门狗兜底
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Twist, '/go2/cmd_vel', self._on_cmd_vel, qb)
        self.create_subscription(String, '/go2/cmd', self._on_cmd, 10)
        self.pub_status = self.create_publisher(String, '/go2/status', 10)
        self.create_timer(1.0, self._pub_status)

        self.get_logger().info('go2_bridge v3 ready: /go2/cmd_vel /go2/cmd → SDK2 RPC (eth1)')

    def _check_mode(self):
        try:
            code, data = self.ms.CheckMode()
            if code == 0:
                return json.loads(data).get('name', '?')
            return 'err%d' % code
        except Exception as e:  # noqa: BLE001
            return 'exc:%s' % e

    # ---------- ROS 回调：只入队 ----------
    def _on_cmd_vel(self, msg):
        vx = clamp(msg.linear.x, MAX_VXY)
        vy = clamp(msg.linear.y, MAX_VXY)
        vyaw = clamp(msg.angular.z, MAX_VYAW)
        with self._cv:
            self.arb.push_vel(vx, vy, vyaw, time.monotonic())
            self._cv.notify()

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd not in CMDS:
            self.get_logger().warn('未知指令: %r（可用: %s）' % (msg.data, ','.join(CMDS)))
            return
        with self._cv:
            self.arb.push_cmd(cmd)   # 'stop' 在此即刻锁存 halt（不等 RPC 执行）
            self._cv.notify()

    # ---------- 工作线程：串行执行 RPC ----------
    def _work(self):
        while not self._stop_thread:
            with self._cv:
                item = self.arb.next_item(time.monotonic())
            if item is None:
                # 空闲时周期性刷新模式（RPC 最多阻塞1s, 不进 executor 线程）
                if time.monotonic() - self._last_mode_check > MODE_CHECK_S:
                    self.mode = self._check_mode()
                    self._last_mode_check = time.monotonic()
                time.sleep(0.02)
                continue

            try:
                self._exec(item)
            except Exception as e:  # noqa: BLE001
                self._last_result = 'rpc exc: %s' % e
                self.get_logger().warn(self._last_result)

    def _exec(self, item):
        ts = time.strftime('%H:%M:%S')
        if item[0] == 'move':
            code = self.sport.Move(item[1], item[2], item[3])  # noreply, 0=已发
            self._last_result = 'move(%.2f,%.2f,%.2f) code=%d' % (item[1], item[2], item[3], code)
            return
        cmd = item[1]

        if cmd == 'resume':
            # 纯解锁（halt 已在入队时刻解除），无 RPC——只是给 status 一个回执
            self._last_result = 'resume ack @%s' % ts
            return
        if cmd == 'devon':
            code = self.ms.SelectMode(SPORT)[0]
            self._last_result = 'SelectMode(sport) code=%d @%s' % (code, ts)
        elif cmd == 'devoff':
            code = self.ms.SelectMode(MCF)[0]
            self._last_result = 'SelectMode(mcf) code=%d @%s' % (code, ts)
        else:
            fn = {
                'balance': lambda: self.sport.BalanceStand(),
                'damp': lambda: self.sport.Damp(),
                'stop': lambda: self.sport.StopMove(),
                'standup': lambda: self.sport.StandUp(),
                'standdown': lambda: self.sport.StandDown(),
                'recovery': lambda: self.sport.RecoveryStand(),
                'switchjoystick_on': lambda: self.sport.SwitchJoystick(True),
                'switchjoystick_off': lambda: self.sport.SwitchJoystick(False),
            }[cmd]
            code = fn()
            self._last_result = '%s code=%d @%s' % (cmd, code, ts)
        if code != 0:
            self.get_logger().warn(self._last_result)

    # ---------- 状态心跳 ----------
    def _pub_status(self):
        s = String()
        s.data = 'mode=%s | moving=%s | halt=%s drop=%d | %s' % (
            self.mode, self.arb.moving, self.arb.halted, self.arb.dropped_vel,
            self._last_result)
        self.pub_status.publish(s)

    def shutdown_stop(self):
        self._stop_thread = True
        try:
            self.sport.StopMove()
        except Exception:  # noqa: BLE001
            pass


CMDS = ('balance', 'damp', 'stop', 'resume', 'standup', 'standdown', 'recovery',
        'switchjoystick_on', 'switchjoystick_off', 'devon', 'devoff')


def main():
    rclpy.init()
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
