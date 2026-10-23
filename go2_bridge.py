#!/usr/bin/env python3
"""
go2_bridge.py v3 — Go2 控制桥（跑在 Orin Nano 上）
工作站 WiFi DDS ──► rclpy 本桥 ──► unitree_sdk2py RPC（eth1 狗内部网）──► Go2 sport 服务

2026-10-22 实测结论:
- 狗新固件默认跑 mcf 模式(App/AI 控制), 此时 sport RPC 一律被拒(code=-1)
- 正确进入开发者模式的路径: motion_switcher 服务 SelectMode("sport")
  (robot_state 的 ServiceSwitch 会 5201 并回退; /api/sport/request 话题在 mcf 下不受理)
- SelectMode("mcf") 交还给 App/遥控

话题（ROS2 domain 0, 走 WiFi）:
  订阅 /go2/cmd_vel  geometry_msgs/Twist   连续速度指令（限幅 |vx|,|vy|≤0.5m/s, |vyaw|≤1rad/s）
  订阅 /go2/cmd      std_msgs/String       balance/damp/stop/standup/standdown/recovery/
                                            switchjoystick_on/off/devon/devoff
  发布 /go2/status   std_msgs/String       心跳 1Hz: 当前模式 + 最近指令结果

安全:
  - 看门狗: 连续速度模式 0.5s 无新 cmd_vel → 自动 Move(0,0,0)
  - 速度先限幅; 所有 RPC 结果码进 status, 出错即告警日志
"""
import json
import threading
import time

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

        # ---- 指令队列 + RPC 工作线程 ----
        self._cv = threading.Condition()
        self._pending = None
        self._moving = False
        self._last_vel_t = 0.0
        self._stop_thread = False
        self._last_result = 'idle'
        self._last_mode_check = time.monotonic()
        threading.Thread(target=self._work, daemon=True).start()

        self.create_subscription(Twist, '/go2/cmd_vel', self._on_cmd_vel, 10)
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
            self._pending = ('move', vx, vy, vyaw)
            self._last_vel_t = time.monotonic()
            self._cv.notify()

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd not in CMDS:
            self.get_logger().warn('未知指令: %r（可用: %s）' % (msg.data, ','.join(CMDS)))
            return
        with self._cv:
            self._pending = ('cmd', cmd)
            self._cv.notify()

    # ---------- 工作线程：串行执行 RPC ----------
    def _work(self):
        while not self._stop_thread:
            item = None
            with self._cv:
                if self._pending is not None:
                    item = self._pending
                    self._pending = None
                elif self._moving and (time.monotonic() - self._last_vel_t) > WATCHDOG_S:
                    self._moving = False
                    item = ('move', 0.0, 0.0, 0.0)  # 看门狗自动停车
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
            self._moving = True
            self._last_result = 'move(%.2f,%.2f,%.2f) code=%d' % (item[1], item[2], item[3], code)
            return
        self._moving = False  # 单条指令接管, 退出连续速度模式
        cmd = item[1]

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
        s.data = 'mode=%s | moving=%s | %s' % (self.mode, self._moving, self._last_result)
        self.pub_status.publish(s)

    def shutdown_stop(self):
        self._stop_thread = True
        try:
            self.sport.StopMove()
        except Exception:  # noqa: BLE001
            pass


CMDS = ('balance', 'damp', 'stop', 'standup', 'standdown', 'recovery',
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
