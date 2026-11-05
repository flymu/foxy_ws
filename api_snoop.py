#!/usr/bin/env python3
"""api_snoop.py — 总线旁听：把遥控器操作翻译成 RPC 流（跑在 Orin，纯订阅零风险）

遥控器是唯一已知能驱动狗的路径。它到底发什么 RPC、带什么 priority/lease/参数？
订阅面：/api/{sport,motion_switcher,robot_state,obstacles_avoid}/request|response
        /servicestate /servicestateactivate /multiplestate /wirelesscontroller
流量控制：每个 api 前 3 条全打，之后每 api 至多 0.5s 一条；结束打计数汇总。
用法: python3 api_snoop.py [秒数=90]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from unitree_api.msg import Request, Response
from unitree_go.msg import WirelessController

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0

NAMES = {
    'sport': {1001: 'Damp', 1002: 'BalanceStand', 1003: 'StopMove',
              1004: 'StandUp', 1005: 'StandDown', 1006: 'RecoveryStand',
              1008: 'Move', 1015: 'SpeedLevel', 1027: 'SwitchJoystick'},
    'motion_switcher': {1001: 'CheckMode', 1002: 'SelectMode',
                        1003: 'ReleaseMode', 1004: 'SetSilent',
                        1005: 'GetSilent'},
    'robot_state': {1001: 'ServiceSwitch', 1002: 'ReportFreq',
                    1003: 'ServiceList'},
    'obstacles_avoid': {1001: 'SwitchSet', 1002: 'SwitchGet', 1003: 'Move',
                        1004: 'UseRemoteCmd'},
}

qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=20)

rclpy.init()
n = Node("api_snoop")
t0 = time.monotonic()
stats = {}        # 'req:sport.Move' -> count
last_print = {}   # name -> monotonic of last printed line
last_keys = [None]
last_wc_t = [0.0]


def api_name(svc, api_id):
    return '%s.%s' % (svc, NAMES.get(svc, {}).get(api_id, '#%s' % api_id))


def rate_ok(name):
    now = time.monotonic()
    if now - last_print.get(name, 0.0) >= 0.5:
        last_print[name] = now
        return True
    return False


def on_req(svc):
    def h(m):
        api_id = m.header.identity.api_id
        name = api_name(svc, api_id)
        stats['req:' + name] = stats.get('req:' + name, 0) + 1
        if stats['req:' + name] <= 3 or rate_ok('req:' + name):
            print('[%6.1fs] REQ  %-22s api=%s lease=%s prio=%s noreply=%s p=%r' % (
                time.monotonic() - t0, name, api_id, m.header.lease.id,
                m.header.policy.priority, m.header.policy.noreply,
                m.parameter[:150]), flush=True)
    return h


def on_resp(svc):
    def h(m):
        api_id = m.header.identity.api_id
        name = api_name(svc, api_id)
        stats['resp:' + name] = stats.get('resp:' + name, 0) + 1
        if stats['resp:' + name] <= 3 or rate_ok('resp:' + name):
            print('[%6.1fs] RESP %-22s api=%s code=%s data=%r' % (
                time.monotonic() - t0, name, api_id, m.header.status.code,
                m.data[:150]), flush=True)
    return h


for svc in ('sport', 'motion_switcher', 'robot_state', 'obstacles_avoid'):
    n.create_subscription(Request, '/api/%s/request' % svc, on_req(svc), qb)
    n.create_subscription(Response, '/api/%s/response' % svc, on_resp(svc), qb)


def on_str(topic):
    def h(m):
        print('[%6.1fs] %s %r' % (time.monotonic() - t0, topic, m.data[:200]),
              flush=True)
    return h


for t in ('/servicestate', '/servicestateactivate', '/multiplestate'):
    n.create_subscription(String, t, on_str(t), qb)


def on_wc(m):
    if m.keys != last_keys[0]:
        last_keys[0] = m.keys
        print('[%6.1fs] 遥控按键 keys=0x%04x lx=%.2f ly=%.2f rx=%.2f ry=%.2f' % (
            time.monotonic() - t0, m.keys, m.lx, m.ly, m.rx, m.ry), flush=True)
    elif any(abs(v) > 0.1 for v in (m.lx, m.ly, m.rx, m.ry)):
        if time.monotonic() - last_wc_t[0] > 1.0:
            last_wc_t[0] = time.monotonic()
            print('[%6.1fs] 遥控摇杆 lx=%.2f ly=%.2f rx=%.2f ry=%.2f keys=0x%04x' % (
                time.monotonic() - t0, m.lx, m.ly, m.rx, m.ry, m.keys), flush=True)


n.create_subscription(WirelessController, '/wirelesscontroller', on_wc, qb)

print('旁听 %ss 开始...（请在窗口内用遥控器操作狗）' % DUR, flush=True)
end = time.monotonic() + DUR
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.5)
print('=== 旁听结束, 汇总 ===', flush=True)
for k in sorted(stats):
    print('  %-32s ×%d' % (k, stats[k]), flush=True)
n.destroy_node()
rclpy.shutdown()