"""控制台冒烟探针（无狗环境，配 synth_publisher.py 用）：一行一个 PASS/FAIL 断言。

覆盖：
  REST   /api/status /api/stacks /api/mapping/presets /api/stacks/{name}/log 404
  WS     mapping_reset / mapping 状态字段 / 二进制帧格式与体素连续性
  遥控   10Hz 指令落到 /cmd_vel_intent、看门狗 0.5s 自动零速、
         急停锁定、解除、规划器在场时 busy 拒绝（互斥门）
用法（容器内，已 source ROS，PYTHONPATH 含 pydeps）：
    python3 /home/mu/foxy_ws/patrol_console/tests/ws_probe.py
"""
import asyncio
import json
import struct
import threading
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

BASE = 'http://127.0.0.1:8000'
WS_URL = 'ws://127.0.0.1:8000/ws'

results = []


def check(name, cond, detail=''):
    results.append((name, bool(cond)))
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{detail}]' if detail else ''), flush=True)


def rest(path, method='GET'):
    req = urllib.request.Request(BASE + path, method=method, data=b'' if method == 'POST' else None)
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status, json.loads(r.read())


def main():
    import websockets

    rclpy.init()
    node = Node('ws_probe')
    echoed = []           # (t, v, w) 收到的 /cmd_vel_intent
    node.create_subscription(Twist, '/cmd_vel_intent', lambda m: echoed.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    # ---------------- REST ----------------
    _, st = rest('/api/status')
    check('status: type', st.get('type') == 'status')
    check('status: stacks 有 point_lio', any(s['name'] == 'point_lio' for s in st.get('stacks', [])))
    check('status: resources', 'cpu' in st.get('resources', {}))
    check('status: topics 列表', len(st.get('topics', [])) >= 5)
    check('status: paths', 'runs' in st.get('paths', {}))
    _, stacks = rest('/api/stacks')
    check('stacks: 唯一白名单进程', len(stacks) == 1 and stacks[0]['name'] == 'point_lio'
          and stacks[0]['state'] == 'idle')
    _, presets = rest('/api/mapping/presets')
    check('presets: 空列表', presets == [])
    _, log = rest('/api/stacks/point_lio/log?lines=5')
    check('stack log 路由', 'text' in log)
    try:
        rest('/api/stacks/nope/log')
        check('未知栈返回 404', False)
    except urllib.error.HTTPError as e:
        code = json.loads(e.read()).get('detail', {}).get('code')
        check('未知栈返回 404', e.code == 404 and code == 'stack_not_found', code)
    with urllib.request.urlopen(BASE + '/', timeout=5) as r:
        check('静态页 index.html', r.status == 200 and b'<div id="app"' in r.read())

    # ---------------- WebSocket + 数据流 ----------------
    async def ws_run(ws):
        await ws.send(json.dumps({'type': 'subscribe', 'streams': ['mapping']}))
        deadline = time.monotonic() + 15
        got_reset, mapping, frames = None, None, []
        offsets = {}
        # publishers 计数来自控制台 2s 一次的图查询，刚起时会先看到 0；等它收敛
        def pubs_ok(m):
            return bool(m) and m['publishers']['scan'] == 1 and m['publishers']['odom'] == 1
        while time.monotonic() < deadline and (got_reset is None or len(frames) < 6 or mapping is None
                                               or not mapping.get('initialized') or not pubs_ok(mapping)):
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if isinstance(msg, bytes):
                (hlen,) = struct.unpack_from('<I', msg, 0)
                head = json.loads(msg[4:4 + hlen])
                body = msg[4 + hlen:]
                check_len = head['count'] * 12 == len(body)
                if not check_len:
                    check('二进制帧长度', False, f"{head['type']} 声明 {head['count']*12} 实际 {len(body)}")
                frames.append((head, check_len))
                if head['type'] in ('map', 'est', 'gt'):
                    off = head['offset']
                    prev = offsets.get((head['type'], head['epoch']))
                    if prev is not None and prev != off:
                        check('体素帧 offset 连续', False, f"{head['type']} {prev}->{off}")
                    offsets[(head['type'], head['epoch'])] = off + head['count']
            else:
                m = json.loads(msg)
                if m.get('type') == 'mapping_reset':
                    got_reset = m['epoch']
                elif m.get('type') == 'mapping':
                    mapping = m
        check('mapping_reset 先到', got_reset is not None)
        check('mapping 状态字段', mapping is not None
              and mapping['voxel'] == 0.1
              and mapping['stacks']['slam'] == 'point_lio'
              and mapping['topics']['scan'] == '/cloud_registered'
              and mapping['slam_options'][0]['algorithm'] == 'Point-LIO', '')
        check('mapping: initialized（里程计活着）', bool(mapping and mapping.get('initialized')))
        check('mapping: 单一发布者', pubs_ok(mapping), str(mapping and mapping['publishers']))
        check('mapping: save 空闲 / waypoints 空', mapping and mapping['save'] == {'state': 'idle'}
              and mapping['waypoints'] is None)
        check('二进制帧 ≥6 且长度全对', len(frames) >= 6 and all(ok for _, ok in frames),
              f"{len(frames)} 帧, 类型 {sorted(set(h['type'] for h, _ in frames))}")
        check('体素帧 offset 连续', len(offsets) >= 1)

    async def teleop_run(ws):
        # 12 条 10Hz 遥控指令 → /cmd_vel_intent 上应看到一串 v≈0.3
        echoed.clear()
        for _ in range(12):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.1}))
            await asyncio.sleep(0.1)
        await asyncio.sleep(0.3)
        nonzero = [(t, v, w) for t, v, w in echoed if abs(v) > 0.01]
        check('遥控: ≥8 条非零速度直达 /cmd_vel_intent', len(nonzero) >= 8, f"{len(nonzero)} 条")
        check('遥控: 数值经限幅透传', nonzero and all(abs(v - 0.3) < 0.01 and abs(w - 0.1) < 0.01
                                                for _, v, w in nonzero))
        # 停发 → 0.5s 看门狗自动补零
        echoed.clear()
        await asyncio.sleep(1.2)
        zeros = [e for e in echoed if abs(e[1]) < 0.001]
        check('遥控: 看门狗 0.5s 自动零速', len(zeros) >= 1 and echoed[-1][1] == 0.0,
              f"收尾 v={echoed[-1][1] if echoed else None}")

        # 急停：锁定遥控 + 连发零速
        _, es = rest('/api/estop', 'POST')
        check('急停: 返回形状', es == {'cancelled_actions': [], 'stopped_stacks': []}, str(es))
        echoed.clear()
        for _ in range(6):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.0}))
            await asyncio.sleep(0.1)
        check('急停: 遥控被锁，不再有非零速度',
              not any(abs(v) > 0.01 for _, v, _ in echoed))
        _, tp_st = rest('/api/teleop/unlock', 'POST')
        check('解除锁定', tp_st.get('locked') is False)
        echoed.clear()
        for _ in range(6):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.0}))
            await asyncio.sleep(0.1)
        check('解锁后遥控恢复', any(abs(v - 0.3) < 0.01 for _, v, _ in echoed))

        # 互斥门：再来一个 /cmd_vel_intent 发布者（模拟规划器在场）→ busy 拒绝
        planner = node.create_publisher(Twist, '/cmd_vel_intent', 10)
        await asyncio.sleep(2.0)          # 等控制台的图查询看到它
        echoed.clear()
        for _ in range(6):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.0}))
            await asyncio.sleep(0.1)
        check('规划器在场: 遥控被拒（busy）', not any(abs(v) > 0.01 for _, v, _ in echoed))
        node.destroy_publisher(planner)
        await asyncio.sleep(2.0)
        echoed.clear()
        for _ in range(6):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.0}))
            await asyncio.sleep(0.1)
        check('规划器退场: 遥控恢复', any(abs(v - 0.3) < 0.01 for _, v, _ in echoed))

    async def amain():
        async with websockets.connect(WS_URL, max_size=None) as ws:
            await ws_run(ws)
            await teleop_run(ws)

    asyncio.run(amain())
    node.destroy_node()
    rclpy.shutdown()
    fails = [n for n, ok in results if not ok]
    print(f"\n=== {len(results) - len(fails)}/{len(results)} 断言通过 ===", flush=True)
    if fails:
        print('FAILED: ' + ', '.join(fails))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
