"""控制台冒烟探针（无狗环境，配 synth_publisher.py 用）：一行一个 PASS/FAIL 断言。

覆盖：
  REST   /api/status /api/stacks /api/mapping/presets /api/stacks/{name}/log 404
  WS     mapping_reset / mapping 状态字段 / 二进制帧格式与体素连续性
  导出   /api/mapping/export 落盘 npz+PCD、非法名 400、历史列表、
         载入 → epoch 变更 + mapping_reset + 新 epoch 整图重发
  体素   /api/mapping/voxel 运行时改分辨率（状态帧生效）、非法值 400
  遥控   10Hz 指令落到 /cmd_vel_intent、看门狗 0.5s 自动零速、
         急停锁定、解除、规划器在场时 busy 拒绝（互斥门）
  巡检栈 live_stack（live_stack_start.sh 的网页包装）：启动→节点就位→LIO 互斥→
         规划器占线遥控让位→停止走 live_stack_stop.sh→清场→互斥解除；
         收尾检查控制台 DDS 没被起栈脚本的 rm SHM 毒化（合成里程计还在收）
用法（容器内，已 source ROS，PYTHONPATH 含 pydeps）：
    python3 /home/mu/foxy_ws/patrol_console/tests/ws_probe.py
"""
import asyncio
import json
import os
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
    check('stacks: 白名单 = 建图 + 巡检栈', len(stacks) == 2
          and {s['name'] for s in stacks} == {'point_lio', 'live_stack'}
          # idle 只在控制台刚重启时成立：跑过一轮后栈停在 exited（没在跑）也算干净
          and all(s['state'] in ('idle', 'exited') for s in stacks),
          str([(s['name'], s['state']) for s in stacks]))
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
        return got_reset, mapping

    async def export_run(ws, cur_epoch):
        """导出/历史/载入三件套。synth 在跑：地图一直在长，断言只做下限。"""
        _, info = rest('/api/mapping/export?name=probe_map', 'POST')
        check('导出: 返回形状', info.get('name') == 'probe_map' and info['points'] > 0
              and info['npz'].endswith('probe_map.npz') and info['pcd'].endswith('probe_map.pcd'),
              f"points={info.get('points')}")
        # 非法名（路径穿越/非法字符）必须 400，而不是落盘
        try:
            rest('/api/mapping/export?name=../evil', 'POST')
            check('导出: 非法名被拒', False)
        except urllib.error.HTTPError as e:
            check('导出: 非法名被拒', e.code == 400
                  and json.loads(e.read())['detail']['code'] == 'bad_export_name')
        # 文件真的落在 exports 目录里（探针和控制台同容器，直接看文件）
        import os
        stem = '/home/mu/foxy_ws/patrol_console/data/exports/probe_map'
        check('导出: npz + PCD 落盘', os.path.isfile(stem + '.npz') and os.path.isfile(stem + '.pcd')
              and os.path.getsize(stem + '.pcd') > 100, stem)
        _, lst = rest('/api/mapping/exports')
        check('历史: 列表含本存档', any(x['name'] == 'probe_map' and x['points'] > 0
                                      for x in lst['exports']), str(lst['exports'])[:120])
        # 载入：epoch 变更，WS 端应看到新 epoch 的 mapping_reset + 整图重发
        _, loaded = rest('/api/mapping/exports/probe_map/load', 'POST')
        new_epoch = loaded['epoch']
        check('载入: 返回形状', loaded['name'] == 'probe_map' and loaded['voxels'] > 0
              and new_epoch > cur_epoch, f"epoch {cur_epoch}->{new_epoch}, voxels={loaded['voxels']}")
        reset_epoch, total_map = None, 0
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and (reset_epoch is None or total_map < loaded['voxels']):
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if isinstance(msg, bytes):
                (hlen,) = struct.unpack_from('<I', msg, 0)
                head = json.loads(msg[4:4 + hlen])
                if head['epoch'] == new_epoch and head['type'] == 'map':
                    total_map += head['count']
            else:
                m = json.loads(msg)
                if m.get('type') == 'mapping_reset':
                    reset_epoch = m['epoch']
        # synth 还在长地图，新 epoch 期间会继续追加，总数只会 ≥ 存档体素数
        check('载入: 新 epoch mapping_reset', reset_epoch == new_epoch,
              f"reset={reset_epoch}")
        check('载入: 整图重发到位', total_map >= loaded['voxels'],
              f"{total_map}/{loaded['voxels']}")

        # 体素分辨率运行时可调：改 0.05 → 状态帧应上报新值（只影响之后新扫入的点）
        rest('/api/mapping/voxel?v=0.05', 'POST')
        vox, deadline = None, time.monotonic() + 5
        while time.monotonic() < deadline and vox != 0.05:
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if not isinstance(msg, bytes):
                m = json.loads(msg)
                if m.get('type') == 'mapping':
                    vox = m['voxel']
        check('体素: 运行时改 0.05 状态生效', vox == 0.05, f"voxel={vox}")
        try:
            rest('/api/mapping/voxel?v=0.001', 'POST')
            check('体素: 非法值被拒', False)
        except urllib.error.HTTPError as e:
            check('体素: 非法值被拒', e.code == 400
                  and json.loads(e.read())['detail']['code'] == 'bad_voxel')
        _, rv = rest('/api/mapping/voxel?v=0.1', 'POST')
        check('体素: 恢复 0.1', rv == {'voxel': 0.1}, str(rv))

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

    def live_nodes(names=('point_lio_node', 'map_localization_node', 'scan_planner_node',
                         'speed_adapter', 'viz_helper')):
        """容器里活着的（非僵尸）巡检节点名集合。直扫 /proc：pgrep -f 对 cmdline
        为空的僵尸会退回按进程名匹配，容器 PID1 不收割的老病会误报成活节点。"""
        found = set()
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                with open(f'/proc/{pid}/stat', encoding='utf-8', errors='replace') as f:
                    state = f.read().rsplit(')', 1)[1].split()[0]
                if state == 'Z':
                    continue
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    line = f.read().decode(errors='replace')
            except OSError:
                continue
            found.update(n for n in names if n in line)
        return found

    async def patrol_run(ws):
        """巡检栈（live_stack）：网页启动 → 互斥/占线 → 网页停止 → 清场。
        节点在容器里真实跑（狗没上电：雷达/里程计无数据，LIO 空转，规划器有发布者
        但只在有定位后才发速度——占线门按"发布者存在"判定，照样生效）。"""
        alive_names = ('point_lio_node', 'map_localization_node', 'scan_planner_node',
                       'speed_adapter', 'viz_helper')
        def patrol_entry():
            _, cur = rest('/api/stacks')
            return next(s for s in cur if s['name'] == 'live_stack')

        # 启动：接口立即返回 running（监护进程在跑），脚本内部 sleep 3+2 后节点才就位
        _, ps = rest('/api/stacks/live_stack/start', 'POST')
        check('巡检栈: 启动即 running', ps.get('state') == 'running', str(ps.get('state')))
        # 等起栈脚本自己跑完（它最后会 echo "live stack up"）
        log_text, deadline = '', time.monotonic() + 40
        while time.monotonic() < deadline:
            _, lg = rest('/api/stacks/live_stack/log?lines=40')
            log_text = lg['text']
            if 'live stack up' in log_text:
                break
            time.sleep(1)
        check('巡检栈: 起栈脚本完成', 'live stack up' in log_text,
              (log_text.strip().splitlines() or [''])[-1][:60])
        alive = live_nodes(alive_names)
        check('巡检栈: 五类节点在跑', alive == set(alive_names), f'{sorted(alive)}')
        # 启动开新 epoch（巡检栈的 LIO 也发 /cloud_registered，建图页从空地图看本趟）
        reset_epoch, deadline = None, time.monotonic() + 6
        while time.monotonic() < deadline and reset_epoch is None:
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if not isinstance(msg, bytes):
                m = json.loads(msg)
                if m.get('type') == 'mapping_reset':
                    reset_epoch = m['epoch']
        check('巡检栈: 启动开新 epoch', reset_epoch is not None, f'epoch={reset_epoch}')
        # LIO 互斥：巡检栈在跑时 point_lio 必须被拒（双开 LIO 地图会跳）
        try:
            rest('/api/stacks/point_lio/start', 'POST')
            check('巡检栈: LIO 互斥挡住 point_lio', False)
        except urllib.error.HTTPError as e:
            detail = json.loads(e.read())['detail']
            check('巡检栈: LIO 互斥挡住 point_lio', e.code == 409
                  and detail['code'] == 'stack_running' and detail['params']['name'] == 'live_stack',
                  str(detail))
        # 规划器占线：scan_planner 在 /cmd_vel_intent 上有发布者（即使还没发速度），
        # 遥控让位。等 DDS 发现 + 控制台 2s 图查询收敛。
        await asyncio.sleep(4.0)
        echoed.clear()
        for _ in range(6):
            await ws.send(json.dumps({'type': 'teleop', 'v': 0.3, 'w': 0.0}))
            await asyncio.sleep(0.1)
        check('巡检栈: 规划器在场遥控被拒', not any(abs(v) > 0.01 for _, v, _ in echoed))
        # 停止：走 live_stack_stop.sh（stop_cmd），有序 pkill（SHM 清理被环境变量跳过）
        _, st = rest('/api/stacks/live_stack/stop', 'POST')
        check('巡检栈: 停止返回 stopping', st.get('state') == 'stopping', str(st.get('state')))
        entry, deadline = patrol_entry(), time.monotonic() + 45
        while time.monotonic() < deadline and entry['state'] != 'exited':
            time.sleep(1)
            entry = patrol_entry()
        check('巡检栈: 停止收敛到 exited', entry['state'] == 'exited', str(entry.get('last_stop')))
        check('巡检栈: 停止走了 stop_cmd 脚本',
              (entry.get('last_stop') or {}).get('stage') == 'stop_cmd',
              str(entry.get('last_stop')))
        check('巡检栈: 停止后节点清场', not live_nodes(alive_names),
              str(sorted(live_nodes(alive_names))))
        # 相机 republish 是"没有才补"的常驻助手（stop.sh 刻意不杀）——组吊死问题的
        # 连坐牺牲品曾今是它：setsid 出栈组后，停止不得误伤
        check('巡检栈: 相机 republish 常驻未连坐',
              'go2_camera_republish' in live_nodes(('go2_camera_republish',)))
        # 互斥解除：point_lio 能起能停（真容器里跑一遍，收尾别留进程）
        _, pl = rest('/api/stacks/point_lio/start', 'POST')
        check('巡检栈停后: point_lio 可启动', pl.get('state') == 'running', str(pl.get('state')))
        rest('/api/stacks/point_lio/stop', 'POST')
        _, cur = rest('/api/stacks')
        pl_state, deadline = next(s['state'] for s in cur if s['name'] == 'point_lio'), time.monotonic() + 30
        while time.monotonic() < deadline and pl_state != 'exited':
            time.sleep(1)
            _, cur = rest('/api/stacks')
            pl_state = next(s['state'] for s in cur if s['name'] == 'point_lio')
        check('巡检栈停后: point_lio 停止收敛', pl_state == 'exited', f'state={pl_state}')
        # 收尾金丝雀：起栈脚本流程必须全程保留控制台的 DDS（stacks.yaml 给 live_stack
        # 配了 LIVE_STACK_KEEP_SHM=1 跳过 rm /dev/shm/fastrtps_port*——11-07 实测 rm 会把
        # 控制台劈进孤代，点云/互斥门全瞎）。若 SHM 被毒化，控制台就收不到合成
        # /Odometry，initialized 会掉。
        mapping, deadline = None, time.monotonic() + 6
        while time.monotonic() < deadline and not (mapping and mapping.get('initialized')):
            msg = await asyncio.wait_for(ws.recv(), timeout=5)
            if not isinstance(msg, bytes):
                m = json.loads(msg)
                if m.get('type') == 'mapping':
                    mapping = m
        check('巡检栈: 收尾控制台 DDS 仍活（合成里程计还在收）',
              bool(mapping and mapping.get('initialized')), str(mapping and mapping.get('pose')))

    async def amain():
        ws = await websockets.connect(WS_URL, max_size=None)
        try:
            epoch0, _ = await ws_run(ws)
            await export_run(ws, epoch0)
            await teleop_run(ws)
            await patrol_run(ws)
        finally:
            # 断言结果才是正事；收尾握手的异常（服务端忙时偶发 no close frame）别吞掉汇总
            try:
                await ws.close()
            except Exception:
                pass

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
