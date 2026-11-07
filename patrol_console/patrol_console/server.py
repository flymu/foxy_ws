"""网页服务器：REST 接口（命令）+ WebSocket（持续推送状态）+ 前端静态文件。

patrol_console 裁剪版：去掉上游的仿真场景 / 地图库 / 导航 / 航点 / 评测 / 批处理模块
（它们依赖 lidar_nav、GLIM、map_library 等真机环境没有的包），保留：
  - 建图点云流（体素增量 + 二进制帧，前端 Three.js 渲染）
  - stacks.yaml 白名单进程启停（本配置里只有 Point-LIO）
  - 遥控（/cmd_vel_intent，经 speed_adapter + veto 安全链）
  - 急停（零速度 + 锁定遥控 + 停 stop_on_estop 的栈）
  - 系统监控（话题频率 / 节点 / TF / CPU 内存）
地图保存没有网页界面：Point-LIO 是退出时落盘（map_save_path），停止 SLAM 即存图，见 docs。

上游差异（其余逐行一致）：
  - create_app 少了 library / nav_maps / navdata 参数
  - /api/mapping/presets 恒返回 []（无仿真场景）
  - 建图状态里 save 恒为 {'state': 'idle'}、waypoints 恒为 None（无对应后端）
"""
import asyncio
import json
import os
import re
import threading
import time
from typing import Optional

import psutil
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .process_manager import StackRunning

STREAMS = {'mapping'}       # 网页可以订阅的数据流；不订阅就不发（监控页不需要点云）
STREAM_PERIOD = 0.2         # 订阅的数据流多久推送一次（s）
EXPORT_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')   # 导出存档名（也挡住路径穿越）


def create_app(cfg, node, procs, mapping, teleop):
    app = FastAPI(title='patrol_console')
    period = 1.0 / float(cfg['server'].get('status_rate', 2.0))
    psutil.cpu_percent()            # 第一次调用只是开始计时，返回值无意义

    def shown(path):
        """Path for display on the page: the home directory written as ~."""
        home = os.path.expanduser('~')
        return '~' + path[len(home):] if path == home or path.startswith(home + '/') else path
    # Data folders shown on the pages (they come from the configuration, so the page must not hard-code them)
    paths_shown = {'maps': shown(cfg['mapping']['maps_dir']), 'runs': shown(cfg['mapping']['runs_dir'])}
    # 导出存档目录（网页「导出地图」/「历史地图」）
    exports_dir = cfg['mapping'].get('exports_dir') or os.path.join(cfg['mapping']['runs_dir'], 'exports')

    def status():
        snap = node.snapshot()
        snap['type'] = 'status'
        snap['time'] = time.time()
        snap['stacks'] = procs.status()
        snap['resources'] = {'cpu': psutil.cpu_percent(), 'mem': psutil.virtual_memory().percent}
        snap['paths'] = paths_shown
        return snap

    def error(status, code, **params):
        """接口错误：只返回错误代码和参数，文字由网页按当前语言生成（见 web/src/locales）。"""
        return HTTPException(status, detail={'code': code, 'params': params})

    def stack_or_404(name):
        s = procs.get(name)
        if s is None:
            raise error(404, 'stack_not_found', name=name)
        return s

    # ---------------- REST ----------------

    @app.get('/api/status')
    def get_status():
        return status()

    @app.get('/api/stacks')
    def get_stacks():
        return procs.status()

    @app.get('/api/mapping/presets')
    def get_presets():
        return []       # 无仿真场景；前端据此自动隐藏航点模式

    # ---------------- 内存地图的导出 / 载入 ----------------
    # 体素地图只活在控制台进程内存里，进程一死就没了。导出=即时落盘（npz+PCD），
    # 载入=把存档读回内存（新 epoch，网页自动清屏重收）。狗断电、控制台重启
    # 前先导出，现场不丢（11-07 神秘障碍物排查就是靠 WS 手工救援才保住的）。

    @app.post('/api/mapping/export')
    def export_map(name: Optional[str] = None):
        if name is not None and not EXPORT_RE.match(name):
            raise error(400, 'bad_export_name', name=name)
        info = mapping.export(exports_dir, name)
        if info is None:
            raise error(409, 'map_empty')
        info['dir'] = shown(exports_dir)
        return info

    @app.get('/api/mapping/exports')
    def list_exports():
        return {'dir': shown(exports_dir), 'exports': mapping.list_exports(exports_dir)}

    @app.post('/api/mapping/exports/{name}/load')
    def load_export(name: str):
        if not EXPORT_RE.match(name):
            raise error(404, 'export_not_found', name=name)
        path = os.path.join(exports_dir, name + '.npz')
        if not os.path.isfile(path):
            raise error(404, 'export_not_found', name=name)
        try:
            info = mapping.load(path)
        except Exception as e:
            raise error(500, 'load_failed', reason=repr(e))
        return {'name': name, **info}

    @app.post('/api/mapping/voxel')
    def set_map_voxel(v: float):
        # 0.02 以下浏览器端点数爆炸，0.5 以上矮物细节全糊掉
        if not (0.02 <= v <= 0.5):
            raise error(400, 'bad_voxel', v=v)
        mapping.set_voxel(v)
        return {'voxel': mapping.voxel}

    @app.post('/api/stacks/{name}/start')
    def start_stack(name: str, origin: Optional[str] = None):
        stack_or_404(name)
        is_slam = name in mapping.profiles
        if is_slam:
            # one SLAM at a time: two would share the CPU and the mapping page shows only one
            other = next((n for n in mapping.profiles if n != name and procs.get(n)
                          and procs.get(n).state() in ('running', 'stopping')), None)
            if other:
                raise error(409, 'stack_running', name=other)
        try:
            info = procs.start(name, [], None, origin if origin in ('mapping', 'monitor') else 'monitor')
            if is_slam:
                mapping.select(name)
                mapping.reset()       # 每次开始建图都从空地图开始
            return info
        except StackRunning:
            raise error(409, 'stack_running', name=name)
        except OSError as e:          # 命令不存在等
            raise error(500, 'start_failed', name=name, reason=str(e))

    @app.post('/api/stacks/{name}/stop')
    def stop_stack(name: str):
        # 停止可能要几十秒（Point-LIO 退出时要写地图），放到后台线程，立即返回；
        # 网页通过 WebSocket 推送的状态看到 stopping → exited
        s = stack_or_404(name)
        threading.Thread(target=procs.stop, args=(name,), daemon=True).start()
        info = s.info()
        if info['state'] == 'running':
            info['state'] = 'stopping'
        return info

    @app.get('/api/stacks/{name}/log')
    def stack_log(name: str, lines: int = 200):
        stack_or_404(name)
        return {'name': name, 'text': procs.log_tail(name, max(1, min(lines, 2000)))}

    @app.post('/api/estop')
    def estop():
        cancelled = node.estop()
        teleop.lock()                 # 急停后遥控锁定，网页上解除后才能再遥控
        stopped = procs.estop_names()
        for n in stopped:
            threading.Thread(target=procs.stop, args=(n,), daemon=True).start()
        return {'cancelled_actions': cancelled, 'stopped_stacks': stopped}

    @app.post('/api/teleop/unlock')
    def teleop_unlock():
        teleop.unlock()
        return teleop.status()

    # ---------------- WebSocket ----------------

    # 每个网页连接两个任务：send_loop 定时发数据，ws 本身接收网页发来的消息。网页消息：
    #   {"type": "subscribe", "streams": ["mapping"]}：订阅数据流。每次订阅 mapping 都会从头重发整张地图
    #     （网页发现数据对不上时也用它来重新同步）
    #   {"type": "teleop", "v": 0.3, "w": 0.0}：遥控速度，网页按住摇杆时 10 Hz 发送；连接断开时立即停车

    async def send_loop(sock, client):
        next_status = 0.0
        # Foxy 是 Python 3.8，没有 asyncio.to_thread（3.9+），用事件的默认线程池
        loop = asyncio.get_running_loop()
        try:
            while True:
                if time.monotonic() >= next_status:
                    next_status = time.monotonic() + period
                    # snapshot 里有 ROS 图查询和加锁，放到线程池里做，不卡住事件循环
                    await sock.send_json(await loop.run_in_executor(None, status))
                if 'mapping' in client['streams']:
                    ms = mapping.status()
                    ms['waypoints'] = None          # 无航点后端（自动航点已裁剪）
                    ms['teleop'] = teleop.status()
                    ms['save'] = {'state': 'idle'}  # 无保存后端（Point-LIO 停止时自己落盘）
                    await sock.send_json(ms)
                    for f in await loop.run_in_executor(None, mapping.pull, client['cursor']):
                        if isinstance(f, bytes):
                            await sock.send_bytes(f)
                        else:
                            await sock.send_json(f)
                await asyncio.sleep(STREAM_PERIOD)
        except (WebSocketDisconnect, RuntimeError):
            pass                       # 浏览器关闭或刷新页面

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        client = {'streams': set(), 'cursor': {}}
        sender = asyncio.create_task(send_loop(sock, client))
        try:
            while True:
                try:
                    msg = json.loads(await sock.receive_text())
                except ValueError:
                    continue           # 不是 JSON，忽略
                if isinstance(msg, dict) and msg.get('type') == 'subscribe':
                    streams = msg.get('streams')
                    client['streams'] = STREAMS & set(streams if isinstance(streams, list) else [])
                    client['cursor'] = {}
                elif isinstance(msg, dict) and msg.get('type') == 'teleop':
                    # 被拒绝（锁定、规划器在发速度）时不回复，网页从建图状态里的 teleop 字段看到原因
                    teleop.command(msg.get('v'), msg.get('w'), owner=id(client))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            teleop.release(id(client))
            sender.cancel()

    # ---------------- 前端页面 ----------------
    # 开发时用 npm run dev（端口 5173）；打包后（npm run build）由这里直接提供页面
    dist = cfg['server']['web_dist']
    if os.path.isdir(dist):
        app.mount('/', StaticFiles(directory=dist, html=True), name='web')
    else:
        @app.get('/')
        def no_web():
            return {'message': f'前端还没打包：{dist} 不存在。开发时请访问 http://localhost:5173',
                    'message_en': f'Web UI not built: {dist} not found. For development open http://localhost:5173'}

    return app
