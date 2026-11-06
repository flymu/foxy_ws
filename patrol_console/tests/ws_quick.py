"""ws_quick.py — 快速验证 #/mapping 数据流：订阅 WS，数 text/二进制帧。
容器内跑：PYTHONPATH=pydeps:. python3 tests/ws_quick.py"""
import asyncio
import json

import websockets

WS_URL = 'ws://127.0.0.1:8000/ws'


async def main():
    async with websockets.connect(WS_URL, max_size=None) as ws:
        await ws.send(json.dumps({'type': 'subscribe', 'streams': ['mapping']}))
        n_text = n_bin = 0
        total = 0
        status = None
        try:
            while n_bin < 3 and (n_text + n_bin) < 80:
                msg = await asyncio.wait_for(ws.recv(), timeout=10)
                if isinstance(msg, bytes):
                    n_bin += 1
                    total += len(msg)
                else:
                    n_text += 1
                    d = json.loads(msg)
                    t = str(d.get('type', ''))
                    if status is None and 'map' in t:
                        status = d
        except asyncio.TimeoutError:
            pass
        print('text帧:', n_text, '| 二进制帧:', n_bin, '| 累计字节:', total)
        if status:
            print('mapping状态样例:', json.dumps(status)[:400])
        else:
            print('!! 没收到 mapping 状态帧')


asyncio.run(main())