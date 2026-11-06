"""map_rescue.py — 把 console 内存里的体素地图完整拉下来存盘（救援/导出通用）。

背景：体素地图只活在 console 进程内存里，进程一死就没了（今晚狗已断电，这份图
是神秘障碍物的唯一现场证据）。本脚本走现有 WS 协议 subscribe → mapping_reset →
整图重放，解码二进制帧，存 npz + PCD。

二进制帧格式（mapping.py encode）：[4B 头长度][JSON头(空格补齐4对齐)][float32 xyz...]

用法（容器内，PYTHONPATH 含 pydeps）：python3 tests/map_rescue.py [输出前缀]
默认输出 /home/mu/foxy_ws/patrol_console/data/rescue_<时间戳>
"""
import asyncio
import json
import struct
import sys
import time

import numpy as np
import websockets

WS_URL = 'ws://127.0.0.1:8000/ws'
OUT = sys.argv[1] if len(sys.argv) > 1 else None


def decode_binary(buf):
    (hlen,) = struct.unpack_from('<I', buf, 0)
    head = json.loads(buf[4:4 + hlen])
    pts = np.frombuffer(buf, dtype='<f4', count=3 * head['count'], offset=4 + hlen)
    return head, pts.reshape(-1, 3).copy()


async def main():
    chunks = {'map': {}, 'est': {}, 'gt': {}}
    scan = None
    status = None
    saw_reset = False
    last_bin_t = time.monotonic()
    t0 = time.monotonic()
    out = OUT or ('/home/mu/foxy_ws/patrol_console/data/rescue_' + time.strftime('%m%d_%H%M%S'))
    async with websockets.connect(WS_URL, max_size=None) as ws:
        await ws.send(json.dumps({'type': 'subscribe', 'streams': ['mapping']}))
        while True:
            done = (saw_reset and status is not None and chunks['map']
                    and time.monotonic() - last_bin_t > 3.0)
            if done or time.monotonic() - t0 > 90:
                break
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=3.0)
            except asyncio.TimeoutError:
                continue
            if isinstance(msg, bytes):
                head, pts = decode_binary(msg)
                t = head['type']
                if t == 'scan':
                    scan = pts
                else:
                    chunks[t][head['offset']] = pts
                last_bin_t = time.monotonic()
                print('  帧 %s offset=%d count=%d' % (t, head.get('offset', 0), head['count']), flush=True)
            else:
                d = json.loads(msg)
                if d.get('type') == 'mapping_reset':
                    saw_reset = True
                    print('mapping_reset epoch=%s' % d.get('epoch'), flush=True)
                elif d.get('type') == 'mapping':
                    status = d

    def assemble(kind):
        if not chunks[kind]:
            return np.zeros((0, 3), np.float32)
        offs = sorted(chunks[kind])
        arrs = [chunks[kind][o] for o in offs]
        # offset 连续性核对
        expect = 0
        for o, a in zip(offs, arrs):
            assert o == expect, '%s 帧不连续: %d != %d' % (kind, o, expect)
            expect += len(a)
        return np.vstack(arrs)

    m, est, gt = assemble('map'), assemble('est'), assemble('gt')
    print('\n地图 %d 点 | 轨迹 %d 点 | 真值 %d 点 | 当前帧 %s 点'
          % (len(m), len(est), len(gt), '无' if scan is None else len(scan)))
    if status:
        print('status: voxels=%d pose=%s extent=%s'
              % (status['voxels'], status['pose'], status['extent']))
        assert len(m) == status['voxels'], '拉取点数 %d != status 体素数 %d' % (len(m), status['voxels'])
    np.savez(out + '.npz', map=m, est=est, gt=gt,
             scan=np.zeros((0, 3), np.float32) if scan is None else scan,
             status=json.dumps(status or {}), wall=time.time())
    # PCD（ascii，CloudCompare/RViz 可直接开）
    with open(out + '.pcd', 'w') as f:
        f.write('# .PCD v0.7 - ASCII\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(m), len(m)))
        for x, y, z in m:
            f.write('%g %g %g\n' % (x, y, z))
    print('已存 %s.npz + %s.pcd' % (out, out))


asyncio.run(main())