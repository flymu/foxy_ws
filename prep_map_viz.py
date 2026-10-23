#!/usr/bin/env python3
# prep_map_viz.py — 全局地图降采样为可视化 blob (float32 交错 x,y,z,intensity)
# 主机上运行（numpy 2.0.2），输出喂给 viz_helper.py 在容器里发布
import numpy as np

PCD = '/home/mu/maps/lio_1792684457_global_map.pcd'
OUT = '/home/mu/maps/global_map_viz.f32'
VOX = 0.1

with open(PCD, 'rb') as f:
    hdr = {}
    while True:
        raw_line = f.readline()
        if not raw_line:  # EOF 防死循环
            raise RuntimeError('PCD 头部没有 DATA 行')
        line = raw_line.decode('ascii', 'ignore').strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()  # PCD 头是空格分隔，不是 KEY: value
        hdr[parts[0].upper()] = ' '.join(parts[1:])
        if parts[0].upper() == 'DATA':
            break
    assert hdr['DATA'] == 'binary', hdr['DATA']
    n = int(hdr['POINTS'])
    # 8 字段 float32 (x y z intensity nx ny nz curvature) = 32 字节/点
    raw = np.frombuffer(f.read(n * 32), dtype=np.float32).reshape(n, 8)

# 体素坐标线性编码成一维 int64（numpy2 的 axis=0 unique 走 void 排序太慢）
i, j, k = (np.floor(raw[:, :3] / VOX).astype(np.int64).T + 512)
key = (i * 1024 + j) * 1024 + k
_, idx = np.unique(key, return_index=True)
idx.sort()
sel = raw[idx, :4].astype(np.float32)  # 只留 x y z intensity
sel.tofile(OUT)
print(f'{n} -> {len(sel)} 点 (vox {VOX}m), 已存 {OUT} ({len(sel)*16//1024}KB)')
