#!/usr/bin/env python3
# eval_loc.py — 对比 /loc/pose 回显 vs 建图时的 trajectory.txt（真值）
import sys

def parse_echo(path):
    """从 ros2 topic echo 的 YAML 输出提取 (t_sec, x, y, z)"""
    out = []
    t = None
    state = 0  # 0=idle, 1..3 = expect x,y,z under position
    for line in open(path):
        s = line.strip()
        if s.startswith('sec:'):
            t = float(s.split(':')[1])
            ns = 0.0
        elif s.startswith('nanosec:'):
            ns = float(s.split(':')[1])
            if t is not None:
                t += ns * 1e-9
        elif s == 'position:':
            state = 1
        elif state and s.startswith('x:'):
            out.append({'t': t, 'x': float(s.split(':')[1])})
            state = 2
        elif state == 2 and s.startswith('y:'):
            out[-1]['y'] = float(s.split(':')[1])
            state = 3
        elif state == 3 and s.startswith('z:'):
            out[-1]['z'] = float(s.split(':')[1])
            state = 0
    return [r for r in out if 'y' in r and 'z' in r]

def main(echo_path, gt_path):
    loc = parse_echo(echo_path)
    gt = []
    with open(gt_path) as f:
        for i, line in enumerate(f):
            if i == 0 or line.startswith('#'):
                continue
            v = line.split()
            gt.append((float(v[0]), float(v[1]), float(v[2]), float(v[3])))
    if not loc:
        print('echo 文件无有效消息!')
        return
    print('loc 消息 %d 条, 真值 %d 位姿' % (len(loc), len(gt)))
    import bisect
    gts = [g[0] for g in gt]
    errs, n = [], 0
    for r in loc:
        i = bisect.bisect_left(gts, r['t'])
        for j in (i, i - 1):  # 先查上邻（含精确命中），再查下邻
            if 0 <= j < len(gt):
                if abs(gt[j][0] - r['t']) < 0.06:
                    dx, dy, dz = gt[j][1] - r['x'], gt[j][2] - r['y'], gt[j][3] - r['z']
                    errs.append((dx * dx + dy * dy + dz * dz) ** 0.5)
                    n += 1
                    break
    if not errs:
        print('时间戳没对上: loc t[%.1f, %.1f] vs gt t[%.1f, %.1f]' %
              (loc[0]['t'], loc[-1]['t'], gt[0][0], gt[-1][0]))
        return
    errs.sort()
    e_final = ((loc[-1]['x'] - gt[-1][1]) ** 2 + (loc[-1]['y'] - gt[-1][2]) ** 2 +
               (loc[-1]['z'] - gt[-1][3]) ** 2) ** 0.5
    print('配准样本 %d: 误差 mean=%.3fm median=%.3fm p95=%.3fm max=%.3fm' %
          (n, sum(errs) / n, errs[len(errs) // 2], errs[int(len(errs) * 0.95)], errs[-1]))
    print('终点误差 %.3fm (loc=(%.2f,%.2f,%.2f) gt=(%.2f,%.2f,%.2f))' %
          (e_final, loc[-1]['x'], loc[-1]['y'], loc[-1]['z'], gt[-1][1], gt[-1][2], gt[-1][3]))

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
