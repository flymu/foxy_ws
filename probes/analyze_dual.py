#!/usr/bin/env python3
"""analyze_dual.py — 双通道对照分析 /tmp/wire_log.txt + /tmp/ros_log.txt
钉死四个问题:
  1. 网线 udp_cnt 结构(连续/重复/帧内重置) → 线上真实唯一点率
  2. 话题点率 vs 唯一线点率 → 丢失发生在哪一段
  3. 戳推进速率 + 滞后漂移 → 10s 是恒定FIFO还是饥饿累积
  4. 逐消息对照 → 每条消息的内容对应几包网线数据
"""
from collections import Counter

W65536 = 65536


def pct(a, q):
    return a[min(len(a) - 1, int(q / 100 * len(a)))]


wire = [tuple(float(x) if i == 0 else int(x) for i, x in enumerate(l.split()))
        for l in open('/tmp/wire_log.txt')]
try:
    ros = [(float(a), float(b), int(c)) for a, b, c in (l.split() for l in open('/tmp/ros_log.txt'))]
except FileNotFoundError:
    ros = []
print('wire 行数=%d  ros 行数=%d' % (len(wire), len(ros)))

# ---------- 网线全程 ----------
w0 = wire[0][0]
span = wire[-1][0] - w0
dots = sum(w[2] for w in wire)
print('\n== 网线全程 %.1fs ==' % span)
print('包率=%.0f/s  点率(按包计数)=%.0f/s' % (len(wire) / span, dots / span))

iv = sorted(b[0] - a[0] for a, b in zip(wire, wire[1:]))
print('包到达间隔: p10=%.2f p50=%.2f p90=%.2f ms  最大=%.1f ms'
      % (pct(iv, 10) * 1e3, pct(iv, 50) * 1e3, pct(iv, 90) * 1e3, max(iv) * 1e3))
big = [((b[0] - a[0]) * 1e3) for a, b in zip(wire, wire[1:]) if b[0] - a[0] > 0.01]
print('>10ms 到达空洞数=%d (前8个: %s)' % (len(big), ['%.0fms' % x for x in big[:8]]))

cnts = [w[1] for w in wire]
dc = Counter((b - a) % W65536 for a, b in zip(cnts, cnts[1:]))
print('udp_cnt 增量 top8: %s' % dc.most_common(8))
uniq_cnt = len(set(cnts))
print('唯一 udp_cnt 数=%d / %d 包 → 唯一包率≈%.0f/s'
      % (uniq_cnt, len(wire), uniq_cnt / span))

# 重复包: 相同cnt的包, lidar ts 是否相同 (真重发=ts相同)
cnt_index = {}
dup_same_ts = 0
dup_diff_ts = 0
for w in wire:
    if w[1] in cnt_index:
        if cnt_index[w[1]] == w[3]:
            dup_same_ts += 1
        else:
            dup_diff_ts += 1
    else:
        cnt_index[w[1]] = w[3]
print('重复cnt包: ts相同=%d ts不同=%d' % (dup_same_ts, dup_diff_ts))

# lidar 内部时钟: 单位自标定 (连续+1对的 ts 增量中位数)
ts_deltas = [b[3] - a[3] for a, b in zip(wire, wire[1:])
             if (b[1] - a[1]) % W65536 == 1]
if ts_deltas:
    med = sorted(ts_deltas)[len(ts_deltas) // 2]
    print('连续包 ts 增量中位数=%d → 每包覆盖 %.1fµs'
          % (med, med / 1e3 if med > 1e4 else med / 10.0))
    tsrate = (wire[-1][3] - wire[0][3]) / (med if med > 0 else 1)
    print('按 ts 推算应到唯一包 %d, 实际唯一 %d (%.0f%%)'
          % (tsrate, uniq_cnt, uniq_cnt / tsrate * 100 if tsrate else 0))

# ---------- ROS 通道 ----------
if not ros:
    print('\n(无 ros_log.txt — 仅完成网线侧分析)')
    raise SystemExit(0)
r0 = ros[0][0]
rspan = ros[-1][0] - r0
npts = [m[2] for m in ros]
sn = sorted(npts)
lags = sorted(wl - st for wl, st, _ in ros)
half = max(1, len(lags) // 2)
print('\n== ROS 通道 %.1fs ==' % rspan)
print('消息数=%d  消息率=%.2f Hz' % (len(ros), len(ros) / rspan))
print('点数/消息: p10=%d p50=%d p90=%d  点率=%.0f/s'
      % (pct(sn, 10), pct(sn, 50), pct(sn, 90), sum(npts) / rspan))
print('戳滞后: p10=%.3f p50=%.3f p90=%.3f s' % (pct(lags, 10), pct(lags, 50), pct(lags, 90)))
print('滞后漂移: 前半均值=%.3f 后半均值=%.3f s'
      % (sum(lags[:half]) / half, sum(lags[half:]) / (len(lags) - half)))
print('戳推进速率=%.4f× 墙钟' % ((ros[-1][1] - ros[0][1]) / rspan))

# ---------- 窗口内对照 ----------
ws = [w for w in wire if r0 <= w[0] <= ros[-1][0]]
wspan = ws[-1][0] - ws[0][0] if len(ws) > 2 else 1.0
w_uniq = len(set(w[1] for w in ws))
w_dots_u = w_uniq * 96
t_dots = sum(npts)
print('\n== ROS 采样窗内对照 ==')
print('网线: 包=%d 唯一=%d → 唯一点率≈%.0f/s' % (len(ws), w_uniq, w_dots_u / wspan))
print('话题点率=%.0f/s → 保真率(对比唯一点率)=%.1f%%' % (t_dots / rspan, t_dots / w_dots_u * 100))

print('\n== 逐消息对照 (话题点数 vs 消息间隔内网线唯一包数) ==')
for i in range(1, min(len(ros), 15)):
    seg = [w for w in wire if ros[i - 1][0] <= w[0] < ros[i][0]]
    u = len(set(w[1] for w in seg))
    print('msg%02d: 话题点=%d (= pkt×%.0f)  间隔%.3fs内 网线包=%d 唯一=%d'
          % (i, ros[i][2], ros[i][2] / 96.0, ros[i][0] - ros[i - 1][0], len(seg), u))
