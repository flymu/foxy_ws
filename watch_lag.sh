#!/bin/bash
# watch_lag.sh — 摄入滞后监视（配合 veto_viz 心跳）
# veto_viz.log 最新一条 摄入滞后 超过阈值时输出一行并退出。
# 用法: bash /home/mu/foxy_ws/watch_lag.sh [阈值秒，默认12]
TH=${1:-12}
while true; do
  L=$(grep 摄入滞后 /tmp/veto_viz.log 2>/dev/null | tail -1 \
      | sed -E 's/.*摄入滞后([0-9.]+)s 处理.*/\1/')
  if awk "BEGIN{exit !(${L:-0} > $TH)}"; then
    echo "LAG_EXCEEDED ${L}s (阈值${TH}s) — 驱动队列又积压了，需要重启驱动"
    exit 0
  fi
  sleep 10
done
