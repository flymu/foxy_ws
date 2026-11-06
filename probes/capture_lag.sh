#!/bin/bash
# capture_lag.sh — 采样 veto_viz 摄入滞后（默认20次×3s），观察驱动重启后的回落
# 用法: bash /home/mu/foxy_ws/capture_lag.sh [次数]
N=${1:-20}
for i in $(seq 1 "$N"); do
  L=$(grep 摄入滞后 /tmp/veto_viz.log 2>/dev/null | tail -1 \
      | sed -E 's/^\[ *([0-9.]+)s\].*摄入滞后([0-9.]+)s 处理([0-9]+)ms.*/viz[\1s] lag=\2s proc=\3ms/')
  echo "$(date -u +%H:%M:%S) ${L:-no_heartbeat_yet}"
  sleep 3
done