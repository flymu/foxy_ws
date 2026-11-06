#!/bin/bash
# link_return_watch.sh — 狗侧链路恢复监视: eno1 载波恢复 → 继续盯 loc.log 数据/健康
# 载波恢复即报一行, 之后转 loc.log(容器内) 等数据回归([match] 重现/续接/异常)
(while true; do
  c=$(cat /sys/class/net/eno1/carrier 2>/dev/null || echo 0)
  if [ "$c" = "1" ]; then echo "[link] eno1 载波恢复 $(date +%H:%M:%S)"; break; fi
  sleep 5
done
sg docker -c "docker exec foxy-live tail -n 0 -f /tmp/live_stack/loc.log") 2>/dev/null \
  | grep --line-buffered -E "match|WARN|ERROR|续接|resume|重种子|reseed|DEGRADED|LOST"
