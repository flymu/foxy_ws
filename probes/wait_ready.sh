#!/bin/bash
# wait_ready.sh — 起栈后等定位就绪（容器内跑）：health=OK 才放行，末尾打印位姿
source /opt/ros/foxy/setup.bash
for i in $(seq 1 40); do
  out=$(timeout 6 python3 /home/mu/foxy_ws/probes/health_pose_probe.py 2>/dev/null)
  echo "[$i] $out"
  case "$out" in
    OK*) echo "READY"; exit 0 ;;
  esac
  sleep 2
done
echo "TIMEOUT-未就绪"
tail -n 15 /tmp/live_stack/loc.log 2>/dev/null
exit 1
