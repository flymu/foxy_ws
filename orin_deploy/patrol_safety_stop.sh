#!/bin/bash
# patrol_safety_stop.sh — 停 Orin 侧安全栈（relay+veto+viz）
# 用法: ssh go2_wifi 'bash /home/unitree/patrol_safety_stop.sh'
pkill -f "[l]ivox_relay" 2>/dev/null
pkill -f "[o]bstacle_veto" 2>/dev/null
pkill -f "[v]eto_viz" 2>/dev/null
sleep 1
pkill -9 -f "[l]ivox_relay" 2>/dev/null
pkill -9 -f "[o]bstacle_veto" 2>/dev/null
pkill -9 -f "[v]eto_viz" 2>/dev/null
if pgrep -f "[l]ivox_relay|[o]bstacle_veto|[v]eto_viz" > /dev/null 2>&1; then
  echo "SAFETY_STILL_ALIVE:"
  pgrep -af "[l]ivox_relay|[o]bstacle_veto|[v]eto_viz"
  exit 1
fi
echo SAFETY_STOPPED
exit 0
