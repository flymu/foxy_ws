#!/bin/bash
# g4_narrate.sh — G4 自主导航实时解说: planner 目标/门控/FSM + loc 看门狗 + adapter
# 滤掉纯闲置噪声: WAIT_TARGET 零速状态行 + adapter 10s 心跳的"全零"形态
sg docker -c "docker exec foxy-live tail -n 0 -f /tmp/live_stack/planner.log /tmp/live_stack/loc.log /tmp/live_stack/adapter.log" 2>/dev/null \
  | grep --line-buffered -E "收到目标|目标已生效|门控|gate|watchdog|分歧|死推|DEGRADED|LOST|WARN|ERROR|起步|地板|clearTarget|Emergency|Replan failed|a star|drone is in|Ran out of pool|feasibility|重分配|\[planner\]" \
  | grep --line-buffered -vE "WAIT_TARGET traj=0 target=0 cmd=\(0\.00,0\.00,0\.00\)|适配: 100\.0Hz 抬地板 0 停步 1000 原样 0"
