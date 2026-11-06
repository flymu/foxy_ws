#!/bin/bash
# g4_narrate.sh — G4 自主导航实时解说: planner 目标/门控 + loc 看门狗/健康 + adapter
sg docker -c "docker exec foxy-live tail -n 0 -f /tmp/live_stack/planner.log /tmp/live_stack/loc.log /tmp/live_stack/adapter.log" 2>/dev/null \
  | grep --line-buffered -E "目标|goal|门控|gate|watchdog|分歧|死推|DEGRADED|LOST|WARN|ERROR|起步|地板|clearTarget"