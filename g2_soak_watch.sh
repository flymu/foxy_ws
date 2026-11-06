#!/bin/bash
# g2_soak_watch.sh — G2 定位浸泡 30min 监视（宿主跑, 由 Monitor 拉起）
# 输出: loc.log 的 [match] 遥测(fitness/漂移) + 异常特征 + 每5min 心跳(位姿漂移+健康+落盘新鲜度)
BASE_X=-1.1560   # 浸泡起点 t_mo (loc_state.txt)
BASE_Y=0.0441
STATE=/home/mu/maps/loc_state.txt   # 容器/宿主共享

# 1) loc.log 流: match 遥测(~80s一条) + 一切 WARN/ERROR/状态异常 + fitness 恶化(>=0.2 早于 0.35 阈值)
sg docker -c "docker exec foxy-live tail -n 0 -f /tmp/live_stack/loc.log" 2>/dev/null \
  | grep -E --line-buffered "WARN|ERROR|FATAL|\[match\]|丢弃|冻结|死推|拒绝|重种子|reseed|DEGRADED|fitness=0\.[2-9]|fitness=[1-9]" &

# 2) 心跳: 每5min 检查 T_mo 距基线 / 定位进程存活 / 落盘新鲜度 / 健康探针
n=0
while true; do
  sleep 300
  n=$((n+1))
  alive=$(ps -eo stat,cmd | grep "[m]ap_localization_node" | grep -vc "^Z")
  wall=$(awk '/^wall/{print $2}' $STATE 2>/dev/null)
  age=$(( $(date +%s) - ${wall:-0} ))
  # loc_state 的 t_mo 行=行优先 3x4(12数)：$5=tx $9=ty $13=tz（$6 是 r10=sin(yaw)，
  # 曾误当 ty 读了 0.254=yaw15° 假报漂移 0.21m——遥测漂移 0.04 才是真值）
  tx=$(awk '/^t_mo/{print $5}' $STATE 2>/dev/null)
  ty=$(awk '/^t_mo/{print $9}' $STATE 2>/dev/null)
  d=$(awk -v bx=$BASE_X -v by=$BASE_Y -v x=${tx:-0} -v y=${ty:-0} \
      'BEGIN{printf "%.3f", sqrt((x-bx)^2+(y-by)^2)}')
  flag="OK"
  [ "$alive" -lt 1 ] && flag="$flag !!进程消失"
  [ "$age" -gt 15 ]   && flag="$flag !!落盘停滞${age}s"
  awk -v d=$d 'BEGIN{exit !(d>0.10)}' && flag="$flag !!漂移超限"
  echo "[soak +$((n*5))min] T_mo=(${tx:-?},${ty:-?}) 距基线=${d}m 落盘龄=${age}s $flag"
done
