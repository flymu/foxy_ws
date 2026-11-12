#!/bin/bash
# rssi_walk_test.sh — 5G 覆盖徒步测试（在 Orin 上跑，牵狗沿巡逻线走）
# 用法：bash rssi_walk_test.sh [if名]        # 缺省自动挑连着 littlered_5G 的接口
# 走完 Ctrl-C，出 RSSI/ping RTT 的 min/p10/median 汇总；CSV 落 /tmp。
set -u
IF="${1:-}"
if [ -z "$IF" ]; then
  for i in $(iw dev 2>/dev/null | awk '/Interface/{print $2}'); do
    if iw dev "$i" link 2>/dev/null | grep -qi 'ssid.*littlered_5G'; then IF="$i"; break; fi
  done
  [ -n "$IF" ] || { echo "!! 没找到连着 littlered_5G 的接口，请传参：bash $0 <if名>（iw dev 看）"; exit 1; }
fi
CSV="/tmp/rssi_walk_$(date +%H%M%S).csv"
WS=192.168.101.25
echo "接口=$IF  目标=WS($WS)  CSV=$CSV"
echo "牵狗沿巡逻线走一圈，走完 Ctrl-C。"
echo "time,rssi_dbm,ssid,ping_ms" | tee "$CSV"
trap 'echo; break' INT
n=0
while :; do
  sig="$(iw dev "$IF" link 2>/dev/null | awk '/signal:/{print $2}')"
  ssid="$(iw dev "$IF" link 2>/dev/null | awk '/[Ss][Ss][Ii][Dd]/{print $NF}')"
  rt="$(ping -c1 -W1 "$WS" 2>/dev/null | awk -F'time=' '/time=/{gsub(/ ms.*/,"",$2);print $2}')"
  t="$(date +%H:%M:%S)"
  echo "$t,${sig:-NA},${ssid:-NA},${rt:-NA}" | tee -a "$CSV"
  n=$((n+1))
  sleep 1
done
echo "============ 汇总（$n 样本）============"
echo "-- RSSI --"
grep -v NA "$CSV" | tail -n +2 | cut -d, -f2 | sort -n | awk '{a[NR]=$1} END{
  if(NR==0){print "无样本"; exit}
  printf "min=%s p10=%s median=%s max=%s\n", a[1], a[int(NR*0.1)+1], a[int(NR/2)], a[NR]}'
echo "-- ping RTT 到 WS(ms) --"
grep -v NA "$CSV" | tail -n +2 | cut -d, -f4 | sort -n | awk '{a[NR]=$1} END{
  if(NR==0){print "无样本"; exit}
  printf "min=%s p50=%s p95=%s max=%s\n", a[1], a[int(NR/2)], a[int(NR*0.95)+1], a[NR]}'
echo "判据：全程 RSSI ≥ -65dBm（-70s 的点记录位置）；缺样/NA 多=掉线漫游，别忽视。"
