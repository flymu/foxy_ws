#!/bin/bash
# push_safety.sh — 把工作站上的安全栈最新副本推到 Orin（2026-11-05 G1 起）
# 用法: bash orin_deploy/push_safety.sh
# 背景：Orin 上 /home/unitree/ 的运行副本靠手工 scp 维护，狗断电时改完本地
# 容易忘推——下次实弹跑的还是旧逻辑。跑本脚本对齐后再上电测试。
# 铁律不变：Orin 侧只许 unitree 用户跑这些文件（root 跑 DDS 毒化 SHM 发现）。
set -e
cd "$(dirname "$0")/.."
FILES="obstacle_veto.py veto_core.py veto_viz.py cloud_sparsify.py \
go2_bridge.py bridge_core.py go2_avoid_switch.py veto_status_snoop.py tf_snoop.py \
creep_test.py"
EXTRA="orin_deploy/livox_watchdog.sh orin_deploy/dds_bomb_guard.cpp"
echo "== 推送: $FILES $EXTRA"
scp -q $FILES $EXTRA go2_wifi:/home/unitree/

# dds_bomb_guard.so — XCDR2 毒包护栏（11-07 起 Orin 系统级常驻）。
# 11-06 案卷：狗主板 24 个 Cyclone 服务的 SEDP 0x75 TypeInformation 可令无校验的
# fastcdr1.0.13 请求 100~1065GB 单笔分配（WS 曾 OOM×2；11-07 Orin relay 冻死同嫌）。
# 编译进 /usr/local/lib 并写 /etc/ld.so.preload → 之后 Orin 所有新进程免疫。
# 已装则跳过（幂等）；改护栏源码后需手动重装。sudo 密码经脚本 stdin 传入（不进仓库）。
if ! ssh go2_wifi 'grep -q dds_bomb_guard.so /etc/ld.so.preload 2>/dev/null'; then
  echo "== 安装 DDS 毒包护栏（需要输入 Orin 的 sudo 密码）"
  ssh go2_wifi 'g++ -shared -fPIC -O2 -o /tmp/dds_bomb_guard.so /home/unitree/dds_bomb_guard.cpp'
  ssh go2_wifi 'sudo -S install -m 644 /tmp/dds_bomb_guard.so /usr/local/lib/dds_bomb_guard.so && sudo sh -c "echo /usr/local/lib/dds_bomb_guard.so > /etc/ld.so.preload" && sync && grep . /etc/ld.so.preload'
fi

echo "== 完成。下次实弹前的开机检查单："
echo "  详见 orin_deploy/POWERON_CHECKLIST.md，顺序概要："
echo "  0. DDS 毒包护栏系统级常驻（/etc/ld.so.preload），本脚本自动保证，无需手工"
echo "  1. ssh go2_wifi 'bash -s' < start_livox.sh                     # 雷达"
echo "  2. ssh go2_wifi 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'"
echo "  3. ssh go2_wifi 'python3 /home/unitree/go2_avoid_switch.py get'  # 自带避障必须 off"
echo "  4. ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]' # relay+veto(G1)"
echo "  5. 桥: ssh go2_wifi 'bash /home/unitree/go2_bridge_start.sh'      # v4 停车锁存版"
echo "  6. 狗唤醒后 known-start/续接 重定位（point_lio 位姿已脏）"