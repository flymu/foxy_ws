#!/bin/bash
# r5_replay_regression.sh — R5 冻结回跳/三态健康/解除迟滞 + P0-1 续接 + /initialpose
#                            + B3 规划器门控 的回放回归（foxy-live 容器内，docker exec -d 跑）
# 宿主机用法:
#   sg docker -c 'docker exec -d foxy-live bash /home/mu/foxy_ws/r5_replay_regression.sh'
# 断言（宿主机）: python3 /home/mu/foxy_ws/r5_verify.py
# 五段（串行，总 ~7min）:
#   Run1  mirror+全袋: 三态健康格式 / goal→运动 / DEGRADED注入→零速清目标 /
#                      注入停→不追旧目标 / 新goal→再运动 / /initialpose 重种子 / 袋尾心跳 /
#                      门控恰三关两开+注入窗零重开(11-06 重开迟滞)
#   Run2  drift 30  : 锁死推无回跳(冻结保活) / 解除迟滞 ≥3s / 解除无跳变 / 锁时真实分歧≥0.6m
#   Run2b latedrift 100 8: 狗走远后才锁死推——冻结回跳决定性验证(距锚点>1.0m)
#   Run3a 拒绝续接  : 落盘狗位姿+5m(模拟狗被搬走) vs 袋首狗位姿 → 应拒、走 known-start
#   Run3b 接受续接  : 构造 T_mo=(2,0,0)+狗(0,0,0) vs 袋首狗位姿 → 应续接、位姿可见注入
export ROS_DOMAIN_ID=77
source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash
cd /home/mu/foxy_ws
R=/home/mu/foxy_ws/logs_r5
B=/home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2
P=/home/mu/foxy_ws/install_x86/scan_planner_ros2/lib/scan_planner_ros2
MAP=/home/mu/maps/lio_1792684457_global_map.pcd
BAG=/home/mu/bags/campus_map
mkdir -p $R
rm -f $R/ALL.done

CLEAN() {
  pkill -f "[r]os2 bag play" 2>/dev/null
  pkill -f "[p]oint_lio_node" 2>/dev/null
  pkill -f "[m]ap_localization_node" 2>/dev/null
  pkill -f "[s]can_planner_node" 2>/dev/null
  pkill -f "[d]og_odom_sim" 2>/dev/null
  pkill -f "[r]5_capture" 2>/dev/null
  sleep 1
  rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
}

GOAL_YML="{header: {frame_id: map}, pose: {position: {x: 2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}"

# ============ Run 1: mirror + 门控/健康/重种子（全袋 226s） ============
CLEAN
rm -f $R/rg1_cap.txt $R/rg1_state.txt $R/rg1_t0
: > $R/rg1_t0
nohup $B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $R/rg1_lio.log 2>&1 &
nohup $B/map_localization_node --ros-args -p map_file:=$MAP -p persist_path:=$R/rg1_state.txt \
    > $R/rg1_loc.log 2>&1 &
nohup $P/scan_planner_node --ros-args -r /cmd_vel:=/cmd_vel_intent -p max_vx:=0.4 \
    > $R/rg1_planner.log 2>&1 &
nohup python3 /home/mu/foxy_ws/dog_odom_sim.py mirror > $R/rg1_dog.log 2>&1 &
nohup python3 /home/mu/foxy_ws/r5_capture.py $R/rg1_cap.txt > $R/rg1_cap.log 2>&1 &
sleep 3
nohup ros2 bag play $BAG > $R/rg1_bag.log 2>&1 &
echo "up $(date +%s%3N)" >> $R/rg1_t0
sleep 40                                              # t=40 等定位就绪
timeout 8 ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "$GOAL_YML" >/dev/null 2>&1
echo "goal1 $(date +%s%3N)" >> $R/rg1_t0
sleep 20                                             # t=60 注入 DEGRADED 8s
echo "inj_start $(date +%s%3N)" >> $R/rg1_t0
timeout 8 ros2 topic pub /loc/health std_msgs/msg/String "{data: 'DEGRADED|test_inject|0.0'}" -r 10 \
    > $R/rg1_inj.log 2>&1
echo "inj_end $(date +%s%3N)" >> $R/rg1_t0
sleep 10                                             # t=78 重发目标（期间验证不追旧目标）
timeout 8 ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "$GOAL_YML" >/dev/null 2>&1
echo "goal2 $(date +%s%3N)" >> $R/rg1_t0
sleep 15                                             # t=93 /initialpose 重种子
python3 /home/mu/foxy_ws/r5_seed_pose.py $R/rg1_cap.txt >> $R/rg1_t0 2>&1
echo "seed $(date +%s%3N)" >> $R/rg1_t0
# 等袋放完（余 ~130s）
n=0; while pgrep -f "[r]os2 bag play" >/dev/null && [ $n -lt 100 ]; do sleep 2; n=$((n+1)); done
echo "bag_end $(date +%s%3N)" >> $R/rg1_t0
sleep 12                                             # 心跳观测窗（数据已停）
echo "hb_end $(date +%s%3N)" >> $R/rg1_t0
CLEAN
echo "run1 done" >> $R/rg1_t0

# ============ Run 2: drift 30 — 冻结回跳/迟滞（70s 截断） ============
CLEAN
rm -f $R/rg2_cap.txt $R/rg2_state.txt
: > $R/rg2_t0
nohup $B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $R/rg2_lio.log 2>&1 &
nohup $B/map_localization_node --ros-args -p map_file:=$MAP -p persist_path:=$R/rg2_state.txt \
    > $R/rg2_loc.log 2>&1 &
nohup python3 /home/mu/foxy_ws/dog_odom_sim.py drift 30 > $R/rg2_dog.log 2>&1 &
nohup python3 /home/mu/foxy_ws/r5_capture.py $R/rg2_cap.txt > $R/rg2_cap.log 2>&1 &
sleep 3
nohup ros2 bag play $BAG > $R/rg2_bag.log 2>&1 &
echo "up $(date +%s%3N)" >> $R/rg2_t0
sleep 70                                             # 30s 冻结 + 迟滞 ≥3s + 余量
CLEAN
echo "run2 done" >> $R/rg2_t0

# ============ Run 2b: latedrift 100 8 — 狗走远后才锁死推（冻结回跳决定性验证） ============
CLEAN
rm -f $R/rg2b_cap.txt $R/rg2b_state.txt
: > $R/rg2b_t0
nohup $B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $R/rg2b_lio.log 2>&1 &
nohup $B/map_localization_node --ros-args -p map_file:=$MAP -p persist_path:=$R/rg2b_state.txt \
    > $R/rg2b_loc.log 2>&1 &
nohup python3 /home/mu/foxy_ws/dog_odom_sim.py latedrift 100 8 > $R/rg2b_dog.log 2>&1 &
nohup python3 /home/mu/foxy_ws/r5_capture.py $R/rg2b_cap.txt > $R/rg2b_cap.log 2>&1 &
sleep 3
nohup ros2 bag play $BAG > $R/rg2b_bag.log 2>&1 &
echo "up $(date +%s%3N)" >> $R/rg2b_t0
sleep 140                                            # 100s 起漂 + 8s 漂移 + 迟滞 + 余量
CLEAN
echo "run2b done" >> $R/rg2b_t0

# ============ Run 3a: 拒绝续接（30s 截断） ============
CLEAN
rm -f $R/rg3a_cap.txt
# Run1 落盘的狗位姿 +5m（模拟"狗被搬走过"——bag 走回起点 dp 仅 0.26m 会误走接受分支）
python3 - <<'PYEOF'
import re
src, dst = "/home/mu/foxy_ws/logs_r5/rg1_state.txt", "/home/mu/foxy_ws/logs_r5/rg3a_state.txt"
out = []
for line in open(src):
    p = line.split()
    if p and p[0] == "dog" and p[1] != "none":
        p[1] = f"{float(p[1]) + 5.0:.6f}"
        line = " ".join(p) + "\n"
    out.append(line)
open(dst, "w").writelines(out)
print("[3a] 落盘狗位姿已 +5m（拒绝分支用）")
PYEOF
: > $R/rg3a_t0
nohup $B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $R/rg3a_lio.log 2>&1 &
nohup $B/map_localization_node --ros-args -p map_file:=$MAP -p persist_path:=$R/rg3a_state.txt \
    > $R/rg3a_loc.log 2>&1 &
nohup python3 /home/mu/foxy_ws/dog_odom_sim.py mirror > $R/rg3a_dog.log 2>&1 &
nohup python3 /home/mu/foxy_ws/r5_capture.py $R/rg3a_cap.txt > $R/rg3a_cap.log 2>&1 &
sleep 3
nohup ros2 bag play $BAG > $R/rg3a_bag.log 2>&1 &
echo "up $(date +%s%3N)" >> $R/rg3a_t0
sleep 30
CLEAN
echo "run3a done" >> $R/rg3a_t0

# ============ Run 3b: 接受续接（30s 截断） ============
CLEAN
rm -f $R/rg3b_cap.txt
WALL=$(date +%s)
printf '# loc_persist v1\nmap_file %s\nwall %s\nt_mo 1 0 0 2.0 0 1 0 0.0 0 0 1 0.0\ndog 0 0 0 0 0 0 1\n' \
    "$MAP" "$WALL" > $R/rg3b_state.txt
: > $R/rg3b_t0
nohup $B/point_lio_node --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $R/rg3b_lio.log 2>&1 &
nohup $B/map_localization_node --ros-args -p map_file:=$MAP -p persist_path:=$R/rg3b_state.txt \
    > $R/rg3b_loc.log 2>&1 &
nohup python3 /home/mu/foxy_ws/dog_odom_sim.py mirror > $R/rg3b_dog.log 2>&1 &
nohup python3 /home/mu/foxy_ws/r5_capture.py $R/rg3b_cap.txt > $R/rg3b_cap.log 2>&1 &
sleep 3
nohup ros2 bag play $BAG > $R/rg3b_bag.log 2>&1 &
echo "up $(date +%s%3N)" >> $R/rg3b_t0
sleep 30
CLEAN
echo "run3b done" >> $R/rg3b_t0
echo done > $R/ALL.done
