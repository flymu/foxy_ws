#!/bin/bash
# e2e_replay_run.sh — G1 端到端 bag 回放验证（在 foxy-live 容器内跑）
# 用法（宿主机，-d 完全脱离会话——普通 docker exec/nohup 会被会话 SIGTERM 连坐杀）:
#   sg docker -c 'docker exec -d foxy-live bash /home/mu/foxy_ws/e2e_replay_run.sh'
# 完成标记 /tmp/e2e.done；宿主机轮询后取结果:
#   sg docker -c 'docker exec foxy-live cat /tmp/e2e.test.log'
# 链路: bag(/livox/lidar CustomMsg + /livox/imu) → livox_relay → /livox/points2
#       → obstacle_veto(remap 输出→/veto/cmd_out, 关戳闸因 bag 是旧 epoch)
#       ← test_veto_offline.py 50Hz 假规划器 /cmd_vel_raw + 记录器
# 退出码: 0=PASS 1=断言失败 2=未收到 /veto/cmd_out 3=前 2s 无云期未全零(熔断失效)
export ROS_DOMAIN_ID=77          # 隔离域，防真机订阅者串台
source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash
cd /home/mu/foxy_ws
L=/tmp/e2e
rm -f $L.done
pkill -f relay_node 2>/dev/null; pkill -f obstacle_veto 2>/dev/null; pkill -f "ros2 bag" 2>/dev/null
sleep 1
nohup ros2 run livox_relay relay_node > $L.relay.log 2>&1 &
nohup python3 obstacle_veto.py --ros-args -r /go2/cmd_vel:=/veto/cmd_out -p stamp_check:=false > $L.veto.log 2>&1 &
sleep 3
nohup ros2 bag play /home/mu/bags/campus_map --rate 2.0 > $L.bag.log 2>&1 &
sleep 2
# 测试 125s 自然退出自印汇总；timeout 135 只是保险网（别贴着赛跑）
timeout -s INT 135 python3 test_veto_offline.py 125 > $L.test.log 2>&1
echo "TEST_EXIT=$?" >> $L.test.log
# 收尾清干净（bag 2.0x≈113s 早已自然播完）
pkill -INT -f obstacle_veto 2>/dev/null; pkill -f relay_node 2>/dev/null; pkill -f "ros2 bag" 2>/dev/null
sleep 1
pkill -9 -f obstacle_veto 2>/dev/null
echo done > $L.done
