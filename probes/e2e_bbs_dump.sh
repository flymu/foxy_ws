#!/bin/bash
# e2e_bbs_dump.sh — BBS Phase 0b：campus_map 回放采真扫描（foxy-live 容器内，docker exec -d 跑）
# 用法: sg docker -c 'docker exec -d foxy-live bash /home/mu/foxy_ws/e2e_bbs_dump.sh'
# 产物: /home/mu/bags/bbs_eval/scan_t*.pcd + meta.json；完成标记 /tmp/bbsdump.done
# 之后主机侧跑: python3 /home/mu/workspace/3d_bbs/local_demo/bbs_eval_report.py
export ROS_DOMAIN_ID=77
source /opt/ros/foxy/setup.bash
source /home/mu/foxy_ws/install_x86/setup.bash
cd /home/mu/foxy_ws
D=/tmp/bbsdump
rm -f $D.done
rm -rf /home/mu/bags/bbs_eval
pkill -f point_lio_node 2>/dev/null; pkill -f bbs_scan_dump 2>/dev/null; pkill -f "ros2 bag" 2>/dev/null
sleep 1
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
nohup /home/mu/foxy_ws/install_x86/point_lio_ros2/lib/point_lio_ros2/point_lio_node \
    --ros-args -p config_file:=/home/mu/workspace/Point-LIO/config/mid360.yaml \
    -p map_save_path:=/tmp > $D.lio.log 2>&1 &
nohup python3 /home/mu/foxy_ws/probes/bbs_scan_dump.py > $D.dump.log 2>&1 &
sleep 3
nohup ros2 bag play /home/mu/bags/campus_map > $D.bag.log 2>&1 &
# dump 节点按窗自退（bag 226s@1.0x，末窗 200s+1s+余量）；保险网 300s
timeout 300 bash -c 'until grep -q "完成" /tmp/bbsdump.dump.log 2>/dev/null; do sleep 5; done'
echo "DUMP_EXIT=$?" >> $D.dump.log
pkill -INT -f point_lio_node 2>/dev/null; pkill -f bbs_scan_dump 2>/dev/null; pkill -f "ros2 bag" 2>/dev/null
sleep 2
pkill -9 -f point_lio_node 2>/dev/null
echo done > $D.done
