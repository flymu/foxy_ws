# Go2 校园巡检平台 · 工作站侧（foxy_ws）

Unitree Go2 EDU + Jetson Orin Nano（狗载，只跑传感器驱动和桥）+ Livox Mid360 + 工作站（Docker ROS2 Foxy，跑 LIO / 定位 / 规划 / 可视化）。本仓库是**工作站侧**的节点、脚本与配置。

## 目录速览

| 内容 | 文件 |
|---|---|
| 狗控制桥（工作站→cmd_vel→SDK2 RPC） | `go2_bridge.py` `go2_bridge_start.sh`（部署到 Orin）`motion_test.py` |
| 四足解算位姿桥（固件 sportmodestate→/go2/odom，开发中） | `go2_odom_bridge.py` |
| 相机接入（前摄拍照 RPC + RTSP 流桥） | `go2_photo_test.py` `go2_rtsp_test.py` `go2_camera_bridge.py`（Orin）`go2_camera_republish.py`（工作站）`go2_videohub_probe.py` `con_notify_probe.py` `probe_proto.py` `probe2.py` `describe_rtsp.py` |
| 雷达启停 / 录包 / 建图 | `start_livox.sh` `stop_livox.sh` `record_bag.sh` `point_lio_start.sh` `test_point_lio.sh` `run_combined_test.sh` |
| LIO / NDT 定位 / 规划（自研 ROS2 包装） | `point_lio_ros2/` `scan_planner_ros2/` `src/livox_ros_driver2`（msg-only 补丁版） |
| 精度评估 / 地图绘图 / 可视化辅助 | `eval_loc.py` `plot_map.py` `prep_map_viz.py` `viz_helper.py` |
| RViz 巡逻台配置 | `live_stack.rviz`（2D Goal Pose 遛狗 + 相机画面）`livox.rviz` `point_lio.rviz` |
| DDS 传输配置 | `fastdds_udp_only.xml` `cyclonedds_eth1.xml` |
| 网络侦察工具 | `scan_ports.py` |

## 相机两条数据通路（无需云端密钥）

- 拍照（1920×1080 JPEG）：SDK2 DDS RPC，`videohub` 服务 api 1001 `GetImageSample`
- 实时流：`rtsp://<狗主板>:8551/video`（H.264 720p），Orin 上 `go2_camera_bridge.py` 拉流转 JPEG 话题

## 注意

- 大文件不入库：bag、地图 PCD、构建产物（见 `.gitignore`）
- 视频链路 QoS 全 best_effort（reliable 大消息跨 WiFi UDP 会静默卡死订阅方）
