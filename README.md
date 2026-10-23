# Go2 校园巡检平台 · 工作站侧（foxy_ws）

Unitree Go2 EDU + Jetson Orin Nano（狗载，只跑传感器驱动和桥）+ Livox Mid360 + 工作站（Docker ROS2 Foxy，跑 LIO / 定位 / 规划 / 可视化）。本仓库是**工作站侧**的节点、脚本与配置。

## 目录速览

| 内容 | 文件 |
|---|---|
| 狗控制桥（工作站→cmd_vel→SDK2 RPC） | `go2_bridge.py` `go2_bridge_start.sh`（部署到 Orin）`motion_test.py` |
| 四足解算位姿桥（固件 /lf/sportmodestate→`/go2/odom`） | `go2_odom_bridge.py`（Orin，rclpy 直订 FastDDS，best_effort）`dog_odom_sim.py`（离线验证器） |
| 巡逻栈一键启停（foxy-live 容器内） | `live_stack_start.sh` `live_stack_stop.sh` `pose_dump.py`（位姿连续性检查） |
| 相机接入（前摄拍照 RPC + RTSP 流桥） | `go2_photo_test.py` `go2_rtsp_test.py` `go2_camera_bridge.py`（Orin）`go2_camera_republish.py`（工作站）`go2_videohub_probe.py` `con_notify_probe.py` `probe_proto.py` `probe2.py` `describe_rtsp.py` |
| 雷达启停 / 录包 / 建图 | `start_livox.sh` `stop_livox.sh` `record_bag.sh` `point_lio_start.sh` `test_point_lio.sh` `run_combined_test.sh` |
| LIO / NDT 定位 / 规划（自研 ROS2 包装） | `point_lio_ros2/` `scan_planner_ros2/` `src/livox_ros_driver2`（msg-only 补丁版） |
| FAST_LIO（hku-mars 基础上的本地版：IMU 无姿态导出兼容、离线 HBA 扫描导出工具） | `FAST_LIO/`（源码快照，未含 doc/PCD/build，GPL-2.0） |
| HBA 离线回环优化（hku-mars 基础上的本地修改） | `HBA/`（源码快照，未含 data/build，GPL-2.0）；gtsam 4.1.1 已附带在 `HBA/3rdparty/gtsam`（源码快照，剔除 .git 与构建产物，BSD） |
| 精度评估 / 地图绘图 / 可视化辅助 | `eval_loc.py` `plot_map.py` `prep_map_viz.py` `viz_helper.py` |
| RViz 巡逻台配置 | `live_stack.rviz`（2D Goal Pose 遛狗 + 相机画面）`livox.rviz` `point_lio.rviz` |
| DDS 传输配置 | `fastdds_udp_only.xml` `cyclonedds_eth1.xml` |
| 网络侦察工具 | `scan_ports.py` |

## 相机两条数据通路（无需云端密钥）

- 拍照（1920×1080 JPEG）：SDK2 DDS RPC，`videohub` 服务 api 1001 `GetImageSample`
- 实时流：`rtsp://<狗主板>:8551/video`（H.264 720p），Orin 上 `go2_camera_bridge.py` 拉流转 JPEG 话题

## 四足航位推算看门狗（定位防飞走）

定位节点把狗固件的运动学+IMU 航位推算（不依赖雷达）纳入裁决：滚动比较 LIO 与狗
各自最近 2s 的位移/偏航差分，分歧超阈值 → 判 LIO 病态，`/loc/pose` 切换为
「最后一致锚点 + 狗航位推算增量」死推，停子图累积（NDT 自然拒绝脏数据）；
窗口重新一致后无跳变交还 LIO+NDT。狗里程计复位（standup/recovery 重置原点）自动
识别重建基准，位姿不断裂。健康状态看 `/loc/health`（OK / DEAD_RECKONING）。
触发过的真实场景：livox 驱动长跑发病喂脏点云，LIO 里程计瞬间漂 28m。
离线验证：`dog_odom_sim.py` 四场景（mirror 回归 / drift 切换+解除 / glitch 复位
不误切 / latchtele 死推中复位保持连续），`pose_dump.py` 复查位姿连续性。

## 注意

- 大文件不入库：bag、地图 PCD、构建产物（见 `.gitignore`）
- 视频链路 QoS 全 best_effort（reliable 大消息跨 WiFi UDP 会静默卡死订阅方）
