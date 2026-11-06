# Go2 校园巡检平台 · 工作站侧（foxy_ws）

开发入口（2026-11-05）：[自主导航恢复计划](DEVELOPMENT_PLAN.md) · [工程 review 与已复现问题](docs/REVIEW_2026-11-05.md)。下文部分历史实验步骤需要按新计划完成部署与回归校验。

Unitree Go2 EDU + Jetson Orin Nano（狗载，只跑传感器驱动和桥）+ Livox Mid360 + 工作站（Docker ROS2 Foxy，跑 LIO / 定位 / 规划 / 可视化）。本仓库是**工作站侧**的节点、脚本与配置。

## 目录速览

| 内容 | 文件 |
|---|---|
| 狗控制桥（工作站→cmd_vel→SDK2 RPC） | `go2_bridge.py` `go2_bridge_start.sh`（部署到 Orin）`archive/motion_test.py` |
| 四足解算位姿桥（固件 /lf/sportmodestate→`/go2/odom`，两段接力） | `go2_odom_bridge.py`（Orin，第1段：cyclone 钉 eth1 直订主板）`go2_odom_republish.py`（第2段：默认 FastDDS 栈跨栈透传给工作站）`go2_odom_start.sh`（两段一键启停）`dog_odom_sim.py`（离线验证器） |
| 巡逻栈一键启停（foxy-live 容器内） | `live_stack_start.sh` `live_stack_stop.sh` `probes/pose_dump.py`（位姿连续性检查） `doctor.py`（全链路只读体检：各级速率/健康语义，异常非零退出） |
| **独立避障否决层**（最后防线，P0-S） | `obstacle_veto.py`（数据流：`/cmd_vel_intent`→speed_adapter→`/cmd_vel_raw`→否决→`/go2/cmd_vel`）`veto_core.py`（G1 无 ROS 决策核心：R1 真距离/R2 不增速/R3 戳闸）`obstacle_veto_cpp/`（C++ 移植 colcon 包，影子对拍已过零 diff，见否决层章节）`veto_shadow_start.sh`/`veto_shadow_diff.py`（C++ 影子启动与对拍工具，Orin 侧）`speed_adapter.py`（运动适配器：起步地板 0.25/停步死区，G1 起 veto 不再越权抬速）`bridge_core.py` + `go2_bridge.py` v4（R4 停车锁存仲裁）`test_veto_regression.py`（R1-R4 无 ROS 回归，53 断言）`test_veto_offline.py`（bag 回放集成测试，非零退出码）`e2e_replay_run.sh`（E2E 回放链一键 runner，docker exec -d 脱离会话）`orin_deploy/push_safety.sh`（G1 副本一键推 Orin，狗断电期间改完防忘推）`probes/veto_calib_probe.py`（安装几何标定）`probes/capture_frames.py`（原始帧采集）`veto_status_snoop.py`（Orin 上拉 /veto/status——daemon 瞎）`tf_snoop.py` `probes/veto_selfmask_probe.py`（体素掩膜法，已弃用留档）`probes/probe_twist_stream.py` `archive/probe_observer.py`（DDS 流诊断探针，50Hz 流"断流"定责用）
`probes/peek_status.py`（String 话题旁观器，/veto/status /loc/health 快速回读）`probes/peek_imu.py`（/livox/imu 旁听：频率/模长/与先验法线夹角） |
| **R5 定位批回归**（冻结回跳/三态健康/解除迟滞/P0-1 续接//initialpose 重种子/B3 规划器门控） | `r5_replay_regression.sh`（五段 bag 回放一键，**必须 docker exec -d**）→ 宿主机 `python3 r5_verify.py` 断言（R5-1 冻结无回跳 / R5-2 健康 1Hz 心跳数据停后仍在 / R5-3 解除迟滞≥3s / R5-4 拒绝与接受续接 / B3 注入 DEGRADED→清目标零速、恢复不追旧目标）；`r5_capture.py`（采集节点）、`r5_seed_pose.py`（/initialpose 注入器）、`dog_odom_sim.py` |
| 上电部署件（Orin） | `orin_deploy/push_safety.sh`（安全栈+看门狗一键推）`orin_deploy/livox_watchdog.sh`（驱动长跑塌方自愈：速率塌→SIGINT+清SHM+重拉）`orin_deploy/POWERON_CHECKLIST.md`（上电→可信巡逻检查单） |
| 相机接入（前摄拍照 RPC + RTSP 流桥） | `go2_photo_test.py` `go2_rtsp_test.py` `go2_camera_bridge.py`（Orin）`go2_camera_republish.py`（工作站）`probes/go2_videohub_probe.py` `probes/con_notify_probe.py` `archive/probe_proto.py` `archive/probe2.py` `probes/describe_rtsp.py` |
| 雷达启停 / 录包 / 建图 | `start_livox.sh` `stop_livox.sh` `record_bag.sh` `point_lio_start.sh` `test_point_lio.sh` `run_combined_test.sh` |
| LIO / NDT 定位 / 规划（自研 ROS2 包装） | `point_lio_ros2/` `scan_planner_ros2/` `src/livox_ros_driver2`（msg-only 补丁版） |
| FAST_LIO（hku-mars 基础上的本地版：IMU 无姿态导出兼容、离线 HBA 扫描导出工具） | `FAST_LIO/`（源码快照，未含 doc/PCD/build，GPL-2.0） |
| HBA 离线回环优化（hku-mars 基础上的本地修改） | `HBA/`（源码快照，未含 data/build，GPL-2.0）；gtsam 4.1.1 已附带在 `HBA/3rdparty/gtsam`（源码快照，剔除 .git 与构建产物，BSD） |
| 精度评估 / 地图绘图 / 可视化辅助 | `probes/eval_loc.py` `plot_map.py` `probes/prep_map_viz.py` `viz_helper.py` |
| RViz 巡逻台配置 | `live_stack.rviz`（2D Goal Pose 遛狗 + 相机画面）`livox.rviz` `point_lio.rviz` |
| DDS 传输配置 | `fastdds_udp_only.xml` `cyclonedds_eth1.xml` |
| 网络侦察工具 | `scan_ports.py` |
| 导航操作工具 | `g4_goal_pub.py`（发目标：**先等 get_subscription_count>0 再发**——毒包风暴下规划器参与者约 1-2 分钟后不再接新 pub 匹配，发不进就重启 planner 再发）`initialpose_pub.py`（/initialpose 重种子，**必须 PoseWithCovarianceStamped**，发 PoseStamped 是静默不投递）`goal_loop.py`（巡航循环）`creep_test.py`（蠕行测试）`go2_avoid_switch.py`（自带避障开关：每次狗上电复位成开，实弹前必查必关；**2026-11-06 起在 Orin 上 segfault 待修**） |
| 只读诊断探针（54 个，`probes/`） | 一次性只读诊断全在此：`health_pose_probe.py`（健康+位姿一行）`hz_any.py`/`lag_probe.py`（频率/戳龄）`self_cluster_probe.py`/`cloud_dogcell_probe.py`（点云自簇/狗格解剖）`match_probe.py`（DDS 端点匹配判定）`veto_calib_probe.py`（安装几何标定）`bbs_*`（3D-BBS 冷启动采集/种子）`wire_probe.py`+`wire_logger.py`+`analyze_dual.py`（双通道对照）`peek_*`（电池/IMU/状态旁听）`api_snoop.py`/`discover_topics.py`（总线侦察）等 |
| 结案考古（14 个，`archive/`） | 已结案不再维护：SDK"权限门"调查时代（`sport_lease_probe.py`/`ms_lease_probe.py`/`speed_gate_probe.py`/`select_mode_test.py`/`stack_probe2.py` 等——结论：无权限门，狗起步阈值 0.20-0.25）与 nav/camera 时代一次性探针 |

## 相机两条数据通路（无需云端密钥）

- 拍照（1920×1080 JPEG）：SDK2 DDS RPC，`videohub` 服务 api 1001 `GetImageSample`
- 实时流：`rtsp://<狗主板>:8551/video`（H.264 720p），Orin 上 `go2_camera_bridge.py` 拉流转 JPEG 话题

## 四足航位推算看门狗（定位防飞走）

定位节点把狗固件的运动学+IMU 航位推算（不依赖雷达）纳入裁决：滚动比较 LIO 与狗
各自最近 2s 的位移/偏航差分，分歧超阈值 → 判 LIO 病态，`/loc/pose` 切换为
「最后一致锚点 + 狗航位推算增量」死推，停子图累积（NDT 自然拒绝脏数据）；
窗口重新一致后无跳变交还 LIO+NDT。狗里程计复位（standup/recovery 重置原点）自动
识别重建基准，位姿不断裂。健康状态看 `/loc/health`（R5 起为三态式，见下）。
触发过的真实场景：livox 驱动长跑发病喂脏点云，LIO 里程计瞬间漂 28m（NDT 闸门兜住）；
2026-10-31 真机实弹——雷达空云（eff=5）把 LIO 喂飞 23m，看门狗反复切入死推（计数至
第10次），`/loc/pose` 全程钉在原地不动，下游未被带飞（期间 latch/unlatch 震荡，待加解除迟滞）。

## R5 定位修复批 + B3 规划器门控 + P0-1 断电续接（2026-11-05，评审 R5/R7 前 4 项）

评审（docs/REVIEW_2026-11-05.md）四缺陷全修，源 `point_lio_ros2/src/map_localization_node.cpp`
与 `scan_planner_ros2/src/scan_planner_node.cpp`：

- **R5-1 冻结回跳**：死推期无狗流时旧代码回跳到**初始锚点**（狗走远后输出瞬移回起点）
  → 改为保存并保持"最后发布的死推位姿" `T_mb_dr_last_`。
- **R5-2 三态健康 + 1Hz 心跳**：`/loc/health` 从数据流驱动的两态改成 1Hz 墙钟心跳的
  `STATE|reason|age_s`（INITIALIZING/OK/STALE/DEGRADED/LOST）——数据停后健康**仍在说**
  LOST/STALE 而不是沉默（下游才能区分"断流"与"没事"）。
- **R5-3 解除迟滞（P0-3）**：狗/LIO 重新一致后需 **连续 30 窗且跨度≥3s** 才解除死推
  （旧 10 窗≈1s，latch/unlatch 震荡）；狗流断供 >0.5s 重建狗缓冲。
- **P0-1 断电续接**：每 5s 落盘 `(T_mo, 狗位姿)` 到 `persist_path`
  （默认 `/home/mu/maps/loc_state.txt`，只落健康态）；下次上电狗位姿在 0.5m/0.3rad 内
  → 直接续接 T_mo 跳过 known-start；超差（狗挪过窝/里程计复位）→ WARN 维持 known-start。
- **/initialpose 重种子（T_mo 注入 API）**：RViz "2D Pose Estimate"（和 BBS 全局定位
  种子助手）等价入口——x/y/yaw 取消息、z/roll/pitch 沿用当前，`T_mo=T_mb·T_ob⁻¹`，
  死推期自动忽略（死推位姿是对的），agree 锚点作废由 NDT 精修接管。
- **B3 规划器准入门控**：`scan_planner_node` 订 `/loc/health`，**fail-closed**：
  非 OK / 健康流超时 2.5s / loc 位姿超龄 2.0s → 清目标（新增核心
  `ScanPlanner::clearTarget()`）+ 显式零速 100Hz 下发 + 不喂里程计不 step + 不喂地图；
  恢复后 **不自动追旧目标**（goal 复位 -1e9，等操作员重发）且须 OK 持续 ≥1s
  （11-06 迟滞，见下）；位姿外推封顶 0.25s
  （旧代码按旧速度无限外推）。

回归：`r5_replay_regression.sh` 五段 bag 回放（~7min，docker exec -d）→
`python3 r5_verify.py` 断言上述行为（含注入 DEGRADED 的门控响应与续接正反例）。

### 11-06 加固（回归首轮复盘出的两处真缺陷）

- **看门狗假锁（mirror 假阳性）**：快走段 LIO 帧对速度 1.53m/s 刚过旧阈值 1.5 → 误报
  "狗里程计复位"重建基准 → 重建后狗窗 1.2s vs LIO 窗 2.0s **不等长比较**，行走位移被
  窗口差顶爆 → mirror 全程一致却在袋 t≈200s 被假锁"位移分歧"3.2s（真机=巡逻中无辜急停）。
  修复：复位阈值 1.5→**4.0 m/s**（真实复位是米级瞬移；Go2 正常极速≈2m/s）+
  LIO 同跳排除（狗/LIO 同跳=真实运动或 LIO 修正，不算复位）+
  **狗/LIO 窗口墙钟等长比较**（跨流匹配必须走墙钟——狗钟与 livox 钟不同源，
  狗 stamp 不能索引 LIO 缓冲；`StampedPose` 增加接收墙钟字段）。
- **门控重开迟滞**：健康在 OK/DEGRADED 边界抖动时门控 1Hz 反复开关（每沿 clearTarget），
  狗会顿挫 → 重开须 **OK 连续保持 ≥1s**（`reopen_hold`，默认 1.0）；关闭方向无迟滞。
  回归注入窗（DEGRADED@10Hz 叠加真 OK@1Hz）实测旧版 11 关 10 开、新版恰 3 关 2 开。
- **standalone 库日志级别**：默认 WARN 吞掉 `[clearTarget]` INFO 生效日志 →
  包装节点 `set_log_level(INFO)`（现场排"不走"必须能看到）。
- **/initialpose 语义注**：健康跟踪下错误种子会被 NDT 当帧精修拉回真值（位姿不跳是
  **设计行为**，防 RViz 误点瞬移）；种子冷启动效果由 Run3b（T_mo 注入可见）与
  BBS Phase 0b 覆盖。回归 34 断言全绿。
狗里程计通路为两段接力：主板 DDS 只认 CycloneDDS 且钉 eth1（FastDDS 怎么配都收不到），
该参与者又只在 eth1 多播、工作站发现不了，故由 Orin 上默认 FastDDS 栈的
`go2_odom_republish.py` 跨栈透传一次，两段一键启停 `go2_odom_start.sh`。
离线验证：`dog_odom_sim.py` 四场景（mirror 回归 / drift 切换+解除 / glitch 复位
不误切 / latchtele 死推中复位保持连续），`probes/pose_dump.py` 复查位姿连续性。

## 独立避障否决层 obstacle_veto（撞车事故防线，2026-11-03）

背景：规划器障碍图唯一数据源是实时点云，驱动病时静默冻结、定位游走时障碍图错位，
且 SportClient.Move 旁路了狗自带 AI 避障——多次撞家具，无独立防线兜底。

本节点是**最后一道闸**：只看原始雷达点云 + 静态安装几何，独立于
LIO/定位/规划器。数据流 `scan_planner -r /cmd_vel:=/cmd_vel_intent` →
`speed_adapter`（G1 起：起步地板/停步死区在这里）→ `/cmd_vel_raw` → veto → `/go2/cmd_vel`。

**G1 修复批（2026-11-05，评审 R1-R4 全落地，53 项回归全绿）**：
- **R1 距离压缩**：旧式在倾斜传感器系里取 hypot(x,y)，前向距离被压 ×cos47°≈0.68
  （实测 veto 读 0.62 vs 真值 0.91，胶带复核）→ 半径其实等效 1.47m，狗 0.9m 外
  对无害物刹停。修正为正交水平基 `hypot(vh·f, vh·l)`，半径 1.0 现在就是真 1.0m。
- **R2 提权违规**：旧版把消解后的残速"抬回"0.25（放大越权）→ 删除。残速低于
  起步阈值一律归零（保守停车）；起步地板移交给 speed_adapter（适配在前、否决在后）。
- **R3 时效盲区**：旧版"刚收到"当"刚采集"，旧帧/重复帧能无限续命 → 加戳闸
  （采集年龄 >2.5s 拒 + 戳不前进拒；回放模式 `-p stamp_check:=false`）。
- **R4 停车竞争**：桥的 stop 和速度共用可覆盖槽位 → bridge_core.CmdArbiter
  （hi 指令通道永远优先、stop 即刻锁存 halt 直到 resume/姿态指令、看门狗瞬态不锁存）。
- 决策逻辑抽成 `veto_core.py`/`bridge_core.py`（无 ROS 依赖），
  `test_veto_regression.py` 53 项断言离线可跑（多倾角×多方位×已知距离、
  不放大性质测试、戳闸、停车锁存），失败非零退出。
- **E2E bag 回放实测（e2e_replay_run.sh，隔离 domain+关戳闸）**：50Hz 假规划器
  恒推 0.25m/s，输出满 50Hz；拦截 90 拍中位距离 **0.91m（贴合胶带复核真值 0.7-0.9m）**；
  首 2s 无云期 100% 零（早启熔断）、bag 停后 100% 零（STALE 熔断），TEST_EXIT=0。
- `orin_deploy/push_safety.sh`：G1 副本一键 scp 到 Orin——狗断电期间本地改完防忘推，
  **下次上电先跑它再实弹**。

**C++ 移植（2026-11-06 影子先行，已到影子对拍阶段）**：`obstacle_veto_cpp/`
（colcon 包：`veto_core.hpp` 纯决策核心 + `obstacle_veto_node.cpp` 全状态机节点
+ `test/veto_core_test.cpp`——Python 53 断言中 R1-R3 38 项的同数值移植；
R4 仲裁 15 项属 bridge 侧不移植）。两侧同断言集全绿=单元级对拍成立
（WS 容器与 Orin ARM 双构建均 38/38，Python 侧 53/53 未动）。
影子阶段实弹对照（Orin 真数据、桥已死零运动风险）：`veto_shadow_start.sh`
（Orin 上起 C++ 影子，输出全部 remap `*_shadow` 不碰生产链）+
`veto_shadow_diff.py`（/cmd_vel_raw 正弦扫描激励，两版输出 **wz 指纹配对**
逐对 diff——按时刻就近配对会把 adapter 100Hz 零速流错配成 wz-only 假差异，
第一轮 1620 个假 mismatch 的教训）。实测 5863 对 **零 mismatch**
（现场真实 0.19m 障碍持续 BLOCK_SECTOR），标定重试风暴两侧逐字节同构
（dev≈1.2°/h=0.22 同被校验门拒）。效率：同负载同数据流 30s 平均
**Python 0.60 核 vs C++ 0.29 核**。切换前待办：行走态实弹复测（对照
E2E 胶带真值 0.91m 基线）→ `orin_deploy/push_safety.sh` 改推 C++。

- 几何：Mid360 尾部桅杆**前下斜装 47°**（n_up=[-0.733,0.0001,0.680]，h=0.486m，
  probes/veto_calib_probe.py 从 campus_map 静止段标定）；频带离地 **0.20-0.75m**（下沿
  0.20：场地坡度+标定残差 ±4cm 会把地板壳漏进 0.05-0.15，净空处也永远拦截——
  11-03 实弹诊断）；前方 ±60° <1.0m 拦截（R1 修正后=真米数）、侧后 <0.55m 拦截，
  只消"朝向障碍"的速度分量（保沿墙滑行/脱离），原地转向永不拦（逃生），
  <0.30m 全消平动。
- 自体掩膜：传感器正下方小方盒（桅杆座/背的紧簇，行走+静止双验证）；腿/侧腹
  自遮挡零回波。体素持久性掩膜法会混入起点环境（51%），弃用。
- 熔断：>1s 无有效云（点数≥2000、散布≥1m、**采集戳新鲜且前进**三判据，
  防空云假新鲜/旧帧续命）→ 全零。节点死 = cmd 流断 = 桥 0.5s 看门狗停车，失效安全。
- QoS：cmd 链全 BEST_EFFORT depth 1（最新指令赢；RELIABLE 重传在执行器被雷达
  反序列化拖住时无意义）。**go2_bridge.py 的 /go2/cmd_vel 订阅已同步改 BE，
  Orin 上的部署副本需 rsync 更新后 veto 才能生效**（BE pub 对 RELIABLE sub
  QoS 不匹配 = 狗收不到指令）。
- 输出率：fresh 时 ~15-25Hz（单线程执行器被 640KB 雷达帧反序列化限速），
  对 0.5s 桥看门狗和 50ms 级拦截延迟无碍；P1 优化改订 PointCloud2 +
  np.frombuffer 可回 50Hz。
- 标定（IMU 重力对齐 + 地板锚，2026-11-03）：Mid360 内置 IMU（`/livox/imu`
  ~181Hz，**g 单位**，静止读数=+g 向上）开机静止窗 ~2s 直接给出法线初值——
  趴着也给得对，不依赖地板可见性；地板 SVD 精修当"锚"（定 h、定坡道上的
  真地板方向，校验门 dev<12° 且 h∈0.35-0.68 站立域——坐/趴姿态的错几何会被
  拒，h=0.309 教训），且**只在 IMU 判准静止时采帧/精修**（"开机=静止"推广成
  "任何静止时刻"，运动中的帧不采）。锚后 IMU 增量跟踪倾斜（坡道/步态俯仰
  实时补偿）；倾斜累计>5°（坐/趴/搬动）或静置>120s 自动重修，6 连败后每
  120s 再给一轮（狗趴了再站起能自愈，无需手动 recalib）。实测：趴着启动
  IMU 对齐与先验差 1.1°，站起后地板精修 dev=0.4° h=0.472，三源自洽。
  `ros2 topic pub /veto/cmd std_msgs/String "{data: recalib}"` 手动重标、
  `"{data: prior}"` 回退先验。状态行 `imu=ok a=0.98 q=1 slope=0.4`：
  slope=地板法线与重力夹角（≈地面坡度）。
- 离线测试（狗不上电，G1 后链路变了——veto 吃 `/livox/points2` 而 bag 里只有
  CustomMsg，必须有 relay；bag 戳是旧 epoch，必须关戳闸；隔离 domain 防真机串台）：
  1) WS 容器构建并启动 relay：`colcon build --packages-select livox_relay`（一次性）
     然后 `ros2 run livox_relay relay_node`
  2) `ROS_DOMAIN_ID=77 ros2 bag play /home/mu/bags/campus_map`
  3) `ROS_DOMAIN_ID=77 python3 obstacle_veto.py --ros-args
     -r /go2/cmd_vel:=/veto/cmd_out -p stamp_check:=false`
  4) `ROS_DOMAIN_ID=77 timeout -s INT 260 python3 test_veto_offline.py`
  一键版（推荐，**必须 docker exec -d 脱离会话**——普通 exec/nohup 的后台任务
  会被宿主会话的 SIGTERM 连坐杀树）:
    sg docker -c 'docker exec -d foxy-live bash /home/mu/foxy_ws/e2e_replay_run.sh'
  完成标记 /tmp/e2e.done，取结果: sg docker -c 'docker exec foxy-live cat /tmp/e2e.test.log'
  快速回归（无 ROS 无 bag）：`python3 test_veto_regression.py`（53 断言，秒级）。
  健康看 `/veto/status`（state/age_ms/blk/calib/计数/stamprej）。

## 注意

- 大文件不入库：bag、地图 PCD、构建产物（见 `.gitignore`）
- 视频链路 QoS 全 best_effort（reliable 大消息跨 WiFi UDP 会静默卡死订阅方）
- 主板侧话题（/lf/sportmodestate 等）别信 CLI：`ros2 daemon` 是 FastDDS 参与者，看不见主板
  （`topic info -v` 显示 publisher count 0 但数据在流）；`topic list` 里"看得见"也可能是自己
  订阅的自echo——主板侧一律用 python 订阅探活
