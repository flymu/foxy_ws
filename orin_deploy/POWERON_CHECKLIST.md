# 上电前检查单（POWERON_CHECKLIST）— 2026-11-05 R5 批之后

> 上电前所有离线工作应已完成（见 README 与 docs/REVIEW_2026-11-05.md）。
> 本单是"狗上电 → 可信巡逻"的现场顺序。每步有验证命令，**验证不过不进下一步**。

## 0. 上电前（WS 侧，可提前做）

- [ ] 代码对齐：`git log --oneline -3` 确认包含 R5/门控批；工作树干净
- [ ] 回放回归全绿（上电前夜跑过）：
      `sg docker -c 'docker exec -d foxy-live bash /home/mu/foxy_ws/r5_replay_regression.sh'`
      → `python3 /home/mu/foxy_ws/r5_verify.py`
- [ ] 推安全栈到 Orin：`bash orin_deploy/push_safety.sh`（对齐后 Orin 侧才有 G1 副本）

## 1. 狗上电，Orin 起来后（顺序即依赖序）

- [ ] **1.0 Orin 对时**（必须在 1.1 之前）。11-07 起换了 WiFi 模块（AIC8800DC→littlered 自动连，
      有外网），**systemd NTP 自同步**：等 WiFi 连上（~30s）后验证
      `ssh go2_lan 'timedatectl | grep -E "synchronized|NTP service"'` → 须 `synchronized: yes`。
      仍要"对好时再起雷达"——雷达活着时跳时钟会打疯墙钟定时器（实测 2074Hz 狂喷），
      垃圾帧流把 point_lio 状态打到 17.8m 游走。WiFi 连不上（断网兜底）才手动：
      `ssh -t go2_lan "sudo date -s @$(date +%s)"`（-t 交互输 sudo 密码，**密码不写进本仓库**）
      然后 `ssh go2_lan 'date -u'` 核对。
- [ ] **1.1 雷达**：`ssh go2_lan 'bash -s' < start_livox.sh`
      验证：`ssh go2_lan 'timeout 8 ros2 topic hz /livox/points2 --window 20'` ≈10Hz
      （须在 1.4 之后验——points2 是 relay 的输出；rclpy 直读天花板 ~2.5-5Hz 属正常）
- [ ] **1.2 关自带避障**（每次上电必查！固件复位成 ON，开着会朝障碍不动/截停污染测量）：
      `ssh go2_lan '...环境见下... python3 /home/unitree/go2_avoid_switch.py get'` → 必须 off
      是 on 就：`... go2_avoid_switch.py off`（巡检期间保持关，veto 是我们的安全层）
      环境（unitree_sdk2py 走源码树 + cyclonedds 库，缺了报 No module named unitree_sdk2py）：
      `source /opt/ros/foxy/setup.bash && source /home/unitree/unitree_ros2/install/setup.bash &&`
      `export PYTHONPATH=/home/unitree/unitree_ros2/unitree_sdk2_python &&`
      `export LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib`
      ⚠️ 11-06 曾报 SEGFAULT，**11-07 用上面完整环境未复现（get/off 各两次全通过）**——
      上面的 segfault 大概率是当时环境不全所致；若再炸带 `python3 -X faulthandler` 抓现场。
      每次上电它仍必查必关（固件复位成 ON 不变）。
- [ ] **1.3 安全栈**：`ssh go2_lan 'bash /home/unitree/patrol_safety_start.sh [viz]'`（relay+veto）
      验证：`ssh go2_lan 'bash -s' <<< 源码见下` 简化命令：
      `timeout -s INT 6 ros2 topic echo /veto/status 2>/dev/null | head -2`
      （⚠️ Foxy 的 echo **没有 --once 参数**——写了会 unrecognized arguments 假报"无流"；
      靠 timeout 截断取首条即可。判定：出 `state=INIT ... blk=none ...` 行。
      state 字段恒为 INIT 是正常的——**真正的否决状态在 blk= 字段**（none/SECTOR/POINT），
      frames/fresh 递增=在消费点云。）
- [ ] **1.4 看门狗**（11-06 改序：必须在 relay 之后！它探 /livox/points2，relay 没起会
      连数两次空探 30s 误判塌方去重启驱动）：
      `ssh go2_lan 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'`
- [ ] **1.5 控制桥**（测试时起，测完杀；开着会锁遥控杆量）：
      `ssh go2_lan 'bash /home/unitree/go2_bridge_start.sh'`
      验证：`timeout 6 ros2 topic echo /go2/status --once`
- [ ] **1.6 四足里程计桥**：`ssh go2_lan 'bash /home/unitree/go2_odom_start.sh'`（两段接力）
      验证：**看桥日志** `ssh go2_lan 'tail -2 /tmp/go2_odom_bridge.log'` 出 `回传中 ... ~20Hz` 即通
      （狗趴着静默属正常）。⚠️ 别用 `ros2 topic hz /go2/odom` 验——**/go2/odom 是
      BEST_EFFORT 而 Foxy 的 hz 默认 RELIABLE 且无 --qos 参数**，QoS 不匹配=永远静默
      无数据（假象"无流"，11-08 实测 20Hz 在流而 hz 零输出）。rclpy 探针必须
      `qos_profile_sensor_data`。

> ssh 远程链里 pkill 别写裸进程名（`pkill -f livox_ros_driver2` 会匹配到含同串的会话
> shell 自身把 ssh 杀断）；一律 `[l]ivox` 括号式。

## 2. 狗唤醒、牵到建图起点 → 重定位（known-start 或续接）

- [ ] **2.1** 起 WS 巡检栈（两个等价入口，跑同一脚本；11-07 起网页可用）：
      网页：控制台建图页「巡检栈」卡片「启动」（先确认卡上不是红色"未检测到 Orin 安全栈"——
      红色=1.3 没起好）；命令行：`sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_start.sh'`
      ⚠️ 命令行起的栈网页不能一键停（启动也会被 409 拒），只能命令行停——上电流程统一用一种入口。
- [ ] **2.2 重定位**（point_lio 位姿已脏，禁直接导航）：
      - **11-07 00:48 起续接候选有效**（狗下电前 loc 全程健康：fitness 0.010→0.018 真盆地，
        看门狗正确扛过下电瞬变）：狗没挪窝时优先等 `[resume] 续接 T_mo`；
      - 续接不过（狗被搬走充电/odom 重置）→ 手动种子省 BBS：
        **趴位 (0.17, -0.13) yaw ≈ -1.3°**（11-07 蠕行后停位，两次独立 NDT 收敛互证）；
        种子略偏没关系，NDT 会拉回（11-07 实测：隔夜旧种子被 NDT 从 0.5m 外拉回真位）。
        落地判据 **fitness ≤ 0.02 且稳定**（假盆地特征是 0.05+，健康门 OK 不算数）。
      - 自动续接（P0-1）：狗没挪窝（<0.5m）时 loc 日志出现 `[resume] ... 续接 T_mo`
        （本次不适用；恢复续接前提是 loc_state 重新健康落盘）
      - 冷启动/挪窝：等 health=OK 后如定位不对，用 RViz "2D Pose Estimate"（/initialpose）
        或 BBS 种子重播种，日志应出现 `[reseed] T_mo ← /initialpose`
      验证：`timeout 8 ros2 topic echo /loc/health` → `OK|lio_ndt|age` 且不抖
- [ ] **2.3 体检**：`sg docker -c 'docker exec foxy-live python3 /home/mu/foxy_ws/doctor.py'` 全绿

## 3. 分级实测（G 门控，逐级放行）

- [ ] **G2 定位浸泡 30min**：狗静置，盯 `/loc/health` 不出 OK、戳龄正常
      （戳龄监控：`bash watch_lag.sh`；驱动 22-48min 发病窗口由看门狗兜底）
- [ ] **G3 最小运动**：creep_test.py 蠕行一圈（≥0.25m/s 起步阈值），复查 health 仍 OK
- [ ] **G4 定位可信**之后才允许规划器接目标巡逻；任何 DEGRADED/LOST 门控会自动
      清目标+零速（B3），恢复后需人工重发目标——这是设计，不是 bug
- [ ] 测试完杀桥：`ssh go2_wifi 'pkill -f "[g]o2_bridge.py"'` 再验证 `pgrep` 为空

## 4. 断电前

- [ ] 停 WS 栈：网页「巡检栈」卡片「停止」，或 `bash live_stack_stop.sh`（容器内）
- [ ] Orin 停安全栈：`ssh go2_wifi 'bash /home/unitree/patrol_safety_stop.sh'`
- [ ] 看门狗：`ssh go2_wifi 'pkill -f "[l]ivox_watchdog"'`
- [ ] 桥确认已死（1.5 的铁律）
- [ ] 位置留档：`/home/mu/maps/loc_state.txt`（loc 每 5s 自动落盘，下次上电续接用）
- [ ] **断电后（WS 侧）核对 loc_state**（11-07 教训）：断电后 loc 节点会在 WS 上继续
      每 5s 落盘（数据停不阻断 save_persist）——落盘的是"断电前最后状态"。
      **断电前 loc 必须处于 fitness 正常的真盆地**；若断电前刚发生看门狗分歧/fitness
      恶化（>0.05），落盘 t_mo 即为毒，须按 `maps/LOC_STATE_中性化说明_1107.md` 撤文件
      （停 loc 节点 → 改名出加载路径），否则下次 [resume] 直接吃进假盆地。
