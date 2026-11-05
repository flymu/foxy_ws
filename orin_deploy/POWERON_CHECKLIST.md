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

- [ ] **1.1 雷达**：`ssh go2_wifi 'bash -s' < start_livox.sh`
      验证：`ssh go2_wifi 'timeout 8 ros2 topic hz /livox/points2 --window 20'` ≈10Hz
- [ ] **1.2 看门狗**：`ssh go2_wifi 'setsid nohup bash /home/unitree/livox_watchdog.sh >/dev/null 2>&1 < /dev/null &'`
      （只防长跑塌方；人工停雷达它会自动退出告警模式，不拉起）
- [ ] **1.3 关自带避障**（每次上电必查！固件复位成 ON，开着会朝障碍不动/截停污染测量）：
      `ssh go2_wifi 'python3 /home/unitree/go2_avoid_switch.py get'` → 必须 off
      是 on 就：`... go2_avoid_switch.py off`（巡检期间保持关，veto 是我们的安全层）
- [ ] **1.4 安全栈**：`ssh go2_wifi 'bash /home/unitree/patrol_safety_start.sh [viz]'`（relay+veto）
      验证：`timeout -s INT 4 ros2 topic echo /veto/status --once`
- [ ] **1.5 控制桥**（测试时起，测完杀；开着会锁遥控杆量）：
      `ssh go2_wifi 'bash /home/unitree/go2_bridge_start.sh'`
      验证：`timeout 6 ros2 topic echo /go2/status --once`
- [ ] **1.6 四足里程计桥**：`ssh go2_wifi 'bash /home/unitree/go2_odom_start.sh'`（两段接力）
      验证：`timeout 6 ros2 topic hz /go2/odom`（狗唤醒后才有数据）

## 2. 狗唤醒、牵到建图起点 → 重定位（known-start 或续接）

- [ ] **2.1** 起 WS 巡检栈：`sg docker -c 'docker exec foxy-live bash /home/mu/foxy_ws/live_stack_start.sh'`
- [ ] **2.2 重定位**（point_lio 位姿已脏，禁直接导航）：
      - 自动续接（P0-1）：狗没挪窝（<0.5m）时 loc 日志出现 `[resume] ... 续接 T_mo`
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

- [ ] 停 WS 栈：`bash live_stack_stop.sh`（容器内）
- [ ] Orin 停安全栈：`ssh go2_wifi 'bash /home/unitree/patrol_safety_stop.sh'`
- [ ] 看门狗：`ssh go2_wifi 'pkill -f "[l]ivox_watchdog"'`
- [ ] 桥确认已死（1.5 的铁律）
- [ ] 位置留档：`/home/mu/maps/loc_state.txt`（loc 每 5s 自动落盘，下次上电续接用）
