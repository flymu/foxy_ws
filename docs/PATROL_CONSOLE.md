# 巡检平台网页控制台（patrol_console）

浏览器里的点云可视化 + 遥控 + 急停 + 系统监控，替代 RViz。代码从 MIT 上游
`slam-autonav-console`（本科毕业设计，`/home/mu/workspace/slam-autonav-console`）裁剪合入，
裁剪明细与逐文件差异见 [patrol_console/VENDORED.md](../patrol_console/VENDORED.md)。

## 为什么需要它

RViz 在这套系统上有三个实测过的病（见项目记忆 11-04/11-05）：

1. **WiFi 单播棘轮**：每个 RViz 订阅者在 WiFi 上多一路独立点云流（32Mbps 级），
   订阅者一多链路直接饱和，雷达戳龄积压、规划崩坏。
2. **rclpy ctypes 读天花板**：Python 侧读大点云只有 ~5.3Hz。
3. **TF 旧戳坑**：RViz 对 TF/时间戳的宽容放大了调试误判。

patrol_console 的解法：后端在**容器里**订阅一次 `/cloud_registered`（与 Point-LIO 同机，
本地 DDS，**零新增 WiFi 流量**），按 0.1 m 体素去重后只把"新出现的体素"发给浏览器；
浏览器 Three.js 增量上 GPU。整条链路对 WiFi 的占用只有一份轻量 WebSocket。

```
点-LIO(容器) --本地DDS--> patrol_console后端(容器) --WebSocket/HTTP(host网络)--> 笔记本浏览器
                                │
                                └─遥控 Twist ──> /cmd_vel_intent（10Hz，0.5s看门狗）
```

## 启动与使用

```bash
# 一次性依赖（fastapi/uvicorn 装进容器可见的 pydeps/，已经装好）
bash /home/mu/foxy_ws/patrol_console/setup_deps.sh

# 启动（容器 foxy-live 里跑；已在跑会直接提示）
bash /home/mu/foxy_ws/patrol_console/start_console.sh

# 浏览器打开（WS 的 IP）
http://192.168.101.25:8000        # 首页=建图页；#/monitor 是系统监控页

# 无狗演示（发合成 /cloud_registered + /Odometry，浏览器里能看到点云流动）
docker exec -d foxy-live bash -c "setsid bash -c 'source /opt/ros/foxy/setup.bash && \
  python3 /home/mu/foxy_ws/patrol_console/tests/synth_publisher.py' \
  >> /home/mu/foxy_ws/patrol_console/data/synth.log 2>&1 < /dev/null"

# 停止控制台 / 停止合成数据
docker exec foxy-live bash -c "pkill -f 'patrol_console[.]main'"      # SIGINT 优雅退出
docker exec foxy-live bash -c "pkill -f 'synth_publisher[.]py'"

# 冒烟回归（26 断言：REST/WS/二进制帧/遥控闭环/急停/互斥门）
docker exec foxy-live bash -c 'source /opt/ros/foxy/setup.bash && \
  export PYTHONPATH=/home/mu/foxy_ws/patrol_console/pydeps:$PYTHONPATH && \
  python3 /home/mu/foxy_ws/patrol_console/tests/ws_probe.py'
```

前端改动后重新构建（宿主机有 node 22）：`cd patrol_console/web && npm ci && npm run build`
（产物在 `web/dist`，后端启动时自动找它；仓库里带着已构建的 dist，容器里不需要 node）。

日志：`patrol_console/data/console.log`；每条建图的轨迹记录在 `patrol_console/data/runs/`
（est.tum / run.json，与上游格式一致）。

## 页面与真机流程的对应

| 页面元素 | 真机栈语义 |
|---|---|
| 建图页 3D 视图 | `/cloud_registered` 体素地图（frame `camera_init`）+ 估计轨迹 + 当前帧 |
| 「① SLAM」卡片 | 启动**纯 Point-LIO**（stacks.yaml 白名单唯一进程）。停止时 LIO 自动把地图落盘 `/tmp/lio_<时间>_global_map.pcd`（`map_save_path:=/tmp`），**停止建图即存图** |
| 「② 移动」卡片 | 遥控（摇杆/WASD，10Hz），见下方安全设计 |
| 系统监控页 | 话题频率/发布者数/节点列表/TF/CPU 内存，话题清单在 console.yaml `topics` |
| 右上角急停 | 见"急停的边界" |

**规约**：纯建图（采新地图）→ 用控制台「启动」；巡检/导航全套栈（LIO+定位+规划+speed_adapter
+看门狗等安全件）→ 继续用 `live_stack_start.sh`，控制台只当"看+遥控"用。
两边互相不知道对方，**同时起两个 LIO 会让地图来回跳**——真发生了页面会弹
"话题有多个发布者"警告，按提示处理。

## 安全设计（遥控与急停）

链路：**网页 10Hz → 后端限幅 → `/cmd_vel_intent` → speed_adapter（起步地板 0.25 m/s）
→ `/cmd_vel_raw` → veto（Orin）→ `/go2/cmd_vel` → 狗**。遥控不绕过任何一层防线。

- **限幅**：`teleop.max_v=0.4 / max_w=1.0`；推荐速度带 `[0.25, 0.4]`（狗实测 0.20~0.25
  以下原地假死，见 11-04），滑条上画出来，超速会红字提示。
- **0.5 s 看门狗**：网页断开、切页、卡死，后端立刻补发零速度（`teleop.timeout`）。
- **断开即停**：每个连接断开时若正在遥控，立即停车。
- **规划器互斥**：`/cmd_vel_intent` 上有**别的节点**在发速度（自主导航中）时遥控直接拒绝
  （`main.py` 的 `planner_active`，按发布者节点名过滤——控制台自己在这个话题上有两个发布者
  不能算）。两个来源交替发速度会让狗来回抢指令，这是硬互斥。
- **急停的边界（重要）**：急停做三件事——2 s 内持续向 `/cmd_vel_intent` 压零速度、锁定遥控
  （需网页点"解除"）、停掉 `stop_on_estop: true` 的进程。它是**软急停**：如果规划器还在发
  目标速度，压完之后会恢复运动；真机的硬防线仍是 **veto + 看门狗 + 断电**。自主导航中要
  彻底停车应同时用一键脚本停止/暂停规划栈，或直接断电。

## Foxy 兼容清单（本次实测踩坑记录）

上游按 Humble/py3.10 写，容器是 Foxy/py3.8。已验证可用：`raw=True` 订阅（监控话题零解析开销）、
`get_publishers_info_by_topic`（含 `node_name`，互斥门就靠它）、`count_publishers`、QoSProfile
全参数、`math.dist`（3.8+）、容器内 psutil/yaml/numpy。踩到的坑（都修在 `server.py`/`main.py`）：

| 坑 | 症状 | 修法 |
|---|---|---|
| `rclpy.signals.SignalHandlerOptions` 是 Humble+ | 进程起不来 ImportError | try/except 回退普通 `rclpy.init()`，uvicorn 的 Python 级处理器覆盖 rcl 的 C 级 |
| `asyncio.to_thread` 是 py3.9+ | WebSocket 一条消息都收不到（send_loop 静默死） | `loop.run_in_executor(None, ...)` |
| 容器无 ensurepip 且 UID 1000 非 root 建 venv | `python3 -m venv` 失败 | `pip3 install --target pydeps` 平铺目录，`_use_venv` 兼容两种布局 |

fastapi 0.124.4 + uvicorn 0.33.0 在 py3.8 下工作正常（pydeps 里 pip 自动选了兼容版本）。

## 上游裁剪与已知边界

裁掉（真机环境没有对应物）：仿真场景/场景导入、地图库与网页保存、导航页、自动航点、评测、
批处理。前端保留对应代码路径但隐藏（`MappingView.vue` 的 `sim`/`WAYPOINTS_ENABLED`/
`SAVE_ENABLED` 开关），后续要接只改开关+补后端。

已知边界 / 后续可做：

1. **网页保存地图卡隐藏**：Point-LIO 是退出落盘，停止建图即存图（见上表）。要网页保存
   （存到地图库目录、生成 2D 栅格）需要把上游 `map_library` 移植过来。
2. **急停不接 `clearTarget`**：把急停同时接到规划器的 clearTarget 服务，能让软急停变硬
   停（规划栈被真正清目标）。涉及改我们的规划器/安全栈，属于安全链变更，谨慎做。
3. **监控话题刻意不含 `/livox/points2`**：那是 Orin 发来的 32Mbps 级大流，从 WS 再订一路
   等于往 WiFi 加一路单播（11-04 教训）。跨 WiFi 的小话题（veto/status、go2/cmd_vel 等）保留。
4. **点云处理 5 Hz 上限**（`process_rate`）：体素去重在 WS 上跑，6 核机器留余量；地图大
   （>10⁶ 体素）后浏览器增量收点会有可感的几秒，属正常。
