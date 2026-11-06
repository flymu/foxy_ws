# Vendored from slam-autonav-console (MIT)

上游仓库：`/home/mu/workspace/slam-autonav-console`（`robot_console/` 目录，MIT 许可证，
全文见本目录 [LICENSE.upstream](LICENSE.upstream)）。
用途：浏览器点云可视化/遥控/监控，接入巡检平台。改动只在"真机化"所需范围内，
保持与上游逐行对齐，方便日后同步上游修复。

## 后端 `patrol_console/`（包名从 robot_console 改为 patrol_console）

逐行未动（上游验证过的模块）：

- `mapping.py` — 体素增量流（0.1m 体素、int64 打包键、二进制帧编码、游标 pull）
- `teleop.py` — 10Hz 遥控、限幅、0.5s 看门狗、锁定
- `process_manager.py` — stacks.yaml 白名单、进程组信号（SIGINT→TERM→KILL）、
  残留进程清理（.running_<port>.json）
- `ros_node.py` — raw 订阅话题监控、TF 边、急停零速、Trigger 服务调用
- `runs.py` — 运行记录（est.tum / gt.tum / run.json）
- `config.py` — 两层配置、`${data_root}` 展开、web_dist 自动定位

改动（都有注释标注）：

- `server.py` — 裁掉仿真场景/地图库/导航/航点/评测/批处理路由；`/api/mapping/presets`
  恒返 `[]`；mapping 状态 `save` 恒 idle、`waypoints` 恒 null；**Foxy 修复：`asyncio.to_thread`
  → `run_in_executor`（py3.8 无 to_thread）**
- `main.py` — 裁掉 MapLibrary/Navigation/GLIM 构造；**遥控互斥门改为"别的节点在发
  /cmd_vel_intent 就拒绝"（上游是航点运行时拒绝）**；**Foxy 修复：SignalHandlerOptions
  try/except 回退**；`_use_venv` 兼容 pip --target 平铺目录（容器无 ensurepip）

## 前端 `web/`（Vue3 + Three.js + Element Plus）

- `src/router.ts` — 只留建图/监控两页（导航/地图库/评测页对应后端已裁掉）
- `src/views/MappingView.vue` — 真机化：仿真卡 `v-if="sim"`（stacks.yaml 无 sim 进程自动隐藏）、
  航点/保存卡 `WAYPOINTS_ENABLED/SAVE_ENABLED` 开关（默认 false）、遥控与步骤高亮改按
  "里程计活着"（`dataLive`）而非"仿真在跑"、步骤重新编号 1/2
- `src/locales/zh-CN.ts` / `en.ts` — 新增 `mapping.teleop.plannerBusy`、`mapping.teleop.needLio`、
  `mapping.steps.needLio` 三个键
- 其余（Three.js 点云渲染 `PointCloudView.vue`、`mapping/stream.ts` 客户端缓冲、
  `stores/`、`api.ts`、导航页等未引用文件）未动；`web/dist` 为本机 `npm run build` 产物
  （vite 8 / vue 3.5 / three 0.186，见 web/package.json）
