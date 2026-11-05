#!/usr/bin/env python3
"""r5_verify.py — R5 回放回归断言器（宿主机跑，读 logs_r5/ 共享卷）
用法: python3 /home/mu/foxy_ws/r5_verify.py
退出码 0=全绿 1=有失败
"""
import math
import os
import re
import sys

R = "/home/mu/foxy_ws/logs_r5"
FAILS = []


def check(name, ok, detail=""):
    tag = "PASS" if ok else "FAIL"
    print(f"[{tag}] {name}{(' — ' + detail) if detail else ''}")
    if not ok:
        FAILS.append(name)


def load(capfile):
    """返回 [(wall_ms, tag, fields)]"""
    rows = []
    path = os.path.join(R, capfile)
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 3:
                continue
            try:
                rows.append((int(p[0]), p[1], p[2:]))
            except ValueError:
                continue
    return rows


def t0dict(name):
    d = {}
    path = os.path.join(R, name)
    if not os.path.exists(path):
        return d
    for line in open(path):
        p = line.split()
        if len(p) >= 2 and p[1].isdigit():
            d[p[0]] = int(p[1])
    return d


def grep_log(name, pat):
    path = os.path.join(R, name)
    if not os.path.exists(path):
        return []
    return re.findall(pat, open(path, errors="ignore").read())


def grep_log_ts(name, pat):
    """返回匹配行的日志时间戳(ms 列表)——门控时序断言用"""
    path = os.path.join(R, name)
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, errors="ignore"):
        if re.search(pat, line):
            m = re.search(r"\[(\d{10})\.", line)
            if m:
                out.append(int(m.group(1)) * 1000)
    return out


# ================= Run 1 =================
cap1 = load("rg1_cap.txt")
t1 = t0dict("rg1_t0")
loc1 = [(w, f) for w, tag, f in cap1 if tag == "loc"]
cmd1 = [(w, f) for w, tag, f in cap1 if tag == "cmd"]
health1 = [(w, f[0]) for w, tag, f in cap1 if tag == "health"]
lio1 = [(w, f) for w, tag, f in cap1 if tag == "lio"]

check("R1-采集非空(≥2000行)", len(cap1) >= 2000, f"{len(cap1)} 行")
check("R1-健康格式三段式",
      all(re.match(r"^[A-Z]+\|[^|]+\|[0-9.]+$", h) for _, h in health1) and len(health1) > 50,
      f"{len(health1)} 条健康消息")

# 初始化完成后应出现 OK
ok_seen = any(h.startswith("OK|") for _, h in health1)
check("R1-健康出现OK", ok_seen)

# goal1 → 注入前有非零 cmd（若 OK 在 goal1 前已出现）
if ok_seen and "goal1" in t1 and "inj_start" in t1:
    moving = any(abs(float(f[0])) > 0.05 or abs(float(f[2])) > 0.05
                 for w, f in cmd1 if t1["goal1"] + 2000 <= w <= t1["inj_start"] - 500)
    check("R1-goal后规划器有输出(非零cmd)", moving)

# 注入窗内全零（跳过 ros2 CLI 起振的 ~1.5s——DEGRADED 首条到达前 cmd 合法非零）
if "inj_start" in t1 and "inj_end" in t1:
    win = [(w, f) for w, f in cmd1 if t1["inj_start"] + 2000 <= w <= t1["inj_end"] - 500]
    zeros = all(abs(float(f[0])) < 1e-6 and abs(float(f[1])) < 1e-6 and abs(float(f[2])) < 1e-6
                for w, f in win)
    check("R1-DEGRADED注入→cmd全零", len(win) > 50 and zeros, f"{len(win)}拍")
    # 注入后到新 goal 前：保持零（不追旧目标）
    hold = [(w, f) for w, f in cmd1 if t1["inj_end"] + 4000 <= w <= t1["goal2"] - 1000]
    holdz = all(abs(float(f[0])) < 1e-6 and abs(float(f[1])) < 1e-6 and abs(float(f[2])) < 1e-6
                for w, f in hold)
    check("R1-注入停后不追旧目标(cmd保持零)", len(hold) > 20 and holdz, f"{len(hold)}拍")

# 新 goal 后重新运动
if "goal2" in t1:
    rem = any(abs(float(f[0])) > 0.05 for w, f in cmd1 if t1["goal2"] <= w <= t1["goal2"] + 15000)
    check("R1-新goal后规划器恢复运动", rem)

# 门控日志恰三关两开：启动无流关 / 注入关 / 袋尾数据停关（各恰一次重开除袋尾）。
# 注入窗内绝不重开：OK@1Hz 与 DEGRADED@10Hz 叠加时 OK 持续 <1s，
# 重开须 OK 保持 ≥reopen_hold——这正是 11-06 修复的开关振荡（曾 11 关 10 开）。
close_n = len(grep_log("rg1_planner.log", r"\[gate\] 定位门控关闭"))
open_n = len(grep_log("rg1_planner.log", r"\[gate\] 定位门控重开"))
check("R1-门控恰三关两开(启动/注入/袋尾)", close_n == 3 and open_n == 2,
      f"关{close_n} 开{open_n}")
clearn = len(grep_log("rg1_planner.log", r"\[clearTarget\] target dropped"))
check("R1-核心clearTarget生效日志(每次关门恰一次)", clearn == close_n and clearn >= 1,
      f"{clearn} 次")
if "inj_start" in t1 and "inj_end" in t1:
    opens_in_inj = [t for t in grep_log_ts("rg1_planner.log", r"门控重开")
                    if t1["inj_start"] - 2000 <= t <= t1["inj_end"] + 800]
    check("R1-注入期间门控保持关闭(无重开振荡)", len(opens_in_inj) == 0,
          f"{len(opens_in_inj)} 次重开")
    reopens = [t for t in grep_log_ts("rg1_planner.log", r"门控重开")
               if t1["inj_end"] <= t <= t1["inj_end"] + 4000]
    check("R1-注入结束后门控及时重开(OK保持1s)", len(reopens) >= 1)

# /initialpose 重种子——健康跟踪下错误种子被 NDT 当帧精修拉回（设计行为，
# 位姿不跳是好事）；断言改为"值被受理 + 跟踪存活"。种子冷启动效果另由
# Run 3b(T_mo 注入可见) 与 BBS Phase 0b 覆盖。
reseed_n = len(grep_log("rg1_loc.log", r"\[reseed\] T_mo"))
check("R1-/initialpose重种子恰一次", reseed_n == 1, f"{reseed_n} 次")
seed_line = ""
for line in open(os.path.join(R, "rg1_t0"), errors="ignore"):
    if "seed T_mb=" in line:
        seed_line = line
        break
m_sent = re.search(r"T_mb=\((-?\d+\.\d+),(-?\d+\.\d+)\) yaw=(-?\d+\.\d+)", seed_line)
m_got = None
path = os.path.join(R, "rg1_loc.log")
for line in open(path, errors="ignore"):
    if "[reseed]" in line:
        m_got = re.search(r"T_mb=\((-?\d+\.\d+), (-?\d+\.\d+), yaw (-?\d+\.\d+)°", line)
        break
if m_sent and m_got:
    dmax = max(abs(float(a) - float(b)) for a, b in
               zip(m_sent.groups(), m_got.groups()))
    check("R1-种子值端到端一致(工具发=节点收)", dmax < 0.05, f"最大偏差 {dmax:.3f}")
else:
    check("R1-种子值端到端一致(工具发=节点收)", False, "未解析到种子行")
if "seed" in t1 and health1:
    post_ok = any(h.startswith("OK|") for w, h in health1 if w > t1["seed"] + 3000)
    check("R1-重种子后跟踪存活(健康回OK)", post_ok)
    no_lost = all(not h.startswith("LOST|")
                  for w, h in health1 if t1["seed"] <= w <= t1["seed"] + 8000)
    check("R1-重种子不引发LOST", no_lost)
    alive = sum(1 for w, tag, f in cap1 if tag == "loc" and t1["seed"] <= w <= t1["seed"] + 10000)
    check("R1-重种子后loc流连续(≥30帧)", alive >= 30, f"{alive}帧")

# 袋尾心跳：数据停止后健康 1Hz 心跳与数据流解耦，继续播报。
# mirror 场景无死推，设计上终态 = STALE|odom_gap（LOST 仅"死推中+狗流也断"）。
if lio1 and "bag_end" in t1:
    last_lio = max(w for w, _ in lio1)
    after = [(w, h) for w, h in health1 if w > last_lio + 3000]
    states = {h.split("|")[0] for _, h in after}
    check("R1-数据停后健康心跳持续(≥6条)", len(after) >= 6, f"{len(after)}条")
    check("R1-心跳终态稳定(单一非OK态)", len(states) == 1 and "OK" not in states,
          "/".join(sorted(states)) or "无")
    check("R1-终态STALE|odom_gap(mirror设计行为)",
          after[-1][1].startswith("STALE|odom_gap|") if after else False,
          after[-1][1] if after else "无")

# 落盘文件
pers = os.path.join(R, "rg1_state.txt")
txt = open(pers).read() if os.path.exists(pers) else ""
check("R1-持久化落盘(t_mo+dog)", "t_mo" in txt and re.search(r"^dog ", txt, re.M) is not None)

# ================= Run 2b: latedrift（狗走远后锁死推 → 冻结回跳决定性验证） =================
cap2b = load("rg2b_cap.txt")
loc2b = [(w, f) for w, tag, f in cap2b if tag == "loc"]
health2b = [(w, f[0]) for w, tag, f in cap2b if tag == "health"]
lio2b = [(w, f) for w, tag, f in cap2b if tag == "lio"]
dog2b = [(w, f) for w, tag, f in cap2b if tag == "dog"]

deg2b = [(w, h) for w, h in health2b if h.startswith("DEGRADED|dead_reckoning")]
check("R2b-行走段锁死推(DEGRADED)", len(deg2b) > 0, f"{len(deg2b)}条")
if deg2b and loc2b:
    t_latch = deg2b[0][0]
    near = [f for w, f in loc2b if t_latch - 1500 <= w <= t_latch + 1500]
    jumps = [math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
             for a, b in zip(near, near[1:])]
    mx = max(jumps) if jumps else -1
    check("R2b-锁死推无回跳(<0.5m, 走远场景)", mx < 0.5, f"max={mx:.3f}m")
    # 判别力: 锁时狗离起点远（旧 bug 回跳幅度≈此距离）
    first = [f for w, f in loc2b if w < t_latch - 5000][:3]
    at = [f for w, f in loc2b if abs(w - t_latch) < 800][:1]
    if first and at:
        d_anchor = math.hypot(float(at[0][0]) - float(first[0][0]),
                              float(at[0][1]) - float(first[0][1]))
        check("R2b-场景有效(锁时距锚点>1.0m, 否则回跳无判别力)", d_anchor > 1.0,
              f"距锚点={d_anchor:.2f}m")
    if lio2b and dog2b:
        lmap = {w: (float(f[0]), float(f[1])) for w, f in lio2b}
        dmap = {w: (float(f[0]), float(f[1])) for w, f in dog2b}
        div = []
        for w, (dx, dy) in dmap.items():
            cand = [lw for lw in lmap if abs(lw - w) < 150]
            if cand:
                lx, ly = lmap[min(cand, key=lambda k: abs(k - w))]
                div.append((w, math.hypot(dx - lx, dy - ly)))
        if div:
            dmax = max(d for _, d in div)
            t_freeze = next(w for w, d in div if d >= dmax - 0.05)
            oks = [w for w, h in health2b if h.startswith("OK|") and w > t_latch]
            if oks:
                span = (oks[0] - t_freeze) / 1000.0
                check("R2b-解除迟滞≥2.9s", span >= 2.9, f"span={span:.1f}s")
                near2 = [f for w, f in loc2b if oks[0] - 1500 <= w <= oks[0] + 1500]
                jumps2 = [math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
                          for a, b in zip(near2, near2[1:])]
                mx2 = max(jumps2) if jumps2 else -1
                check("R2b-解除无跳变(<0.5m)", mx2 < 0.5, f"max={mx2:.3f}m")

# ================= Run 2: drift =================
cap2 = load("rg2_cap.txt")
loc2 = [(w, f) for w, tag, f in cap2 if tag == "loc"]
health2 = [(w, f[0]) for w, tag, f in cap2 if tag == "health"]
lio2 = [(w, f) for w, tag, f in cap2 if tag == "lio"]
dog2 = [(w, f) for w, tag, f in cap2 if tag == "dog"]

deg = [(w, h) for w, h in health2 if h.startswith("DEGRADED|dead_reckoning")]
check("R2-锁死推(DEGRADED|dead_reckoning)", len(deg) > 0, f"{len(deg)}条")
if deg:
    t_latch = deg[0][0]
    # 冻结回跳: 锁前后 1.5s 内 loc 最大相邻跳 < 0.5m
    near = [f for w, f in loc2 if t_latch - 1500 <= w <= t_latch + 1500]
    jumps = [math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
             for a, b in zip(near, near[1:])]
    mx = max(jumps) if jumps else -1
    check("R2-锁死推无回跳(最大相邻跳<0.5m)", mx < 0.5, f"max={mx:.3f}m")
    # 分歧曲线 → 冻结点
    if lio2 and dog2:
        lmap = {w: (float(f[0]), float(f[1])) for w, f in lio2}
        dmap = {w: (float(f[0]), float(f[1])) for w, f in dog2}
        div = []
        for w, (dx, dy) in dmap.items():
            cand = [lw for lw in lmap if abs(lw - w) < 150]
            if cand:
                lx, ly = lmap[min(cand, key=lambda k: abs(k - w))]
                div.append((w, math.hypot(dx - lx, dy - ly)))
        if div:
            dmax = max(d for _, d in div)
            t_freeze = next(w for w, d in div if d >= dmax - 0.05)
            oks = [w for w, h in health2 if h.startswith("OK|") and w > t_latch]
            t_unlatch = oks[0] if oks else None
            span = (t_unlatch - t_freeze) / 1000.0 if t_unlatch else -1
            check("R2-解除迟滞≥2.9s(冻结→OK)", span >= 2.9, f"span={span:.1f}s")
            if t_unlatch:
                near2 = [f for w, f in loc2 if t_unlatch - 1500 <= w <= t_unlatch + 1500]
                jumps2 = [math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
                          for a, b in zip(near2, near2[1:])]
                mx2 = max(jumps2) if jumps2 else -1
                check("R2-解除无跳变(<0.5m)", mx2 < 0.5, f"max={mx2:.3f}m")
            cand = [(w, d) for w, d in div if abs(w - t_latch) <= 300]
            latch_div = min(cand, key=lambda p: abs(p[0] - t_latch))[1] if cand else -1
            # 看门狗锁的是窗口位移分歧(≥0.5m)；绝对差在锁时刻≈0.5×窗长
            check("R2-锁由真实分歧触发(锁时刻绝对差≥0.6m)", latch_div >= 0.6,
                  f"div@latch={latch_div:.2f}m")

# ================= Run 3a: 拒绝续接 =================
rej = grep_log("rg3a_loc.log", r"\[resume\] 狗已移动")
cand = grep_log("rg3a_loc.log", r"\[resume\] 落盘.*有续接候选")
check("R3a-识别候选+拒绝续接(维持known-start)",
      len(cand) >= 1 and len(rej) >= 1, f"候选{len(cand)} 拒绝{len(rej)}")
cap3a = load("rg3a_cap.txt")
loc3a = [(w, f) for w, tag, f in cap3a if tag == "loc"]
check("R3a-known-start照常定位(loc流≥50)", len(loc3a) >= 50, f"{len(loc3a)}条")

# ================= Run 3b: 接受续接 =================
acc = grep_log("rg3b_loc.log", r"\[resume\] 狗位姿与上次会话差 [0-9.]+m")
check("R3b-续接成功日志", len(acc) >= 1, f"{len(acc)} 次")
cap3b = load("rg3b_cap.txt")
loc3b = [(w, f) for w, tag, f in cap3b if tag == "loc"]
xs3b = [float(f[0]) for _, f in loc3b[:600]]  # 前段
mx3b = max(xs3b) if xs3b else -99
check("R3b-T_mo注入可见(x达1.5~3.4)", 1.5 <= mx3b <= 3.4, f"max_x={mx3b:.2f}")

print(f"\n===== R5 回放回归: {'全绿' if not FAILS else str(len(FAILS)) + ' 项失败'} =====")
if FAILS:
    for f in FAILS:
        print("  FAIL:", f)
sys.exit(1 if FAILS else 0)
