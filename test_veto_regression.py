#!/usr/bin/env python3
"""test_veto_regression.py — G1 无 ROS 回归套件（2026-11-05，狗不上电可跑）

覆盖（对应 docs/REVIEW_2026-11-05.md 的 R1–R4 与 DEVELOPMENT_PLAN G1 退出条件）：
  R1 距离度量：多倾角×多方位×已知物理距离；评审复现用例（真前向 1.20m、
     离地 0.40m——旧公式算 0.8162559）；频带/自体掩膜/near_keep/方位符号
  R2 只抑制不放大：空场放行、未消解指令原样、消解残速低于地板→归零
     （旧版抬升回 0.25 的越权行为必须死）、随机性质测试 |out|≤|in|
  R3 采集时效：新鲜前进放行、超龄拒、冻结/回退拒、闸门关闭放行
  R4 指令仲裁：stop 不被速度顶掉（评审纯复现用例）、halt 期间速度丢弃、
     resume/balance 解锁、看门狗瞬态不锁存、hi 通道 FIFO

运行: python3 test_veto_regression.py   （依赖仅 numpy；失败非零退出）
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bridge_core  # noqa: E402
import veto_core  # noqa: E402

FAILS = []
N_CHECKS = [0]


def check(name, cond, detail=""):
    N_CHECKS[0] += 1
    tag = "PASS" if cond else "FAIL"
    print("  [%s] %s%s" % (tag, name, (" — " + detail) if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------- R1：距离度量 ----------------
def test_r1_geometry():
    print("R1 距离度量（倾斜系投影压缩 bug）")
    prm = veto_core.Params()
    g = veto_core.Geom(veto_core.PRIOR_N, veto_core.PRIOR_H, body_along=0.05)

    # 评审复现：狗身轴心前方 1.20m、离地 0.40m 的单点（轴心在传感器投影前 0.05m）
    # → 旧公式 0.8162559（压缩 ×0.6802），正确值 1.20
    P = g.to_sensor(np.array([[1.25, 0.0, 0.40]]))
    sl = veto_core.ingest(P, g, prm)
    check("评审复现: 真前向1.20m→dh=1.20", len(sl) == 1 and approx(sl[0, 0], 1.20),
          "got %s" % (sl.tolist() if len(sl) else "空"))
    check("评审复现: az=0(正前)", len(sl) == 1 and approx(sl[0, 1], 0.0))

    # 各向同性：同一水平距离 1.0m（距轴心）、不同方位 → dh 必须同为 1.0。
    # 旧公式前向 0.68、侧向 1.0（椭圆），这是最直接的回归哨兵
    for az_deg in (0, 30, 59, 61, 90, 150, 180, 270):
        x = 0.05 + 1.0 * math.cos(math.radians(az_deg))
        y = 1.0 * math.sin(math.radians(az_deg))
        P = g.to_sensor(np.array([[x, y, 0.50]]))
        sl = veto_core.ingest(P, g, prm)
        check("方位 %3d°: dh=1.00" % az_deg,
              len(sl) == 1 and approx(sl[0, 0], 1.0, 1e-5),
              "got %s" % (sl.tolist() if len(sl) else "空"))

    # 多倾角：0°平装 / 25° / 47°先验 / 30°侧倾（绕x轴）——真距离与倾角无关
    geoms = [
        ("平装0°", np.array([0.0, 0.0, 1.0])),
        ("前倾25°", np.array([-math.sin(math.radians(25)), 0.0,
                              math.cos(math.radians(25))])),
        ("前倾47°(先验)", veto_core.PRIOR_N),
        ("侧倾30°", np.array([0.0, -math.sin(math.radians(30)),
                              math.cos(math.radians(30))])),
    ]
    for name, n in geoms:
        gg = veto_core.Geom(n, 0.486, body_along=0.05)
        P = gg.to_sensor(np.array([[1.25, 0.0, 0.40]]))
        sl = veto_core.ingest(P, gg, prm)
        check("倾角%s: 前向1.20m→dh=1.20" % name,
              len(sl) == 1 and approx(sl[0, 0], 1.20, 1e-5),
              "got %s" % (sl.tolist() if len(sl) else "空"))

    # 方位符号：+左。狗系 (1.05, +0.5, 0.5) → dh=1.12<near_keep，az>0
    P = g.to_sensor(np.array([[1.05, 0.5, 0.50]]))
    sl = veto_core.ingest(P, g, prm)
    check("az 符号: 左正", len(sl) == 1 and sl[0, 1] > 0,
          "got %s" % (sl.tolist() if len(sl) else "空"))

    # 频带外剔除：h=0.10（<band_lo）/ h=0.80（>band_hi）
    P = g.to_sensor(np.array([[1.25, 0.0, 0.10], [1.25, 0.0, 0.80]]))
    sl = veto_core.ingest(P, g, prm)
    check("频带外(0.10m/0.80m高)剔除", len(sl) == 0, "got %d 点" % len(sl))

    # 自体掩膜：传感器正下方（狗系 0,0,0.45 → 传感器系 z≈-0.036 落掩膜盒）
    P = g.to_sensor(np.array([[0.0, 0.0, 0.45]]))
    sl = veto_core.ingest(P, g, prm)
    check("自体掩膜剔除", len(sl) == 0, "got %d 点" % len(sl))

    # near_keep：2.0m 外不入库
    P = g.to_sensor(np.array([[2.0, 0.0, 0.50]]))
    sl = veto_core.ingest(P, g, prm)
    check("near_keep=1.25: 2.0m外不入库", len(sl) == 0, "got %d 点" % len(sl))

    # 半径方位语义：前扇区远、侧后环近；±60° 恰在边界算环（严格小于）
    check("半径: az=59°→sector_r", approx(float(veto_core.radius(np.radians(59), prm)), 1.0))
    check("半径: az=61°→ring_r", approx(float(veto_core.radius(np.radians(61), prm)), 0.55))
    check("半径: az=60°边界→ring_r", approx(float(veto_core.radius(np.radians(60), prm)), 0.55))

    # Geom.rebase 保体轴心（IMU 增量跟踪后 body_c 仍沿新前向 +0.05）
    gg = veto_core.Geom(veto_core.PRIOR_N, 0.486, body_along=0.05)
    gg.rebase(np.array([-0.5, 0.3, 0.8]))
    check("rebase: body_c 沿新前向 0.05",
          approx(float(gg.body_c @ gg.f), 0.05) and approx(float(gg.body_c @ gg.l), 0.0))


# ---------------- R2：只抑制不放大 ----------------
def test_r2_no_amplify():
    print("R2 只抑制不放大（残速抬升越权已删）")
    prm = veto_core.Params()

    # 空场放行：0.05 原样（起步死区是上游 speed_adapter 的职责，veto 不越权）
    vx, vy, st, blk = veto_core.apply_veto(0.05, 0.0, np.zeros((0, 2)), prm)
    check("空场: 0.05 原样放行", st == "PASS" and approx(vx, 0.05) and approx(vy, 0))

    # 未消解指令原样：侧向障碍、前进指令无朝向分量 → 不动它（哪怕 <地板）
    pts = np.array([[0.5, math.radians(90)]])
    vx, vy, st, blk = veto_core.apply_veto(0.10, 0.0, pts, prm)
    check("侧障碍+前进: 未消解→原样", st == "BLOCK_RING" and approx(vx, 0.10) and approx(vy, 0),
          "got (%.3f,%.3f) %s" % (vx, vy, st))

    # 正前拦截：朝向分量全消
    pts = np.array([[0.6, 0.0]])
    vx, vy, st, blk = veto_core.apply_veto(0.25, 0.0, pts, prm)
    check("正前0.6m: 全消→(0,0)", st == "BLOCK_SECTOR" and vx == 0 and vy == 0)
    check("blk 回报 (dh,az°)", blk is not None and approx(blk[0], 0.6) and approx(blk[1], 0))

    # ★ R2 回归哨兵：45° 障碍 + 0.25 输入 → 残速 0.177<地板 → 归零。
    # 旧版此处抬升成 (0, 0.25)——输出是输入的 √2 倍，越权必须死
    pts = np.array([[0.6, math.radians(45)]])
    vx, vy, st, blk = veto_core.apply_veto(0.25, 0.0, pts, prm)
    check("45°障碍+0.25: 残速<地板→归零(不抬升)", vx == 0 and vy == 0,
          "got (%.4f,%.4f) — 旧bug会是 (0,0.25)" % (vx, vy))

    # 45° 障碍 + 0.40 输入 → 消解朝向分量后残速 0.283≥地板 → 滑行。
    # 残差=输入减去(投影到障碍方向)→与障碍方向正交: (0.2,-0.2)，不放大
    vx, vy, st, blk = veto_core.apply_veto(0.40, 0.0, pts, prm)
    check("45°障碍+0.40: 残速≥地板→滑行",
          approx(vx, 0.2, 1e-9) and approx(vy, -0.2, 1e-9),
          "got (%.4f,%.4f)" % (vx, vy))
    check("滑行不放大: |out|≤|in|", math.hypot(vx, vy) <= 0.40 + 1e-9)

    # 极近全消：0.25m<touch_r，哪怕纯侧向速度也归零
    pts = np.array([[0.25, 0.0]])
    vx, vy, st, blk = veto_core.apply_veto(0.0, 0.30, pts, prm)
    check("极近0.25m: 全消平动", vx == 0 and vy == 0)

    # 身后障碍不拦前进（脱离方向永远放行）
    pts = np.array([[0.4, math.radians(180)]])
    vx, vy, st, blk = veto_core.apply_veto(0.30, 0.0, pts, prm)
    check("身后障碍: 前进放行", approx(vx, 0.30) and approx(vy, 0),
          "got (%.3f,%.3f)" % (vx, vy))

    # 性质测试（固定种子）：任意输入×任意障碍组，输出模长永不增
    rng = np.random.default_rng(42)
    ok = True
    bad = None
    for _ in range(2000):
        va = rng.uniform(0, 2 * math.pi)
        vs = rng.uniform(0, 0.6)
        vin = (vs * math.cos(va), vs * math.sin(va))
        nb = int(rng.integers(0, 4))
        blockers = np.column_stack([rng.uniform(0.15, 1.4, nb),
                                    rng.uniform(-math.pi, math.pi, nb)])
        vx, vy, st, blk = veto_core.apply_veto(vin[0], vin[1], blockers, prm)
        if math.hypot(vx, vy) > math.hypot(vin[0], vin[1]) + 1e-9:
            ok, bad = False, ("放大", vin, blockers.tolist())
            break
        if st == "PASS" and (not approx(vx, vin[0], 1e-9) or not approx(vy, vin[1], 1e-9)):
            ok, bad = False, ("PASS被改动", vin, blockers.tolist())
            break
    check("性质: 2000组随机输入 |out|≤|in| 且 PASS=原样", ok, "违例 %s" % (bad,))


# ---------------- R3：采集时效 ----------------
def test_r3_stamp():
    print("R3 采集时效（刚收到≠刚采集）")
    prm = veto_core.Params()
    ok, why = veto_core.stamp_ok(100.0, 99.9, 100.15, prm)
    check("新鲜前进: 放行", ok and why == "ok")
    ok, why = veto_core.stamp_ok(100.0, None, 100.20, prm)
    check("首帧新鲜: 放行", ok)
    ok, why = veto_core.stamp_ok(100.0, None, 103.0, prm)
    check("超龄 3.0s>2.5: 拒('age')", not ok and why == "age")
    ok, why = veto_core.stamp_ok(100.0, 100.0, 100.1, prm)
    check("重复戳(冻结): 拒('frozen')", not ok and why == "frozen")
    ok, why = veto_core.stamp_ok(99.5, 100.0, 100.1, prm)
    check("戳回退: 拒('frozen')", not ok and why == "frozen")
    prm_off = veto_core.Params(stamp_check=False)
    ok, why = veto_core.stamp_ok(100.0, 100.0, 999.0, prm_off)
    check("闸门关闭(回放模式): 放行", ok and why == "off")


# ---------------- R4：指令仲裁 ----------------
def test_r4_arbiter():
    print("R4 停车锁存与槽位竞争（评审纯复现：stop 后紧跟速度→stop 消失）")
    arb = bridge_core.CmdArbiter(watchdog_s=0.5)

    # ★ 评审复现：stop 入队后紧跟一帧速度——旧版槽里只剩 ('move',...)
    arb.push_vel(0.05, 0.0, 0.0, now=1.0)
    arb.push_cmd("stop")
    item = arb.next_item(now=1.01)
    check("stop 不被随后速度顶掉", item == ("cmd", "stop"), "got %s" % (item,))
    check("在途速度已被清除", arb.next_item(now=1.02) is None)

    # halt 期间速度一律丢弃并计数
    for _ in range(5):
        arb.push_vel(0.30, 0.0, 0.0, now=1.1)
    check("halt 锁存: 5帧速度全丢弃", arb.dropped_vel == 5 and arb.halted)
    check("丢弃后队列仍空", arb.next_item(now=1.2) is None)

    # resume 解锁：hi 通道先出回执，随后速度恢复流动
    arb.push_cmd("resume")
    check("resume 即刻解锁", not arb.halted)
    arb.push_vel(0.25, 0.0, 0.0, now=1.3)
    check("先出 resume 回执", arb.next_item(now=1.3) == ("cmd", "resume"))
    check("速度恢复流动", arb.next_item(now=1.31) == ("move", 0.25, 0.0, 0.0))

    # 看门狗：瞬态零速，不锁存；速度恢复即续走（上帧速度 t=1.3，>1.8 触发）
    check("断流0.4s内不动", arb.next_item(now=1.7) is None)
    item = arb.next_item(now=1.85)
    check("断流>0.5s: 看门狗零速", item == ("move", 0.0, 0.0, 0.0), "got %s" % (item,))
    check("看门狗不锁存(只发一次)", arb.next_item(now=1.9) is None and not arb.halted)
    arb.push_vel(0.20, 0.0, 0.0, now=2.0)
    check("速度恢复即续走", arb.next_item(now=2.0) == ("move", 0.20, 0.0, 0.0))

    # 姿态指令视为操作员接管：balance 解锁 halt
    arb2 = bridge_core.CmdArbiter(watchdog_s=0.5)
    arb2.push_cmd("stop")
    check("stop 即刻锁存", arb2.halted)
    arb2.push_cmd("balance")
    check("balance 解锁", not arb2.halted)

    # hi 通道 FIFO 保序
    arb3 = bridge_core.CmdArbiter(watchdog_s=0.5)
    arb3.push_cmd("standup")
    arb3.push_cmd("damp")
    check("hi FIFO 保序", arb3.next_item(0.0) == ("cmd", "standup")
          and arb3.next_item(0.0) == ("cmd", "damp"))

    # stop 入队时刻即清在途速度（不等 RPC 执行完）
    arb4 = bridge_core.CmdArbiter(watchdog_s=0.5)
    arb4.push_vel(0.4, 0.0, 0.0, now=0.0)
    arb4.push_cmd("stop")
    arb4.push_cmd("resume")
    items = [arb4.next_item(0.1), arb4.next_item(0.2)]
    check("在途速度被 stop 清除(只剩 stop,resume)",
          items == [("cmd", "stop"), ("cmd", "resume")], "got %s" % (items,))


def main():
    test_r1_geometry()
    test_r2_no_amplify()
    test_r3_stamp()
    test_r4_arbiter()
    print("\n==== 回归汇总: %d 项断言, 失败 %d ====" % (N_CHECKS[0], len(FAILS)))
    if FAILS:
        print("失败项: %s" % ", ".join(FAILS))
        return 1
    print("全部通过 ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())