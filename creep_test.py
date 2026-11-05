#!/usr/bin/env python3
"""creep_test.py — veto 真机蠕行测试（区分"钉死"与"贴墙滑行"）

向 /cmd_vel_raw 发恒速假规划器指令，旁观 /veto/status + /go2/cmd_vel（veto 实际输出）：
  - PASS: 跑道净空，狗全速前进（out=VX）
  - BLOCK 但 out>0: 朝向障碍的分量被消，狗沿安全方向滑行绕行（贴墙特性）
  - out≈0 持续 2s: 真被钉死（极近全消/正对消光），收手
  - STALE 持续 2s: 雷达断供熔断，收手
硬超时 25s 自动停（停发后桥 0.5s 看门狗自然停车）。
用法（foxy-live 容器内）: timeout -s INT 30 python3 ~/foxy_ws/creep_test.py [速度]
⚠️ 2026-11-04 速度门已实测破案：本狗起步阈值 0.20~0.25 之间
   （Move(0.25) 走、0.20/0.15/0.10 假死）。默认速度用 0.25（实测可靠起步）。
   判"钉死"时区分"veto 拦"与"速度低于狗起步阈值"——veto out 高于
   FREEZE_V 但狗没动 = 速度阈值问题，不是 veto 的锅。
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
HARD_TIMEOUT = 25.0
HOLD = 2.0          # 钉死/STALE 判定持续时长
FREEZE_V = 0.03     # veto 输出平动速度低于此 = 真被钉住

rclpy.init()
node = Node("creep_test")
# veto 输出 /go2/cmd_vel 是 BEST_EFFORT depth1，订阅必须 BE 才匹配
qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1)
qr = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=5)
pub = node.create_publisher(Twist, "/cmd_vel_raw",
                            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                       history=HistoryPolicy.KEEP_LAST, depth=1))

t0 = time.monotonic()
out = {"v": None, "t": None}
hold_since = {"kind": None, "t": 0.0}
done = {"reason": None}


def on_out(m):
    out["v"] = math.hypot(m.linear.x, m.linear.y)
    out["t"] = time.monotonic()


def on_status(m):
    kv = dict(x.split("=", 1) for x in m.data.split() if "=" in x)
    st = kv.get("state", "?")
    now = time.monotonic()
    fresh = out["t"] is not None and now - out["t"] < 0.3
    ov = "%.3f" % out["v"] if fresh else "-"
    print("%5.1fs %s blk=%s out=%s calib=%s" % (
        now - t0, st, kv.get("blk", "-"), ov, kv.get("calib", "-")), flush=True)
    kind = None
    if st == "STALE":
        kind = "STALE熔断（雷达断供，veto全零）"
    elif fresh and out["v"] < FREEZE_V:
        kind = "veto钉死（输出≈0，狗进不去）"
    if kind:
        if hold_since["kind"] != kind:
            hold_since["kind"] = kind
            hold_since["t"] = now
            print("*** %s：持续 %ds 后收手 ***" % (kind, HOLD), flush=True)
        elif now - hold_since["t"] > HOLD:
            done["reason"] = kind
    else:
        hold_since["kind"] = None


node.create_subscription(Twist, "/go2/cmd_vel", on_out, qb)
node.create_subscription(String, "/veto/status", on_status, qr)


def tick():
    if done["reason"] or time.monotonic() - t0 > HARD_TIMEOUT:
        return
    m = Twist()
    m.linear.x = VX
    pub.publish(m)


node.create_timer(0.05, tick)  # 20Hz
print("蠕行开始: %.2f m/s, 硬超时 %.0fs, 等 /veto/status ..." % (VX, HARD_TIMEOUT),
      flush=True)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
while not done["reason"] and time.monotonic() - t0 < HARD_TIMEOUT + 2:
    time.sleep(0.2)
print("==== 结束: %s, 历时 %.1fs ====" % (
    done["reason"] or "硬超时（25s 未钉死——一路畅通或持续滑行绕行）",
    time.monotonic() - t0), flush=True)
node.destroy_node()
rclpy.shutdown()
