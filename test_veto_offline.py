#!/usr/bin/env python3
"""test_veto_offline.py — obstacle_veto 离线回放测试（狗不上电也能跑）

一个节点干三件事：
  1. 假规划器：50Hz 恒发 (0.25,0,0) → /cmd_vel_raw（veto 的输入）
  2. 记录器：订阅 /veto/cmd_out + /veto/status 记时间线
  3. 结束打印统计：全零/受限/放行占比、拦截距离-方位分布、末段 STALE 行为
退出码（G1：测试失败必须非零退出）：0=过 2=没收到输出 3=首段无云期没熔断
配套完整回放链（2026-11-05 G1 起，veto 吃 /livox/points2，bag 里只有
CustomMsg——必须有 relay；bag 戳是录制时刻的旧 epoch——必须关戳闸）：
  # 终端1: relay（需先在 WS 容器里 colcon build livox_relay）
  source /home/mu/foxy_ws/install_x86/setup.bash && ros2 run livox_relay relay_node
  # 终端2: bag（隔离 domain，防真机订阅者串台）
  ROS_DOMAIN_ID=77 ros2 bag play /home/mu/bags/campus_map
  # 终端3: veto（同 domain + remap 别碰真 /go2/cmd_vel + 关戳闸）
  ROS_DOMAIN_ID=77 python3 ~/foxy_ws/obstacle_veto.py \
      --ros-args -r /go2/cmd_vel:=/veto/cmd_out -p stamp_check:=false
  # 终端4: 本测试（同 domain）
  ROS_DOMAIN_ID=77 timeout -s INT 260 python3 ~/foxy_ws/test_veto_offline.py [秒]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0


def main():
    rclpy.init()
    node = Node("veto_offline_test")
    qr = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=10)
    qp = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=1)

    rec = {"cmd": [], "status": [], "t0": time.monotonic()}

    node.create_publisher(Twist, "/cmd_vel_raw", qp)

    def tick_pub():
        if time.monotonic() - rec["t0"] > DUR:
            return
        m = Twist()
        m.linear.x = 0.25
        node.pub.publish(m)

    node.pub = node.create_publisher(Twist, "/cmd_vel_raw", qp)
    node.create_timer(0.02, tick_pub)  # 50Hz 假规划器

    def on_cmd(m):
        rec["cmd"].append((time.monotonic() - rec["t0"],
                           m.linear.x, m.linear.y, m.angular.z))

    def on_status(m):
        rec["status"].append((time.monotonic() - rec["t0"], m.data))

    qbe = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    node.create_subscription(Twist, "/veto/cmd_out", on_cmd, qbe)
    node.create_subscription(String, "/veto/status", on_status, qr)

    # spin() 常驻线程代替 spin_once 轮询：spin_once 每次重建 wait set，
    # 50Hz cmd + 50Hz 定时器 + 1Hz status 会把单线程执行器拖到跟不上，
    # RELIABLE 深度10队列溢出后 DDS 流在 ~35-40s 静默断流（前几轮实测）
    import threading
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        end = rec["t0"] + DUR
        while rclpy.ok() and time.monotonic() < end and spin.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass

    # ---- 统计 ----
    cmds = rec["cmd"]
    print("\n==== veto 离线测试汇总（时长 %.0fs）====" % (time.monotonic() - rec["t0"]))
    rc = 0
    if not cmds:
        print("!! 未收到任何 /veto/cmd_out — veto 没跑起来或 remap 错")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)
    dt = cmds[-1][0] - cmds[0][0]
    print("输出 %d 条 / %.0fs = %.0fHz" % (len(cmds), dt, len(cmds) / max(0.1, dt)))
    full_zero = sum(1 for c in cmds if abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6
                    and abs(c[3]) < 1e-6)
    clipped = sum(1 for c in cmds if 1e-6 < abs(c[1]) < 0.249 or abs(c[2]) > 1e-6)
    passed = len(cmds) - full_zero - clipped
    print("全零(熔断/早启) %d (%.0f%%) | 分量削减 %d (%.0f%%) | 原样放行 %d (%.0f%%)"
          % (full_zero, 100.0 * full_zero / len(cmds),
             clipped, 100.0 * clipped / len(cmds),
             passed, 100.0 * passed / len(cmds)))

    states = {}
    blks = []
    for t, s in rec["status"]:
        kv = dict(x.split("=", 1) for x in s.split() if "=" in x)
        st = kv.get("state", "?")
        states[st] = states.get(st, 0) + 1
        # status 里 blk=dh=X 与 az=Y 是两个空白分隔 token（publish_status 的格式）
        if st.startswith("BLOCK") and "dh" in kv.get("blk", ""):
            blks.append((float(kv["blk"].split("=")[1]), float(kv["az"])))
    print("status 状态分布(1Hz):", {k: v for k, v in sorted(states.items())})
    if blks:
        ds = [b[0] for b in blks]
        azs = [b[1] for b in blks]
        import math
        fwd = sum(1 for a in azs if abs(a) < 60)
        print("拦截 %d 拍: 距离 min/med/max = %.2f/%.2f/%.2f m, 前扇区占 %.0f%%"
              % (len(blks), min(ds), sorted(ds)[len(ds) // 2], max(ds),
                 100.0 * fwd / len(blks)))
    # 末段行为（bag 停后应 STALE→全零）
    tail = [c for c in cmds if c[0] > cmds[-1][0] - 3.0]
    tail_zero = sum(1 for c in tail if abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6
                    and abs(c[3]) < 1e-6)
    print("末 3s: %d 条, 全零 %d (%.0f%%) — 期望 bag 停后 ~100%%"
          % (len(tail), tail_zero, 100.0 * tail_zero / max(1, len(tail))))
    # 首段行为（无云期应全零）
    head = [c for c in cmds if c[0] < 2.0]
    if head:
        head_zero = sum(1 for c in head if abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6
                        and abs(c[3]) < 1e-6)
        print("首 2s: %d 条, 全零 %d — 期望 100%%（无云不动）"
              % (len(head), head_zero))
        if head_zero < len(head):
            print("!! 首段无云期未全零 — 熔断失效，见 obstacle_veto 设计4")
            rc = 3
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
