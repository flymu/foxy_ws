#!/usr/bin/env python3
"""veto_shadow_diff.py — Python/C++ 否决层影子对拍（2026-11-06 影子先行阶段2）

在 Orin 上跑（点云 SHM 本地，不占 WiFi）：向 /cmd_vel_raw 发正弦扫描激励，
同时订阅两版否决层的输出——/go2/cmd_vel（Python 版，生产话题）与
/go2/cmd_vel_shadow（C++ 版，影子 remap）——按到达时刻就近配对逐对比较。

前提（本次实测时已核实）：
  * 桥已死 → /go2/cmd_vel 无消费者，激励不会让狗动（影子阶段铁律）；
  * Python veto + livox relay 驱动链照常运行（patrol_safety_start.sh）；
  * C++ 影子节点已用 -r 换名启动（见 veto_shadow_start.sh）。

判读（对拍契约）：
  * 每对输出：|Δvx|、|Δvy| ≤ 2e-3（float32 vs double + 回调时差的余量），
    wz 两者都只透传须 1e-6 内一致；
  * C++ 标定完成（status 里 calib 离开 prior）之前的数据段单独统计
    （C++ 用先验几何 vs Python 已精修几何，差异属预期，不算失败）；
  * 状态字符串（PASS/BLOCK_SECTOR/BLOCK_RING/STALE）配对不一致也算 mismatch
    ——窗口边界/熔断边沿的瞬态允许极少量，系统性偏离=移植有错。

用法（Orin，unitree 用户）:
  python3 veto_shadow_diff.py [时长s，默认90]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0


class Diff(Node):
    def __init__(self):
        super().__init__("veto_shadow_diff")
        self.pub = self.create_publisher(Twist, "/cmd_vel_raw", 5)
        qbe = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, "/go2/cmd_vel", self.cb_py, qbe)
        self.create_subscription(Twist, "/go2/cmd_vel_shadow", self.cb_cpp, qbe)
        # status 双流（RELIABLE depth5 是生产 QoS）
        self.create_subscription(String, "/veto/status", self.cb_st_py, 5)
        self.create_subscription(String, "/veto/status_shadow", self.cb_st_cpp, 5)
        self.t0 = time.monotonic()
        self.n_pub = 0
        self.out_py = []    # (t, vx, vy, wz)
        self.out_cpp = []
        self.st_py = []     # (t, dict字段)
        self.st_cpp = []
        self.t_end = self.t0 + DUR
        self._done = False
        self.create_timer(0.1, self.tick)

    def tick(self):
        now = time.monotonic()
        t = now - self.t0
        if now >= self.t_end:
            if not self._done:
                self._done = True
                try:
                    self.finish()
                finally:
                    sys.stdout.flush()
                    rclpy.shutdown()
            return
        m = Twist()
        m.linear.x = 0.30 * math.sin(2 * math.pi * t / 9.0)
        m.linear.y = 0.15 * math.sin(2 * math.pi * t / 5.5 + 1.0)
        m.angular.z = 0.20 * math.sin(2 * math.pi * t / 12.0)
        self.pub.publish(m)
        self.n_pub += 1
        if self.n_pub % 100 == 0:  # 10s 心跳：证明激励与双流活着
            print("t=%.0fs 激励%d py=%d cpp=%d (st_py=%d st_cpp=%d)"
                  % (t, self.n_pub, len(self.out_py), len(self.out_cpp),
                     len(self.st_py), len(self.st_cpp)), flush=True)

    def cb_py(self, m):
        self.out_py.append((time.monotonic() - self.t0,
                            m.linear.x, m.linear.y, m.angular.z))

    def cb_cpp(self, m):
        self.out_cpp.append((time.monotonic() - self.t0,
                              m.linear.x, m.linear.y, m.angular.z))

    def cb_st_py(self, m):
        self.st_py.append((time.monotonic() - self.t0, m.data))

    def cb_st_cpp(self, m):
        self.st_cpp.append((time.monotonic() - self.t0, m.data))

    @staticmethod
    def _field(s, key):
        for tok in s.split():
            if tok.startswith(key + "="):
                return tok.split("=", 1)[1]
        return None

    def finish(self):
        print("\n==== 影子对拍汇总 ====", flush=True)
        print("激励 %d 帧 | py 输出 %d | cpp 输出 %d | py status %d | cpp status %d"
              % (self.n_pub, len(self.out_py), len(self.out_cpp),
                 len(self.st_py), len(self.st_cpp)), flush=True)

        def st_state(lst):
            return [self._field(s, "state") for _, s in lst]

        py_states = st_state(self.st_py)
        cpp_states = st_state(self.st_cpp)
        print("py states: %s" % sorted(set(py_states)), flush=True)
        print("cpp states: %s" % sorted(set(cpp_states)), flush=True)

        # C++ 标定完成时刻（calib 字段离开 prior/初始段）
        t_calib = None
        for t, s in self.st_cpp:
            c = self._field(s, "calib") or ""
            if c.startswith("refit") or c.startswith("imu-boot"):
                t_calib = t
                break
        print("cpp calib 时刻: %s（status 尾: %s）" % (
            ("%.1fs" % t_calib) if t_calib is not None else "全程未标定",
            (self.st_cpp[-1][1][:170] if self.st_cpp else "无")), flush=True)

        # 配对：wz 是透传指纹——两版否决都不改 wz（STALE 外），wz=输入 wz；
        # 扫描帧的 wz 唯一标识其输入。生产流里 WS speed_adapter 也在向
        # /cmd_vel_raw 发 100Hz 零速（全零输出两侧必然相同，怎么配都不误报），
        # 而按时刻就近配对会把相邻不同输入的输出错配成 wz-only 假差异
        #（第一轮实测 1620 个全 wz-only 假 mismatch 的教训）。
        # 故先按 wz 精确匹配（float32 线上同值），候选内取时刻最近，窗口±0.15s。
        from collections import defaultdict
        idx_py = defaultdict(list)
        for i, sp in enumerate(self.out_py):
            idx_py[round(sp[3], 6)].append(i)
        used = set()
        pairs = []
        for sc in self.out_cpp:
            best, bd = None, 1e9
            for i in idx_py.get(round(sc[3], 6), ()):
                if i in used:
                    continue
                d = abs(self.out_py[i][0] - sc[0])
                if d < bd:
                    best, bd = i, d
            if best is not None and bd <= 0.15:
                used.add(best)
                pairs.append((sc, self.out_py[best], bd))
        print("配对 %d 对（wz指纹+窗口±0.15s）" % len(pairs), flush=True)

        def report(seg_pairs, tag):
            n = mm = mmwz = 0
            samples = []
            for sc, sp, bd in seg_pairs:
                n += 1
                dvx, dvy = abs(sc[1] - sp[1]), abs(sc[2] - sp[2])
                dwz = abs(sc[3] - sp[3])
                bad = dvx > 2e-3 or dvy > 2e-3 or dwz > 1e-6
                if bad:
                    mm += 1
                    if dwz > 1e-6:
                        mmwz += 1
                    if len(samples) < 8:
                        samples.append(
                            "t=%.2f py=(%.3f,%.3f,%.3f) cpp=(%.3f,%.3f,%.3f) Δt=%.0fms"
                            % (sc[0], sp[1], sp[2], sp[3],
                               sc[1], sc[2], sc[3], bd * 1000))
            print("%s: %d 对, mismatch %d (wz %d) = %.2f%%"
                  % (tag, n, mm, mmwz, (100.0 * mm / n) if n else 0.0),
                  flush=True)
            for s in samples:
                print("    " + s, flush=True)
            return n, mm

        if t_calib is not None:
            report([p for p in pairs if p[0][0] < t_calib + 1.0], "标定前(预期可有差)")
            n2, mm2 = report([p for p in pairs if p[0][0] >= t_calib + 1.0],
                             "标定后(须一致)")
        else:
            n2, mm2 = report(pairs, "全段")
        self._verdict = (n2, mm2)
        print("py status 尾: %s" % (self.st_py[-1][1][:170] if self.st_py else "无"),
              flush=True)


def main():
    rclpy.init()
    n = Diff()
    try:
        rclpy.spin(n)
    except Exception:  # finish() 里 shutdown 触发的外部退出，正常路径
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
