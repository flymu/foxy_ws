#!/usr/bin/env python3
"""veto_shadow_watch.py — 行走态影子旁观记录器（2026-11-07 蠕行实弹复测）

creep_test 在 WS 容器里向 /cmd_vel_raw 发 20Hz 恒速（**测试前杀掉 speed_adapter**
保证单输入源），本工具在 Orin 上旁听全程：
  * 双状态流 /veto/status(Python 生产) + /veto/status_shadow(C++)：字段变化
    即打一行（state/blk/calib 翻转时刻两侧对齐 = 决策级对拍）；
  * 输入流 /cmd_vel_raw + 双输出流 /go2/cmd_vel & /go2/cmd_vel_shadow 全记录；
  * 结束汇总：①输入归属配对（每输入在 [t, t+45ms] 找两侧各一条未用输出，
    单输入源 50ms 间距窗口不重叠）逐对 diff；②每秒速度画像（输出数/非零数/
    非零均值）两侧并排；③状态翻转时间线。
用法（Orin）: python3 veto_shadow_watch.py [时长s，默认60]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


class Watch(Node):
    def __init__(self):
        super().__init__("veto_shadow_watch")
        qbe = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cb_in, qbe)
        self.create_subscription(Twist, "/go2/cmd_vel", self.cb_py, qbe)
        self.create_subscription(Twist, "/go2/cmd_vel_shadow", self.cb_cpp, qbe)
        self.create_subscription(String, "/veto/status", self.cb_st_py, 5)
        self.create_subscription(String, "/veto/status_shadow", self.cb_st_cpp, 5)
        self.t0 = time.monotonic()
        self.t_end = self.t0 + DUR
        self._done = False
        self.inputs, self.out_py, self.out_cpp = [], [], []
        self.st_py, self.st_cpp = [], []
        self.last = {"py": None, "cpp": None}
        self.create_timer(0.05, self.tick)

    def t(self):
        return time.monotonic() - self.t0

    def tick(self):
        if time.monotonic() >= self.t_end and not self._done:
            self._done = True
            try:
                self.finish()
            finally:
                sys.stdout.flush()
                rclpy.shutdown()

    def cb_in(self, m):
        self.inputs.append((self.t(), m.linear.x, m.linear.y, m.angular.z))

    def cb_py(self, m):
        self.out_py.append((self.t(), m.linear.x, m.linear.y, m.angular.z))

    def cb_cpp(self, m):
        self.out_cpp.append((self.t(), m.linear.x, m.linear.y, m.angular.z))

    @staticmethod
    def _key(s):
        return dict(x.split("=", 1) for x in s.split() if "=" in x)

    def _st_line(self, side, s):
        kv = self._key(s)
        keys = ("state", "blk", "calib")
        cur = tuple(kv.get(k, "-") for k in keys)
        if self.last[side] is not None and cur != self.last[side]:
            print("t=%5.1f %-3s %s" % (
                self.t(), side.upper(),
                " ".join("%s=%s" % (k, v) for k, v in zip(keys, cur))), flush=True)
        self.last[side] = cur

    def cb_st_py(self, m):
        self.st_py.append((self.t(), m.data))
        self._st_line("py", m.data)

    def cb_st_cpp(self, m):
        self.st_cpp.append((self.t(), m.data))
        self._st_line("cpp", m.data)

    def finish(self):
        print("\n==== 行走态影子旁观汇总 ====", flush=True)
        print("输入 %d | py 输出 %d | cpp 输出 %d | st %d/%d"
              % (len(self.inputs), len(self.out_py), len(self.out_cpp),
                 len(self.st_py), len(self.st_cpp)), flush=True)

        # ① 输入归属配对（单输入源）
        pairs = []
        ip = cp = 0  # 双游标
        for t_in, *_ in self.inputs:
            while ip < len(self.out_py) and self.out_py[ip][0] < t_in:
                ip += 1
            while cp < len(self.out_cpp) and self.out_cpp[cp][0] < t_in:
                cp += 1
            if (ip < len(self.out_py) and self.out_py[ip][0] <= t_in + 0.045
                    and cp < len(self.out_cpp)
                    and self.out_cpp[cp][0] <= t_in + 0.045):
                pairs.append((self.out_py[ip], self.out_cpp[cp]))
                ip += 1
                cp += 1
        n = mm = 0
        samples = []
        for sp, sc in pairs:
            n += 1
            bad = (abs(sp[1] - sc[1]) > 2e-3 or abs(sp[2] - sc[2]) > 2e-3
                   or abs(sp[3] - sc[3]) > 1e-6)
            if bad:
                mm += 1
                if len(samples) < 8:
                    samples.append("t=%.2f py=(%.3f,%.3f,%.3f) cpp=(%.3f,%.3f,%.3f)"
                                   % (sp[0], sp[1], sp[2], sp[3],
                                      sc[1], sc[2], sc[3]))
        print("配对 %d 对, mismatch %d (%.2f%%)"
              % (n, mm, (100.0 * mm / n) if n else 0.0), flush=True)
        for s in samples:
            print("    " + s, flush=True)

        # ② 每秒画像
        def prof(lst):
            bins = {}
            for t, vx, vy, wz in lst:
                b = int(t)
                c, nz, s = bins.get(b, (0, 0, 0.0))
                c += 1
                if abs(vx) > 0.05:
                    nz += 1
                    s += vx
                bins[b] = (c, nz, s)
            return bins

        bp, bc = prof(self.out_py), prof(self.out_cpp)
        print("每秒画像 py(n/非零/Σvx) | cpp: ", flush=True)
        for b in sorted(set(bp) | set(bc)):
            p, c = bp.get(b, (0, 0, 0.0)), bc.get(b, (0, 0, 0.0))
            print("  t=%02d py=(%3d/%3d/%+.2f) cpp=(%3d/%3d/%+.2f)%s"
                  % (b, p[0], p[1], p[2], c[0], c[1], c[2],
                     "" if p == c else "  ←差"), flush=True)

        # ③ 状态尾
        print("py status 尾: %s" % (self.st_py[-1][1][:150] if self.st_py else "无"),
              flush=True)
        print("cpp status 尾: %s"
              % (self.st_cpp[-1][1][:150] if self.st_cpp else "无"), flush=True)


def main():
    rclpy.init()
    n = Watch()
    try:
        rclpy.spin(n)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
