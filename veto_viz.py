#!/usr/bin/env python3
"""veto_viz.py — veto 视角的障碍诊断可视化（RViz + 控制台 3D 包围盒）

订阅 /livox/points2（Orin 上 livox_relay 的 PointCloud2 输出，D2 修复：
np.frombuffer 零拷贝，不再是 CustomMsg ctypes 天花板受害者），跑 veto 同款
管线（频带+自体掩膜+近场），把"veto 认为是障碍"的点云和聚类 3D 包围盒发布出来：
  /veto_diag/band  sensor_msgs/PointCloud2  频带内近场点（聚类着色，灰=未聚类）
  /veto_diag/boxes visualization_msgs/MarkerArray  每簇线框立方体 + 文本标签
  TF livox_frame -> veto_dog（重力对齐狗视角，原点=传感器地面投影）
RViz Fixed Frame 用 veto_dog：狗朝 +X，左 +Y，上 +Z，地面水平。
控制台每 2s 打一次各簇包围盒（狗身坐标 + 是否触发拦截）。
用法（Orin 上，需先起 livox_relay）: bash /home/unitree/patrol_safety_start.sh viz
"""
import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

N_UP = np.array([-0.7330, 0.0001, 0.6802]); N_UP /= np.linalg.norm(N_UP)
H_SENSOR = 0.486
BAND = (0.20, 0.75)
SELF_BOX = (0.25, 0.15, -0.30, 0.35)
NEAR_KEEP = 1.25
SECTOR_HALF = np.deg2rad(60.0)
SECTOR_R, RING_R = 1.0, 0.55
CELL = 0.12          # 聚类格边长(m)
MIN_CLUS = 30        # 单帧成簇最少点数
F_AX = np.array([1.0, 0.0, 0.0]); F_AX = F_AX - (F_AX @ N_UP) * N_UP; F_AX /= np.linalg.norm(F_AX)
L_AX = np.cross(N_UP, F_AX)
BODY_C = 0.05 * F_AX
PALETTE = [(0.23, 0.44, 0.83), (0.88, 0.51, 0.08), (0.18, 0.62, 0.36),
           (0.58, 0.40, 0.74), (0.13, 0.60, 0.69), (0.78, 0.36, 0.36)]


def to_sensor(x, y, z):
    x, y, z = np.asarray(x, float), np.asarray(y, float), np.asarray(z, float)
    if x.ndim == 0:
        return x * F_AX + y * L_AX + (z - H_SENSOR) * N_UP
    return (x[:, None] * F_AX + y[:, None] * L_AX
            + (z - H_SENSOR)[:, None] * N_UP)


class Viz(Node):
    def __init__(self):
        super().__init__('veto_viz')
        self.pub_c = self.create_publisher(PointCloud2, '/veto_diag/band', 5)
        self.pub_m = self.create_publisher(MarkerArray, '/veto_diag/boxes', 5)
        self.tf = TransformBroadcaster(self)
        self.tracked = []   # (cid, centroid(2,), color)
        self.next_cid = 0
        self.last_print = 0.0
        self.last_age = 0.0
        self.last_proc_ms = 0.0
        self._t_entry = None
        self.t0 = time.monotonic()
        # 深度2：处理一旦跟不上就丢旧帧——诊断视图永远显示最新（宁缺勿旧）
        qb = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(PointCloud2, '/livox/points2', self.on_cloud, qb)
        self.create_timer(1.0, self.pub_tf)

    def pub_tf(self, stamp=None):
        m = TransformStamped()
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        m.header.stamp = stamp
        m.header.frame_id = 'livox_frame'
        m.child_frame_id = 'veto_dog'
        # TF 语义（11-05 修）：parent=livox_frame, child=veto_dog 存的是
        # child→parent 变换，即 p_sensor = R·p_dog + t——与 to_sensor() 同一公式。
        # 旧版错存了 M=rows(f,l,n)（sensor→dog 的分量提取）和平移 -H·ẑ_sensor：
        # RViz 显示时应用 R^T → 场景被多转 -2×47°≈-94°，地板立起来
        # （频带云不含地板所以一直没人发现，稀疏全景放进地板才暴露）。
        # 正确版：R = M^T（列才是 f,l,n），t = to_sensor(0,0,0) = -H·n。
        t = -H_SENSOR * N_UP
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = float(t[0]), float(t[1]), float(t[2])
        q = mat2quat(np.vstack([F_AX, L_AX, N_UP]).T)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        self.tf.sendTransform(m)

    def on_cloud(self, m):
        if m.width < 2000 or m.point_step != 16:
            return
        self._t_entry = time.monotonic()
        # 摄入滞后（壁钟-stamp）——>2s = 驱动端积压/时钟域异常，实时盯防
        self.last_age = time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.pub_tf(m.header.stamp)   # TF 与云同戳发布：RViz 变换查询零等待
        # D2 修复：np.frombuffer 零拷贝（旧 CustomMsg ctypes 解析 ~190ms/帧）
        arr = np.frombuffer(m.data, dtype=np.float32).reshape(-1, 4)
        P = arr[:, :3]
        h = P @ N_UP + H_SENSOR
        band = (h > BAND[0]) & (h < BAND[1])
        band &= ~((np.abs(P[:, 0]) < SELF_BOX[0]) & (np.abs(P[:, 1]) < SELF_BOX[1])
                  & (P[:, 2] > SELF_BOX[2]) & (P[:, 2] < SELF_BOX[3]))
        Q = P[band] - BODY_C
        vh = Q - (Q @ N_UP)[:, None] * N_UP
        px, py, ph = vh @ F_AX, vh @ L_AX, h[band]
        keep = np.hypot(px, py) < NEAR_KEEP
        px, py, ph = px[keep], py[keep], ph[keep]
        if len(px) == 0:
            return
        clus = cluster(px, py)
        refl = arr[:, 3][band][keep]   # intensity 列 = livox reflectivity(0~255)
        # 簇身份跨帧匹配（最近未占用质心）
        raw = []
        for idx in np.unique(clus):
            k = clus == idx
            if k.sum() < MIN_CLUS:
                continue
            raw.append({'x': px[k], 'y': py[k], 'h': ph[k], 'r': refl[k],
                        'cen': (np.median(px[k]), np.median(py[k]))})
        raw.sort(key=lambda b: b['cen'][1])          # 稳定顺序：从左到右
        claimed = set()
        boxes = []
        for b in raw:
            cx, cy = b['cen']
            best, bestd = None, 0.45
            for cid, cen, col in self.tracked:
                if cid in claimed:
                    continue
                d = np.hypot(cx - cen[0], cy - cen[1])
                if d < bestd:
                    best, bestd = (cid, np.array([cx, cy]), col), d
            if best is None:
                col = PALETTE[self.next_cid % len(PALETTE)]
                best = (self.next_cid, np.array([cx, cy]), col)
                self.next_cid += 1
            claimed.add(best[0])
            b.update(cid=best[0], color=best[2], n=int(len(b['x'])))
            boxes.append(b)
        self.tracked = [(b['cid'], np.array(b['cen']), b['color']) for b in boxes]
        self.publish_cloud(px, py, ph, boxes, m.header.stamp)
        self.publish_boxes(boxes, m.header.stamp)
        now = time.monotonic()
        if now - self.last_print > 2.0:
            self.last_print = now
            self.print_boxes(boxes)
        self.last_proc_ms = (time.monotonic() - self._t_entry) * 1000

    def publish_cloud(self, px, py, ph, boxes, stamp):
        n = len(px)
        rgb = np.zeros(n, np.uint32)
        rgb[:] = (160 << 16) | (160 << 8) | 160
        for b in boxes:
            col = b['color']
            val = (int(col[0] * 255) << 16) | (int(col[1] * 255) << 8) | int(col[2] * 255)
            m2 = (px >= b['x'].min()) & (px <= b['x'].max()) & \
                 (py >= b['y'].min()) & (py <= b['y'].max()) & \
                 (ph >= b['h'].min()) & (ph <= b['h'].max())
            rgb[m2] = val
        ps = to_sensor(px, py, ph)
        buf = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        buf['x'], buf['y'], buf['z'] = ps[:, 0], ps[:, 1], ps[:, 2]
        buf['rgb'] = rgb
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'livox_frame'
        msg.height, msg.width = 1, n
        msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
        msg.point_step, msg.row_step = 16, 16 * n
        msg.data = buf.tobytes()
        msg.is_bigendian, msg.is_dense = False, True
        self.pub_c.publish(msg)

    def publish_boxes(self, boxes, stamp):
        ma = MarkerArray()
        for i, b in enumerate(boxes):
            x0, x1 = b['x'].min(), b['x'].max()
            y0, y1 = b['y'].min(), b['y'].max()
            z0, z1 = b['h'].min(), b['h'].max()
            corners = [to_sensor(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
            edges = [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6), (5, 7), (6, 7),
                     (0, 4), (1, 5), (2, 6), (3, 7)]
            mk = Marker()
            mk.header.stamp, mk.header.frame_id = stamp, 'livox_frame'
            mk.ns, mk.id, mk.type, mk.action = 'box', b['cid'], Marker.LINE_LIST, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.02
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*b['color'], 0.95)
            for a, c in edges:
                for j in (a, c):
                    mk.points.append(_pt(corners[j]))
            mk.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            ma.markers.append(mk)
            txt = Marker()
            txt.header.stamp, txt.header.frame_id = stamp, 'livox_frame'
            txt.ns, txt.id = 'label', b['cid']
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            top = to_sensor((x0 + x1) / 2, (y0 + y1) / 2, min(z1 + 0.15, 1.6))
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = top
            txt.scale.z = 0.12
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = '#%d n=%d dh=%.2f' % (b['cid'], b['n'], np.hypot(*b['cen']))
            txt.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
            ma.markers.append(txt)
        self.pub_m.publish(ma)

    def print_boxes(self, boxes):
        print('[%5.1fs] 簇数=%d 摄入滞后%.2fs 处理%.0fms' %
              (time.monotonic() - self.t0, len(boxes), self.last_age, self.last_proc_ms),
              flush=True)
        for b in boxes:
            x0, x1 = b['x'].min(), b['x'].max()
            y0, y1 = b['y'].min(), b['y'].max()
            z0, z1 = b['h'].min(), b['h'].max()
            dh = np.hypot(*b['cen'])
            az = np.degrees(np.arctan2(b['cen'][1], b['cen'][0]))
            rad = SECTOR_R if abs(az) < 60.0 else RING_R
            inside = np.any(np.hypot(b['x'], b['y']) < rad)
            print('  簇#%d n=%4d  前[%.2f..%.2f] 左[%.2f..%.2f] 高[%.2f..%.2f]'
                  '  中心 dh=%.2f az=%+.0f° 尺寸 %.2f×%.2f×%.2f 反射率中位%3.0f  %s'
                  % (b['cid'], b['n'], x0, x1, y0, y1, z0, z1, dh, az,
                     x1 - x0, y1 - y0, z1 - z0, np.median(b['r']),
                     '→ 会拦' if inside else '(不拦)'), flush=True)
        print('  （反射率实测：真结构 p50≈64/p90≈108；深色衣服的人仅 9~12，鬼影簇 4~21，'
              '地板≈8——反射率分不开真物/鬼影，不作判据）', flush=True)


def _pt(v):
    from geometry_msgs.msg import Point
    p = Point()
    p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
    return p


def cluster(px, py):
    n = len(px)
    lab = np.zeros(n, np.int32)
    gx = np.floor(px / CELL).astype(np.int64)
    gy = np.floor(py / CELL).astype(np.int64)
    cells = {}
    for i in range(n):
        cells.setdefault((gx[i], gy[i]), []).append(i)
    nid = 0
    for (cx, cy), idxs in cells.items():
        if lab[idxs[0]]:
            continue
        nid += 1
        stack = [(cx, cy)]
        while stack:
            c = stack.pop()
            for j in cells.get(c, ()):
                if lab[j]:
                    continue
                lab[j] = nid
                stack.append((gx[j], gy[j]))
    return lab


def mat2quat(R):
    q = np.empty(4)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = q / np.linalg.norm(q)
    return q[0], q[1], q[2], q[3]


def main():
    rclpy.init()
    rclpy.spin(Viz())


if __name__ == '__main__':
    main()
