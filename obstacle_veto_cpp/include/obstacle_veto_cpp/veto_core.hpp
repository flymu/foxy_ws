// veto_core.hpp — obstacle_veto 纯决策核心的 C++ 移植（2026-11-06）
//
// 移植自 veto_core.py（G1 2026-11-05，无 ROS 纯核心）。本文件只负责对错，
// 节点接线在 obstacle_veto_node.cpp。语义逐条对照 Python 版，移植纪律：
//   R1 距离度量：dh 在重力正交水平基下度量（旧式 hypot(vh.x,vh.y) 在 47°
//      斜装系下前向压缩 ×0.68——实测 0.62 vs 真值 0.91 胶带复核）。半径
//      语义随之校正：sector_r=1.0 就是真 1.0m。
//   R2 只抑制不放大：消解过的残速低于起步阈值→归零（不抬升）；未消解
//      的指令原样放行；输出速度模长 ≤ 输入。
//   R3 采集时效双闸：采集年龄超限（'age'）或时间戳不前进（'frozen'）即拒；
//      回放模式 stamp_check=false 显式关门。
// 回归对照：test/veto_core_test.cpp（与 test_veto_regression.py 的 R1-R3
// 断言同数值逐条对拍；R4 仲裁器在 bridge 侧 Python，不在此移植范围）。
#pragma once
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace veto_core {

// ---- 先验（veto_calib_probe.py 从 campus_map 静止段标定）----
inline Eigen::Vector3d prior_n() { return {-0.7330, 0.0001, 0.6802}; }
inline constexpr double PRIOR_H = 0.486;

// ---------------- 安装几何 ----------------
struct Geom {
    Eigen::Vector3d n{Eigen::Vector3d::Zero()};  // 重力"向上"（传感器系）
    Eigen::Vector3d f{Eigen::Vector3d::Zero()};  // 狗前向
    Eigen::Vector3d l{Eigen::Vector3d::Zero()};  // 狗左向
    Eigen::Vector3d body_c{Eigen::Vector3d::Zero()};  // 体轴心（传感器系）
    double h = 0.0;                              // 传感器离地高
    bool has_body_c_ = false;

    // 平装默认（节点在构造体里读完参数后立刻重赋值；默认仅供成员声明）
    Geom() : Geom(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0, 0.0, 0.0) {}

    Geom(const Eigen::Vector3d& n_in, double h_in,
         double body_along = 0.05, double body_lat = 0.0) {
        rebase(n_in);  // 首次：body_c 尚不存在（与 Python hasattr 分支一致）
        h = h_in;
        body_c = body_along * f + body_lat * l;
        has_body_c_ = true;
    }

    // 法线更新后重导 f/l/体心（IMU 增量跟踪用；h 不动）。
    // 体轴心的 along/lat 分量必须先在【旧基】下分解——先换基再分解会让
    // 体心冻结在旧向量上不随地面估计转（Python 回归抓过的顺序 bug）。
    void rebase(const Eigen::Vector3d& n_in) {
        double ba = 0.0, bl = 0.0;
        if (has_body_c_) {
            ba = body_c.dot(f);
            bl = body_c.dot(l);
        }
        n = n_in.normalized();
        Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
        Eigen::Vector3d fp = x_axis - x_axis.dot(n) * n;
        f = fp.normalized();
        l = n.cross(f);
        if (has_body_c_) body_c = ba * f + bl * l;
    }

    // 狗身坐标 → 传感器系（与 veto_viz 同式）
    Eigen::Vector3d to_sensor(double x, double y, double z) const {
        return x * f + y * l + (z - h) * n;
    }
};

// ---------------- 决策参数 ----------------
struct Params {
    double band_lo = 0.20;         // 离地检查频带下沿 (m)
    double band_hi = 0.75;         // 上沿
    double self_half_x = 0.25;    // 自体掩膜半宽 (x)
    double self_half_y = 0.15;     // (y)
    double self_z_lo = -0.30;     // 自体掩膜 z 范围（传感器系）
    double self_z_hi = 0.35;
    double sector_half = 60.0 * M_PI / 180.0;  // 前扇区半角
    double sector_r = 1.0;         // 前扇区拦截半径（真·水平米，R1 语义）
    double ring_r = 0.55;         // 侧后环拦截半径
    double touch_r = 0.30;        // 极近全消平动
    double near_keep = 1.25;      // 入库水平距上限
    double vx_dead = 0.02;        // 分量消解死区
    double v_min_walk = 0.25;     // 残速低于此→归零（不抬升）
    bool stamp_check = true;      // R3 时效闸
    double stamp_max_age = 2.5;   // 采集年龄上限(s)
};

// ---------------- ingest ----------------
// 一帧点云（传感器系 N×3）→ 频带内、近场、非自体的 (dh, az)。
// dh=正交水平基下真距离；az=方位角(rad，0=正前，+左)。
struct SL { double dh = 0.0, az = 0.0; };

inline std::vector<SL> ingest(const Eigen::MatrixX3d& P, const Geom& g,
                             const Params& prm) {
    std::vector<SL> out;
    const Eigen::Index n = P.rows();
    if (n == 0) return out;
    out.reserve(static_cast<size_t>(n));
    for (Eigen::Index i = 0; i < n; ++i) {
        const Eigen::Vector3d p = P.row(i);
        const double hgt = p.dot(g.n) + g.h;
        if (!(hgt > prm.band_lo && hgt < prm.band_hi)) continue;   // 频带
        if (std::fabs(p.x()) < prm.self_half_x &&                  // 自体掩膜
            std::fabs(p.y()) < prm.self_half_y &&
            p.z() > prm.self_z_lo && p.z() < prm.self_z_hi)
            continue;
        const Eigen::Vector3d rel = p - g.body_c;
        const Eigen::Vector3d vh = rel - rel.dot(g.n) * g.n;        // 水平面内
        // R1 修正核心：正交水平基下取距离（倾斜系 hypot(x,y) 会丢 z 压缩前向）
        const double dh = std::hypot(vh.dot(g.f), vh.dot(g.l));
        if (!(dh < prm.near_keep)) continue;
        out.push_back({dh, std::atan2(vh.dot(g.l), vh.dot(g.f))});
    }
    return out;
}

// 方位相关拦截半径：前扇区远、侧后环近（|az| 严格小于才进扇区）
inline double radius(double az, const Params& prm) {
    return std::fabs(az) < prm.sector_half ? prm.sector_r : prm.ring_r;
}

// ---------------- apply_veto ----------------
enum class State { PASS, BLOCK_SECTOR, BLOCK_RING };

struct VetoResult {
    double vx = 0.0, vy = 0.0;
    State state = State::PASS;
    bool has_blk = false;
    double blk_dh = 0.0;      // 最近拦截者水平距
    double blk_az_deg = 0.0;  // 方位角（度）
};

// 对速度指令执行否决。pts = 窗口内全部 (dh, az)。
// 语义（G1 定案）：只抑制不放大；消解过的残速低于起步阈值→归零；
// 未消解的指令原样放行；角速度不在此管辖区。
inline VetoResult apply_veto(double vx_in, double vy_in,
                             const std::vector<SL>& pts, const Params& prm) {
    VetoResult r;
    r.vx = vx_in;
    r.vy = vy_in;
    if (pts.empty()) return r;  // 空场放行

    // 拦截者：dh < radius(az)，取最近 3 个
    std::vector<const SL*> blockers;
    blockers.reserve(pts.size());
    for (const auto& s : pts)
        if (s.dh < radius(s.az, prm)) blockers.push_back(&s);
    if (blockers.empty()) return r;

    std::sort(blockers.begin(), blockers.end(),
              [](const SL* a, const SL* b) { return a->dh < b->dh; });
    if (blockers.size() > 3) blockers.resize(3);

    r.state = std::fabs(blockers[0]->az) > prm.sector_half
                  ? State::BLOCK_RING : State::BLOCK_SECTOR;

    if (blockers[0]->dh < prm.touch_r) {
        r.vx = r.vy = 0.0;  // 极近：全消平动
    } else {
        bool cancelled = false;
        for (const SL* b : blockers) {
            Eigen::Vector2d u(std::cos(b->az), std::sin(b->az));
            Eigen::Vector2d v(r.vx, r.vy);
            const double toward = v.dot(u);
            if (toward > prm.vx_dead) {
                v -= toward * u;
                r.vx = v.x();
                r.vy = v.y();
                cancelled = true;
            }
        }
        if (cancelled) {
            // R2：残速低于起步阈值 → 归零（宁停不倒车，不抬升）
            if (std::hypot(r.vx, r.vy) < prm.v_min_walk)
                r.vx = r.vy = 0.0;
        }
    }
    r.has_blk = true;
    r.blk_dh = blockers[0]->dh;
    r.blk_az_deg = blockers[0]->az * 180.0 / M_PI;
    return r;
}

// ---------------- R3 时效闸 ----------------
struct StampResult { bool ok = false; std::string reason; };

inline StampResult stamp_ok(double stamp, bool has_last, double last_stamp,
                            double now, const Params& prm) {
    if (!prm.stamp_check) return {true, "off"};
    if (has_last && stamp <= last_stamp) return {false, "frozen"};
    if (now - stamp > prm.stamp_max_age) return {false, "age"};
    return {true, "ok"};
}

// numpy percentile（线性插值，default_rng 同式）——地板拟合取 h 的 5 分位
inline double percentile5(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::vector<double> s(v);
    std::sort(s.begin(), s.end());
    const double idx = 0.05 * static_cast<double>(s.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return s[lo] * (1.0 - frac) + s[hi] * frac;
}

// Rodrigues：把单位向量 u 转到 v 的最小旋转。反平行（s<1e-9）返回 false。
inline bool rot_between(const Eigen::Vector3d& u, const Eigen::Vector3d& v,
                        Eigen::Matrix3d& R) {
    const double c = std::min(1.0, std::max(-1.0, u.dot(v)));
    const Eigen::Vector3d w = u.cross(v);
    const double s = w.norm();
    if (s < 1e-9) return false;
    const Eigen::Vector3d k = w / s;
    Eigen::Matrix3d K;
    K << 0.0, -k.z(), k.y(),
         k.z(), 0.0, -k.x(),
         -k.y(), k.x(), 0.0;
    const double th = std::atan2(s, c);
    R = Eigen::Matrix3d::Identity() + std::sin(th) * K +
        (1.0 - std::cos(th)) * (K * K);
    return true;
}

}  // namespace veto_core
