// obstacle_veto_node.cpp — 独立避障否决层 rclcpp 移植（2026-11-06 影子先行）
//
// 移植自 obstacle_veto.py（P0-S 最后防线）。决策语义逐条对照 Python 版，
// 回归契约见 test/veto_core_test.cpp（R1-R3 38 断言；R4 仲裁在桥侧）。
// 接线与 Python 版完全一致：
//   /livox/points2(BE10) + /livox/imu(BE10) + /cmd_vel_raw(BE1)
//   → /go2/cmd_vel(BE1) + /veto/status(REL5) + /veto/cmd(REL5)
// 影子模式（不碰真输出，Python 版照常服役）：
//   --ros-args -r /go2/cmd_vel:=/go2/cmd_vel_shadow
//                -r /veto/status:=/veto/status_shadow
//                -r /veto/cmd:=/veto/cmd_shadow -r __node:=obstacle_veto_shadow
//   对拍：同流订阅双方输出，diff 决策（速度差容忍 1e-3，状态字符串须相同）。
// 行为移植要点（对照 Python 逐条核对过的）：
//   * R3 戳闸只认"过闸帧"刷新 last_stamp，拒帧不推进基准；
//   * min_pts 用 m.width 判（与 Python 同），实际行数取 data.size()/16；
//   * STALE 熔断连角速度一起归零；正常路 wz 永远原样透传；
//   * 标定 3 轮 SVD 精修后 hh 取【末轮】的 c/nn（先签后值的顺序语义）；
//   * _set_ground 换几何必清窗口（dh 基准变了，旧数据重灌）；
//   * IMU 增量跟踪先 _apply_ground(R·n) 再提交 g_new；
//   * 姿态自愈：倾斜>5° / 静置>120s / 6 连败后 120s，三路 _rearm。
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "obstacle_veto_cpp/veto_core.hpp"

namespace veto {

using veto_core::Geom;
using veto_core::Params;
using veto_core::SL;
using veto_core::State;

class ObstacleVeto : public rclcpp::Node {
public:
    ObstacleVeto() : Node("obstacle_veto") {
        // ---- 参数（全部可 -p 覆盖，名字与 Python 版一致）----
        prm_.band_lo = p_d("band_lo", 0.20);
        prm_.band_hi = p_d("band_hi", 0.75);
        prm_.self_half_x = p_d("self_half_x", 0.25);
        prm_.self_half_y = p_d("self_half_y", 0.15);
        prm_.self_z_lo = p_d("self_z_lo", -0.30);
        prm_.self_z_hi = p_d("self_z_hi", 0.35);
        // band_lo=0.20：实弹教训——坡度+标定残差让地板壳漏进 h 0.05-0.15
        //（拦截点 51% 是地板，净空处也永远 BLOCK）；代价 <20cm 低矮物不可见
        prm_.sector_half = p_d("sector_half_deg", 60.0) * M_PI / 180.0;
        prm_.sector_r = p_d("sector_r", 1.0);   // R1：真·水平米
        prm_.ring_r = p_d("ring_r", 0.55);
        prm_.touch_r = p_d("touch_r", 0.30);
        prm_.near_keep = p_d("near_keep", 1.25);
        prm_.vx_dead = p_d("vx_dead", 0.02);
        prm_.v_min_walk = p_d("v_min_walk", 0.25);  // 残速地板：低于即归零
        prm_.stamp_check = p_b("stamp_check", true);
        prm_.stamp_max_age = p_d("stamp_max_age", 2.5);

        geom_ = Geom(veto_core::prior_n(), veto_core::PRIOR_H,
                     p_d("body_along", 0.05), p_d("body_lat", 0.0));
        window_s_ = p_d("cloud_window", 0.5);
        timeout_s_ = p_d("cloud_timeout", 1.0);
        min_pts_ = p_i("min_pts", 2000);
        min_spread_ = p_d("min_spread", 1.0);
        calib_after_s_ = p_d("calib_after_s", 6.0);
        imu_quiet_g_ = p_d("imu_quiet_g", 0.05);
        imu_tau_ = p_d("imu_tau", 2.0);
        imu_track_ = p_b("imu_track", true);
        posture_rearm_deg_ = p_d("posture_rearm_deg", 5.0);
        recheck_s_ = p_d("recheck_s", 120.0);
        calib_frames_ = p_i("calib_frames", 55);

        // ---- 状态 ----
        t_first_ = mono_now();
        calib_done_ = false;
        calib_attempts_ = 0;
        next_calib_t_ = t_first_ + calib_after_s_;
        g_ = geom_.n;
        g_at_fit_ = geom_.n;
        last_fit_t_ = t_first_;
        imu_n_ = 0;
        quiet_n_ = 0;
        a_mag_ = 0.0;
        imu_quiet_state_ = ImuQuiet::kNone;
        last_imu_t_ = -1.0;
        imu_sign_ = 0.0;
        boot_aligned_ = false;
        calib_src_ = "prior";
        last_state_ = "INIT";
        last_has_blk_ = false;
        last_blk_dh_ = 0.0;
        last_blk_az_ = 0.0;
        last_fresh_t_ = -1.0;
        has_last_stamp_ = false;
        last_stamp_ = 0.0;
        last_stamp_warn_ = 0.0;
        st_frames_ = st_fresh_ = st_pass_ = st_sector_ = st_ring_ = st_stale_ =
            st_stamprej_ = 0;

        // ---- 话题（QoS 与 Python 版逐一对应）----
        rclcpp::QoS qb(rclcpp::KeepLast(10));
        qb.best_effort();
        rclcpp::QoS qcmd(rclcpp::KeepLast(1));  // 最新指令赢，过期不重放
        qcmd.best_effort();
        rclcpp::QoS qr(rclcpp::KeepLast(5));
        qr.reliable();
        rclcpp::QoS qout(rclcpp::KeepLast(1));  // BE：桥看门狗兜底
        qout.best_effort();

        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points2", qb,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { on_cloud(m); });
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", qb,
            [this](sensor_msgs::msg::Imu::SharedPtr m) { on_imu(m); });
        sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", qcmd,
            [this](geometry_msgs::msg::Twist::SharedPtr m) { on_cmd(m); });
        sub_vcmd_ = create_subscription<std_msgs::msg::String>(
            "/veto/cmd", qr, [this](std_msgs::msg::String::SharedPtr m) {
                on_cmd_string(m);
            });
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
            "/go2/cmd_vel", qout);
        pub_status_ = create_publisher<std_msgs::msg::String>("/veto/status", 5);

        status_timer_ = create_wall_timer(std::chrono::duration<double>(1.0),
                                          [this]() { publish_status(); });

        RCLCPP_INFO(get_logger(),
                    "obstacle_veto(C++) 就绪(G1): n=[%.3f,%.3f,%.3f] h=%.3f, "
                    "前扇±%.0f°/%.2fm(真距) 环%.2fm, 超时%.1fs, 戳闸%s(%.1fs), "
                    "残速地板%.2f(低于即归零,不抬升)",
                    geom_.n.x(), geom_.n.y(), geom_.n.z(), geom_.h,
                    prm_.sector_half * 180.0 / M_PI, prm_.sector_r,
                    prm_.ring_r, timeout_s_,
                    prm_.stamp_check ? "开" : "关", prm_.stamp_max_age,
                    prm_.v_min_walk);
    }

private:
    // ---- 时钟：monotonic 对齐 Python time.monotonic；wall 对齐 time.time ----
    static double mono_now() {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    static double wall_now() {
        return std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    // Foxy 的 declare_parameter(name, value) 直接返回 T（Humble 才是 ParameterValue）
    double p_d(const char* n, double d) {
        return declare_parameter(n, d);
    }
    int p_i(const char* n, int d) {
        return declare_parameter(n, d);
    }
    bool p_b(const char* n, bool d) {
        return declare_parameter(n, d);
    }

    // ---------------- 回调 ----------------
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr& m) {
        ++st_frames_;
        // R3 采集时效闸：刚收到≠刚采集（旧帧延迟到达/重复帧无限续命是评审
        // 实锤盲区）。拒帧不推进 last_stamp 基准。
        const double stamp_s =
            static_cast<double>(m->header.stamp.sec) +
            static_cast<double>(m->header.stamp.nanosec) * 1e-9;
        const auto sr =
            veto_core::stamp_ok(stamp_s, has_last_stamp_, last_stamp_,
                                wall_now(), prm_);
        if (!sr.ok) {
            ++st_stamprej_;
            const double now = mono_now();
            if (now - last_stamp_warn_ > 10.0) {
                last_stamp_warn_ = now;
                RCLCPP_WARN(get_logger(),
                            "[R3] 拒帧(%s): 戳龄 %.2fs, 连拒 %d — 驱动冻结/"
                            "时钟域异常?",
                            sr.reason.c_str(), wall_now() - stamp_s,
                            st_stamprej_);
            }
            return;
        }
        last_stamp_ = stamp_s;
        has_last_stamp_ = true;
        if (static_cast<int>(m->width) < min_pts_ || m->point_step != 16)
            return;  // 空云/病云/异格式不计新鲜
        const Eigen::Index rows =
            static_cast<Eigen::Index>(m->data.size()) / 16;
        if (rows <= 0) return;
        // D2 精神：数据 Blob 直接 Map 成 (N,4) float，一拷贝进 Eigen，绝不逐点
        using Map4 = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 4,
                                                   Eigen::RowMajor>>;
        const Map4 arr(reinterpret_cast<const float*>(m->data.data()),
                       rows, 4);
        Eigen::MatrixX3d P(rows, 3);
        P = arr.leftCols<3>().cast<double>();
        // 塌缩成团的病云不计新鲜（散布双轴之和）
        const double spread =
            (P.col(0).maxCoeff() - P.col(0).minCoeff()) +
            (P.col(1).maxCoeff() - P.col(1).minCoeff());
        if (spread < min_spread_) return;
        ++st_fresh_;
        const double now = mono_now();
        last_fresh_t_ = now;
        auto sl = veto_core::ingest(P, geom_, prm_);
        if (!sl.empty()) win_.emplace_back(now, std::move(sl));
        // 建议保留窗口
        while (!win_.empty() && win_.front().first < now - window_s_)
            win_.pop_front();
        // 标定采集：IMU 在手时只在"准静止"采帧；断流/缺席回退纯雷达时序。
        // 收满即精修；失败 15s 后重采，至多 6 次。
        const bool imu_fresh = last_imu_t_ >= 0.0 && now - last_imu_t_ < 3.0;
        const bool quiet_ok = imu_quiet_state_ == ImuQuiet::kNone ||
                              !imu_fresh ||
                              imu_quiet_state_ == ImuQuiet::kQuiet;
        if (!calib_done_ && calib_attempts_ < 6 && now >= next_calib_t_ &&
            quiet_ok) {
            raw_.push_back(P);
            if (raw_.size() > 80) raw_.pop_front();
            if (static_cast<int>(raw_.size()) >= calib_frames_) {
                if (calibrate("auto")) {
                    calib_done_ = true;
                } else {
                    ++calib_attempts_;
                    next_calib_t_ = now + 15.0;
                    raw_.clear();
                }
            }
        }
    }

    void on_cmd(const geometry_msgs::msg::Twist::SharedPtr& m) {
        double vx = m->linear.x, vy = m->linear.y, wz = m->angular.z;
        const double now = mono_now();
        std::string state;
        if (last_fresh_t_ < 0.0 || now - last_fresh_t_ > timeout_s_) {
            state = "STALE";
            vx = vy = wz = 0.0;  // 熔断：无有效云=禁走（设计4，撞车教训①）
            ++st_stale_;
            last_has_blk_ = false;
        } else {
            while (!win_.empty() && win_.front().first < now - window_s_)
                win_.pop_front();
            std::vector<SL> pts;
            size_t total = 0;
            for (const auto& w : win_) total += w.second.size();
            pts.reserve(total);
            for (const auto& w : win_)
                pts.insert(pts.end(), w.second.begin(), w.second.end());
            const auto r = veto_core::apply_veto(vx, vy, pts, prm_);
            vx = r.vx;
            vy = r.vy;
            if (r.state == State::PASS) {
                state = "PASS";
                ++st_pass_;
            } else if (r.state == State::BLOCK_RING) {
                state = "BLOCK_RING";
                ++st_ring_;
            } else {
                state = "BLOCK_SECTOR";
                ++st_sector_;
            }
            last_has_blk_ = r.has_blk;
            last_blk_dh_ = r.blk_dh;
            last_blk_az_ = r.blk_az_deg;
        }
        last_state_ = state;
        geometry_msgs::msg::Twist out;
        out.linear.x = vx;
        out.linear.y = vy;
        out.angular.z = wz;
        pub_cmd_->publish(out);
    }

    void on_cmd_string(const std_msgs::msg::String::SharedPtr& m) {
        if (m->data == "recalib") {
            raw_.clear();
            calib_done_ = false;  // 否则自动标定永不触发
            calib_attempts_ = 0;
            next_calib_t_ = mono_now();
            RCLCPP_INFO(get_logger(), "重标定：采 %d 帧静止帧后精修...",
                        calib_frames_);
        } else if (m->data == "prior") {
            set_ground(veto_core::prior_n(), veto_core::PRIOR_H, "prior");
            RCLCPP_INFO(get_logger(), "已回退先验");
        }
    }

    // ---------------- IMU 重力对齐 ----------------
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr& m) {
        const Eigen::Vector3d a(m->linear_acceleration.x,
                                m->linear_acceleration.y,
                                m->linear_acceleration.z);
        ++imu_n_;
        const double now = mono_now();
        const double dt = last_imu_t_ < 0.0
                               ? 0.005
                               : std::min(0.5, now - last_imu_t_);
        last_imu_t_ = now;
        const double am = a.norm();
        a_mag_ = am;
        // Mid360 内置 IMU 输出 g 单位（静止 |a|≈0.997）：贴 1g 判准静止
        imu_quiet_state_ = std::fabs(am - 1.0) < imu_quiet_g_
                               ? ImuQuiet::kQuiet
                               : ImuQuiet::kMoving;
        if (imu_quiet_state_ != ImuQuiet::kQuiet) return;
        if (imu_sign_ == 0.0)  // 符号约定（实测 +1：读数即向上）
            imu_sign_ = a.dot(geom_.n) > 0.0 ? 1.0 : -1.0;
        const Eigen::Vector3d up = imu_sign_ * a / std::max(am, 1e-6);
        ++quiet_n_;
        // EMA：对齐前快收敛（50帧≈0.3s 残差<1%），对齐后慢跟踪
        const double tau = boot_aligned_ ? imu_tau_ : 0.05;
        const double alpha = dt / (tau + dt);
        Eigen::Vector3d g_new = g_ + alpha * (up - g_);
        g_new.normalize();
        // 锚后增量跟踪：地板锚定的法线随重力增量旋转（坡道/步态俯仰实时补偿）
        if (imu_track_ && calib_src_ != "prior") {
            Eigen::Matrix3d R;
            if (veto_core::rot_between(g_, g_new, R)) geom_.rebase(R * geom_.n);
        }
        g_ = g_new;
        // 开机静止窗重力对齐（启动=静止假设）：趴着也给得出正确法线
        if (!boot_aligned_ && quiet_n_ >= 50 && now - t_first_ > 2.0) {
            boot_aligned_ = true;
            const double dev = 180.0 / M_PI *
                               std::acos(std::min(1.0, std::fabs(
                                       g_.dot(veto_core::prior_n()))));
            if (dev < 15.0) {
                set_ground(g_, veto_core::PRIOR_H, "imu-boot");
                RCLCPP_INFO(get_logger(),
                            "[imu-boot] 重力对齐: 与先验夹角 %.1f°，法线即刻"
                            "可用，h 暂用先验 %.3f",
                            dev, veto_core::PRIOR_H);
            } else {
                RCLCPP_WARN(get_logger(),
                            "[imu-boot] 重力与先验差 %.1f°>15°（坡道/被搬动?），"
                            "保持先验等地板精修",
                            dev);
            }
            return;
        }
        // 姿态自愈：有锚才谈得上"变了"。倾斜累计超阈（坐/趴/搬动）、静置超
        // 周期、6 连败后周期重试——三路都 _rearm（狗趴一阵再站起能自愈）
        const double tilt = 180.0 / M_PI *
                            std::acos(std::min(1.0, std::fabs(g_.dot(g_at_fit_))));
        const double since_fit = now - last_fit_t_;
        if (calib_src_ != "prior") {
            if (calib_done_ && tilt > posture_rearm_deg_) {
                rearm(now, "IMU 倾斜累计 " + std::to_string(tilt) + "°>" +
                               std::to_string(posture_rearm_deg_) +
                               "°（坐/趴/搬动）");
            } else if (calib_done_ && since_fit > recheck_s_) {
                rearm(now, "静置复检（上次锚定 " + std::to_string(since_fit) +
                               "s 前）");
            } else if (calib_attempts_ >= 6 && since_fit > recheck_s_) {
                rearm(now, "精修 6 连败后定期重试（等狗回站立域）");
            }
        }
    }

    void rearm(double now, const std::string& why) {
        calib_done_ = false;
        calib_attempts_ = 0;
        raw_.clear();
        next_calib_t_ = now + 1.0;
        last_fit_t_ = now;
        RCLCPP_WARN(get_logger(), "%s：待静止后自动重标定", why.c_str());
    }

    // ---------------- 标定 ----------------
    bool calibrate(const std::string& tag) {
        if (raw_.size() < 5) {
            RCLCPP_WARN(get_logger(), "[%s] 标定帧不足(%zu)，保持先验",
                        tag.c_str(), raw_.size());
            return false;
        }
        Eigen::Index total = 0;
        for (const auto& r : raw_) total += r.rows();
        Eigen::MatrixX3d P(total, 3);
        Eigen::Index off = 0;
        for (const auto& r : raw_) {
            P.middleRows(off, r.rows()) = r;
            off += r.rows();
        }
        const Eigen::Vector3d prior = geom_.n;
        Eigen::VectorXd h = P * prior;
        // 5 分位做地板候选种子（numpy percentile 线性插值同式）
        const double p5 = veto_core::percentile5(
            std::vector<double>(h.data(), h.data() + h.size()));
        auto band_sel = [&P](const Eigen::VectorXd& d)
            -> std::pair<Eigen::MatrixX3d, int> {
            std::vector<Eigen::Index> idx;
            idx.reserve(d.size());
            for (Eigen::Index i = 0; i < d.size(); ++i)
                if (std::fabs(d[i]) < 0.08) idx.push_back(i);
            Eigen::MatrixX3d Q(static_cast<Eigen::Index>(idx.size()), 3);
            for (size_t k = 0; k < idx.size(); ++k)
                Q.row(static_cast<Eigen::Index>(k)) = P.row(idx[k]);
            return {Q, static_cast<int>(idx.size())};
        };
        auto [Q, nsel] = band_sel(Eigen::VectorXd(h.array() - p5));
        if (nsel < 500) {
            RCLCPP_WARN(get_logger(), "[%s] 地板候选点不足，保持先验",
                        tag.c_str());
            return false;
        }
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        Eigen::Vector3d nn = prior;
        for (int it = 0; it < 3; ++it) {
            c = Q.colwise().mean().transpose();
            Eigen::JacobiSVD<Eigen::MatrixX3d> svd(
                Q.rowwise() - c.transpose(), Eigen::ComputeThinV);
            nn = svd.matrixV().col(2);  // 最小奇异值的右奇异向量=法线
            if (nn.dot(prior) < 0.0) nn = -nn;
            const auto sel = band_sel(
                Eigen::VectorXd((P * nn).array() - c.dot(nn)));
            if (sel.second < 500) {
                RCLCPP_WARN(get_logger(), "[%s] 精修发散，保持先验",
                            tag.c_str());
                return false;
            }
            Q = sel.first;
        }
        const double hh = -c.dot(nn);
        const double dev = 180.0 / M_PI *
                            std::acos(std::min(1.0, std::fabs(nn.dot(prior))));
        // 校验门收紧到"站立姿态"合理域：坐/趴下传感器掉 ~0.18m，误锁会把
        // 0.38m 以下障碍全弄瞎（2026-11-03 实弹 h=0.309 教训）
        if (dev < 12.0 && 0.35 <= hh && hh <= 0.68) {
            set_ground(nn, hh, "refit-" + tag);
            RCLCPP_INFO(get_logger(),
                        "[%s] 地板精修: dev=%.1f° h=%.3fm（先验 %.3f）",
                        tag.c_str(), dev, hh, veto_core::PRIOR_H);
            return true;
        }
        RCLCPP_WARN(get_logger(),
                    "[%s] 精修未过校验(dev=%.1f° h=%.2f)，保持先验，15s 后重采"
                    "重试",
                    tag.c_str(), dev, hh);
        return false;
    }

    void set_ground(const Eigen::Vector3d& nn, double hh,
                    const std::string& src) {
        geom_.rebase(nn);
        geom_.h = hh;
        calib_src_ = src;
        g_at_fit_ = g_;  // 锚定当前重力：姿态变化检测的基准
        last_fit_t_ = mono_now();
        win_.clear();  // 几何变了，窗口数据重灌
    }

    // ---------------- 状态 ----------------
    void publish_status() {
        const double now = mono_now();
        const long age_ms = last_fresh_t_ < 0.0
                                ? -1
                                : static_cast<long>((now - last_fresh_t_) *
                                                    1000.0);
        std::string imu = "none", q = "-";
        double slope = -1.0;
        if (imu_n_ > 0) {
            if (last_imu_t_ >= 0.0 && now - last_imu_t_ < 3.0) {
                imu = "ok";
                q = imu_quiet_state_ == ImuQuiet::kQuiet ? "1" : "0";
                slope = 180.0 / M_PI *
                        std::acos(std::min(1.0, std::fabs(g_.dot(geom_.n))));
            } else {
                imu = "dead";
            }
        }
        char blk[48];
        if (last_has_blk_)
            std::snprintf(blk, sizeof(blk), "dh=%.2f az=%.0f", last_blk_dh_,
                          last_blk_az_);
        else
            std::snprintf(blk, sizeof(blk), "none");
        char buf[320];
        std::snprintf(buf, sizeof(buf),
                      "state=%s age_ms=%ld win=%zu blk=%s calib=%s "
                      "frames=%d fresh=%d pass=%d sector=%d ring=%d stale=%d "
                      "stamprej=%d imu=%s a=%.2f q=%s slope=%.1f try=%d/6",
                      last_state_.c_str(), age_ms, win_.size(), blk,
                      calib_src_.c_str(), st_frames_, st_fresh_, st_pass_,
                      st_sector_, st_ring_, st_stale_, st_stamprej_,
                      imu.c_str(), a_mag_, q.c_str(), slope, calib_attempts_);
        std_msgs::msg::String msg;
        msg.data = buf;
        pub_status_->publish(msg);
    }

    // ---------------- 成员 ----------------
    enum class ImuQuiet { kNone, kQuiet, kMoving };

    Params prm_{};
    Geom geom_;
    double window_s_, timeout_s_;
    int min_pts_, calib_frames_;
    double min_spread_, calib_after_s_;
    double imu_quiet_g_, imu_tau_;
    bool imu_track_;
    double posture_rearm_deg_, recheck_s_;

    std::deque<std::pair<double, std::vector<SL>>> win_;
    std::deque<Eigen::MatrixX3d> raw_;
    double last_fresh_t_;
    bool has_last_stamp_;
    double last_stamp_, last_stamp_warn_;
    double t_first_, next_calib_t_, last_fit_t_;
    bool calib_done_;
    int calib_attempts_;
    Eigen::Vector3d g_, g_at_fit_;
    int imu_n_, quiet_n_;
    ImuQuiet imu_quiet_state_;
    double a_mag_, last_imu_t_, imu_sign_;
    bool boot_aligned_;
    std::string calib_src_, last_state_;
    bool last_has_blk_;
    double last_blk_dh_, last_blk_az_;
    int st_frames_, st_fresh_, st_pass_, st_sector_, st_ring_, st_stale_,
        st_stamprej_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_vcmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace veto

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<veto::ObstacleVeto>());
    rclcpp::shutdown();
    return 0;
}
