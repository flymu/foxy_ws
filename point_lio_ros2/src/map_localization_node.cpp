// map_localization_node.cpp — 在已建地图上定位：Point-LIO 里程计 + NDT 地图匹配
//                        + 狗四足航位推算看门狗（LIO 病态时接管位姿输出）
// 订阅: /Odometry (Point-LIO, camera_init 系), /cloud_registered (camera_init 系点),
//       /go2/odom (狗固件四足解算航位推算, Orin go2_odom_bridge 转发, dog_odom 系)
// 发布: /loc/pose (nav_msgs/Odometry, map 系), /loc/health (std_msgs/String),
//       TF: map->body, map->camera_init
// 原理: /cloud_registered 已在 odom(camera_init) 系 → 累积 ~0.8s 子图 S_od；
//       NDT 解 T_map_odom 使 M_map ≈ T_mo * S_od；位姿 = T_mo * T_odom_body。
//       已知起点初始化（参数 init_x/y/yaw 覆盖里程计起点的平移+偏航，roll/pitch/z 沿用
//       里程计首帧——dog 每次从同一位置出发，巡检场景够用；全局绑架重定位留待后续）。
//
// 狗四足航位推算看门狗（2026-10-23 加入，治"雷达驱动发病喂脏数据 → LIO 飞走"）:
//   狗固件运动学+IMU 航位推算不依赖雷达。滚动比较 LIO 与狗各自最近 dog_window 秒
//   的位移/偏航差分（各用自身时间戳内部差分，跨设备时钟偏差不敏感）：
//     ||ΔLIO|-|Δdog|| > dog_bad_abs + dog_bad_rel_k*|Δdog|，或
//     LIO 走了狗没走，或 LIO 转了狗没转 → 判 LIO 病态：
//   /loc/pose 切换为「最后一致锚点 + 狗航位推算增量」死推（狗流速率驱动）；
//   停子图累积（病态点不进 NDT）；狗流断供则冻结位姿保活。
//   连续 dog_unlatch_checks 次窗口一致 → 解除，并按当前死推位姿反推 T_mo 无跳变
//   交还 LIO+NDT。无 /go2/odom 数据时看门狗整体停用，行为与旧版一致。
//   线程约定：回调均在默认单线程执行器，狗/LIO 缓冲只需执行器内自洽；
//   T_mo_ 仍由 t_mo_mutex_ 保护（NDT 后台线程写）。
//
// R5/G4 修复批（2026-11-05，评审 R5 + P0-1/2/3）：
//   * 冻结回跳修复：死推期狗流断供的保活位姿=最后发布的死推位姿（旧版回到
//     初始锚点，狗在死推期间移动时会跳回，评审 R5 实锤）。
//   * 解除迟滞（P0-3）：解除需连续 dog_unlatch_checks 窗一致且跨度 ≥
//     dog_unlatch_span 秒（旧版 10 窗≈1s，latch/unlatch 震荡）；狗流断供 >0.5s
//     恢复时清空差分缓冲重建基准（治 10-31 断供后相邻窗口错判）。
//   * 三态健康（P0-2）：/loc/health 改 1Hz 墙钟心跳（旧版只在 on_odom 里发，
//     odom 停了健康也停），格式 "STATE|reason|age_s"：
//       INITIALIZING = 未初始化/续接判决未决；OK = LIO+NDT 可用；
//       STALE = odom 流断供（位姿冻结）；DEGRADED = 看门狗死推中（狗流驱动）;
//       LOST = 死推中且狗流也断供（位姿冻结，不可用）。
//     OK 且 use_dog_odom 但从未收到狗流 → reason=no_dog_watchdog（可见性）。
//   * 持久化续接（P0-1）：每 5s 落盘 (T_mo, 狗位姿)；下次启动狗位姿与落盘值
//     平移 <0.5m 且偏航 <0.3rad（狗里程计系未重置）→ 续接 T_mo，跳过 known-start。
//     狗动过/换系/超龄 → 回退 known-start。
//   * /initialpose 重定位注入（BBS/人工兜底）：PoseWithCovarianceStamped 按
//     T_mb(map←body) 语义生效——x/y/yaw 取消息值，z/roll/pitch 沿用当前，
//     T_mo = T_mb·T_ob⁻¹，子图在 odom 系无需清，NDT 以新 T_mo 为猜测自然精修。
//     看门狗锁存中收到 → 仅记录不生效（死推位姿不受影响，解除时按死推接管）。
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>

using namespace std::chrono_literals;

class MapLocalizationNode : public rclcpp::Node
{
public:
    MapLocalizationNode() : Node("map_localization")
    {
        // ---- 参数 ----
        map_file_ = declare_parameter<std::string>("map_file", "");
        init_x_ = declare_parameter<double>("init_x", 0.0);
        init_y_ = declare_parameter<double>("init_y", 0.0);
        init_yaw_deg_ = declare_parameter<double>("init_yaw", 0.0);
        submap_frames_ = declare_parameter<int>("submap_frames", 8);
        scan_vox_ = declare_parameter<double>("scan_vox", 0.1);
        map_vox_ = declare_parameter<double>("map_vox", 0.2);
        ndt_res_ = declare_parameter<double>("ndt_res", 1.0);
        ndt_step_ = declare_parameter<double>("ndt_step", 0.1);
        ndt_iter_ = declare_parameter<int>("ndt_iter", 30);
        fitness_limit_ = declare_parameter<double>("fitness_limit", 0.35); // 超过视为失配
        jump_limit_ = declare_parameter<double>("jump_limit", 0.5);        // 单次修正跳变上限(m)
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
        body_frame_ = declare_parameter<std::string>("body_frame", "body");
        use_dog_odom_ = declare_parameter<bool>("use_dog_odom", true);
        dog_window_ = declare_parameter<double>("dog_window", 2.0);          // 一致性窗口(s)
        dog_bad_abs_ = declare_parameter<double>("dog_bad_abs", 0.5);        // 位移分歧绝对阈值(m)
        dog_bad_rel_k_ = declare_parameter<double>("dog_bad_rel_k", 0.25);   // 随狗位移加宽
        dog_bad_yaw_ = declare_parameter<double>("dog_bad_yaw", 0.8);         // 偏航分歧阈值(rad)
        // 狗流复位检测(m/s)：真实复位=瞬移量级(米/0.1s)；Go2 正常极速≈2m/s，
        // 4.0=2 倍余量。旧值 1.5 会在快走/推车段误报（实测袋回放 1.53m/s 帧对）。
        dog_jump_speed_ = declare_parameter<double>("dog_jump_speed", 4.0);
        dog_unlatch_checks_ = declare_parameter<int>("dog_unlatch_checks", 30); // P0-3: 10→30 窗
        dog_unlatch_span_ = declare_parameter<double>("dog_unlatch_span", 3.0); // P0-3: 一致窗还需 ≥3s
        persist_path_ = declare_parameter<std::string>("persist_path", "/home/mu/maps/loc_state.txt");
        persist_max_age_h_ = declare_parameter<double>("persist_max_age_h", 12.0);
        resume_dog_tol_ = declare_parameter<double>("resume_dog_tol", 0.5);   // 续接位移容差(m)
        resume_dog_yaw_ = declare_parameter<double>("resume_dog_yaw", 0.3);   // 续接偏航容差(rad)
        resume_grace_ = declare_parameter<double>("resume_grace", 1.5);       // 续接判决宽限(s)

        if (map_file_.empty())
        {
            RCLCPP_FATAL(get_logger(), "必须给 map_file 参数（全局地图 PCD）");
            throw std::runtime_error("map_file empty");
        }

        // ---- 全局地图载入 + 降采样 ----
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile(map_file_, *raw) < 0 || raw->empty())
        {
            RCLCPP_FATAL(get_logger(), "读地图失败: %s", map_file_.c_str());
            throw std::runtime_error("load map failed");
        }
        map_ds_ = vox(raw, map_vox_);
        kdtree_.setInputCloud(map_ds_);
        RCLCPP_INFO(get_logger(), "地图: %zu 点 -> 降采样 %zu 点 (vox %.2f)",
                    raw->size(), map_ds_->size(), map_vox_);

        // ---- NDT ----
        ndt_.setResolution(ndt_res_);
        ndt_.setStepSize(ndt_step_);
        ndt_.setMaximumIterations(ndt_iter_);
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setInputTarget(map_ds_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(std::move(m)); });
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(std::move(m)); });
        if (use_dog_odom_)
        {
            rclcpp::QoS dq(rclcpp::KeepLast(10));
            dq.best_effort();
            sub_dog_ = create_subscription<nav_msgs::msg::Odometry>(
                "/go2/odom", dq,
                [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_dog(std::move(m)); });
        }
        pub_health_ = create_publisher<std_msgs::msg::String>("/loc/health", 10);
        sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
                on_initialpose(std::move(m));
            });
        pub_loc_ = create_publisher<nav_msgs::msg::Odometry>("/loc/pose", 10);
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        start_wall_ = now();
        last_odom_wall_ = start_wall_;
        health_timer_ = create_wall_timer(1s, [this]() { on_health_timer(); }); // R5: 健康独立心跳
        persist_timer_ = create_wall_timer(5s, [this]() { save_persist(); });  // P0-1: 持久化
        load_persist();                                                          // P0-1: 续接判决
        RCLCPP_INFO(get_logger(),
                    "map_localization 就绪: init=(%.2f, %.2f, %.0f deg) submap=%d 帧 | 狗航位推算看门狗%s%s",
                    init_x_, init_y_, init_yaw_deg_, submap_frames_,
                    use_dog_odom_ ? "开(等 /go2/odom)" : "关",
                    resume_candidate_ ? " | 有续接候选(等狗流验证)" : "");
    }

    ~MapLocalizationNode() override
    {
        if (match_thread_.joinable())
            match_thread_.join(); // 等后台匹配退出，避免析构竞态
    }

private:
    struct StampedPose
    {
        double t;    // 流内时间戳（狗钟/livox 钟不同源，只在流内使用）
        double wall; // 接收墙钟秒（跨流对齐专用——狗 stamp 不能索引 LIO 缓冲）
        Eigen::Affine3d T;
        StampedPose(double t_, const Eigen::Affine3d &T_) : t(t_), wall(0.0), T(T_) {}
        StampedPose(double t_, double wall_, const Eigen::Affine3d &T_)
            : t(t_), wall(wall_), T(T_) {}
    };
    enum class Verdict { NO_INFO, GOOD, BAD };

    static pcl::PointCloud<pcl::PointXYZI>::Ptr vox(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &in, double leaf)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.setInputCloud(in);
        vg.filter(*out);
        return out;
    }

    static double stamp2sec(const builtin_interfaces::msg::Time &t)
    {
        return (double)t.sec + 1e-9 * (double)t.nanosec;
    }

    static double yawOf(const Eigen::Affine3d &T)
    {
        return std::atan2(T.rotation()(1, 0), T.rotation()(0, 0));
    }

    static double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static void pushPose(std::deque<StampedPose> &buf, double t, double wall,
                         const Eigen::Affine3d &T, double keep)
    {
        buf.emplace_back(t, wall, T);
        while (!buf.empty() && buf.front().t < t - keep)
            buf.pop_front();
    }

    // 最近 win 秒的内部差分（用各自流自身时间戳，跨度不足不判）
    static bool windowDelta(const std::deque<StampedPose> &buf, double win,
                            double &dpos, double &dyaw, double *wall0_out = nullptr)
    {
        if (buf.size() < 2)
            return false;
        const StampedPose &newest = buf.back();
        if (newest.t - buf.front().t < 0.5 * win)
            return false;
        const StampedPose *old = &buf.front();
        for (const auto &s : buf)
            if (s.t >= newest.t - win)
            {
                old = &s;
                break;
            }
        if (newest.t - old->t < 0.5 * win)
            return false;
        if (wall0_out)
            *wall0_out = old->wall;
        Eigen::Vector3d dp = newest.T.translation() - old->T.translation();
        dpos = std::hypot(dp.x(), dp.y());
        dyaw = wrapAngle(yawOf(newest.T) - yawOf(old->T));
        return true;
    }

    // buf 在墙钟 [wall0, newest] 间隔上的内部差分——与狗窗口等长比较用。
    // 跨流匹配必须走墙钟：狗 stamp 与 livox stamp 不同源，域不可比。
    static bool intervalDelta(const std::deque<StampedPose> &buf, double wall0,
                              double &dpos, double &dyaw)
    {
        if (buf.size() < 2)
            return false;
        const StampedPose &newest = buf.back();
        const StampedPose *old = nullptr;
        for (const auto &s : buf)
            if (s.wall >= wall0 - 0.05)
            {
                old = &s;
                break;
            }
        // LIO 在该间隔覆盖不足（起点晚出 >0.2s）→ 不裁决，防止又一次窗口错配
        if (!old || old == &newest || old->wall - wall0 > 0.2)
            return false;
        Eigen::Vector3d dp = newest.T.translation() - old->T.translation();
        dpos = std::hypot(dp.x(), dp.y());
        dyaw = wrapAngle(yawOf(newest.T) - yawOf(old->T));
        return true;
    }

    // LIO 在狗跳变对所在的墙钟间隔内是否发生了 ≥50% 的可比位移——
    // 狗/LIO 同跳 = 真实运动或 LIO 自身修正，不是狗复位
    static bool lioMovedLikewise(const std::deque<StampedPose> &buf,
                                 double wall0, double wall1, double dp)
    {
        if (buf.size() < 2 || dp <= 0)
            return false;
        const StampedPose *a = nullptr, *b = nullptr;
        for (const auto &s : buf)
        {
            if (!a && s.wall >= wall0 - 0.05)
                a = &s;
            if (s.wall <= wall1 + 0.05)
                b = &s;
        }
        if (!a || !b || b->wall <= a->wall)
            return false;
        double dl = (b->T.translation() - a->T.translation()).head<2>().norm();
        return dl >= 0.5 * dp;
    }

    // ---- 狗四足航位推算回调（dog_odom 系全位姿，只用相对差分）----
    void on_dog(nav_msgs::msg::Odometry::ConstSharedPtr m)
    {
        if (!use_dog_odom_)
            return;
        // P0-3: 断供 >0.5s 后恢复 → 清差分缓冲重建基准（治 10-31 断供后
        // 相邻窗口拿断供前后样本错判 latch；狗系没换，锚点仍有效）
        if (have_dog_ && (now() - last_dog_wall_).seconds() > 0.5)
        {
            dog_buf_.clear();
            RCLCPP_WARN(get_logger(), "[watchdog] 狗流断供 %.1fs 后恢复，重建差分基准",
                        (now() - last_dog_wall_).seconds());
        }
        // P0-1: 续接判决——狗流首帧（或恢复帧）与落盘狗位姿比对
        if (resume_candidate_ && !resume_attempted_ && have_init_)
            try_resume(odom2affine(*m));
        double t = stamp2sec(m->header.stamp);
        if (t <= 0.0)
            return;
        Eigen::Affine3d T = odom2affine(*m);
        // 复位/野值检测：瞬移速度超限（standup/recovery 会重置狗里程计原点）
        // 11-06 修复: (1)阈值 1.5→4.0 m/s——真人快走推杆架就有 1.53m/s 的帧对，
        // 1.5 会在快走段天天误报复位; (2)LIO 同跳排除——狗/LIO 同跳=真实运动或
        // LIO 修正，只有"狗独跳"才配叫复位。
        if (!dog_buf_.empty() && t > dog_buf_.back().t)
        {
            double dt = t - dog_buf_.back().t;
            double dp = (T.translation() - dog_buf_.back().T.translation()).head<2>().norm();
            double wall_now = now().seconds();
            if (dp / dt > dog_jump_speed_ &&
                !lioMovedLikewise(lio_buf_, dog_buf_.back().wall, wall_now, dp))
            {
                dog_resets_++;
                if (lio_bad_)
                {
                    // 死推期间复位：先把锚点推进到复位前最后一帧（位姿连续），
                    // 再换用新世界的狗样本做增量基准
                    T_mb_anchor_ = T_mb_anchor_ * dog_anchor_.T.inverse() * dog_buf_.back().T;
                    dog_anchor_ = StampedPose(t, T);
                }
                dog_buf_.clear();
                RCLCPP_WARN(get_logger(),
                            "[watchdog] 狗里程计疑似复位(%.2fm/%.0fms)第%d次，重建差分基准",
                            dp, dt * 1e3, dog_resets_);
            }
        }
        pushPose(dog_buf_, t, now().seconds(), T, 2.0 * dog_window_);
        have_dog_ = true;
        last_dog_wall_ = now();
        last_dog_msg_ = *m;
        if (lio_bad_ && t >= dog_anchor_.t)
            publishDR(dog_buf_.back(), false, m->header.stamp); // 狗流速率驱动的死推输出
    }

    Verdict eval_consistency()
    {
        // 狗流新鲜度（墙钟，与消息时间戳域无关）
        double dog_age = (now() - last_dog_wall_).seconds();
        if (dog_age > 1.0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "[watchdog] 狗航位推算流断供 %.1fs，暂不裁决", dog_age);
            return Verdict::NO_INFO;
        }
        double lio_d = 0, lio_dyaw = 0, dog_d = 0, dog_dyaw = 0;
        double dog_wall0 = 0;
        // 狗窗口先行（重建后如实缩短，≥0.5×win 才裁决）；LIO 必须在狗窗口的
        // 同一墙钟间隔上取位移。11-06 复盘: 基准重建后狗窗 1.2s vs LIO 窗 2.0s，
        // 快走 0.8m/s 时窗口长度差本身就能顶爆 |Δ|≥0.5 阈值 → mirror 全程一致
        // 却在快走段被假锁"位移分歧"3.2s。等长比较后窗口差不再参与判决。
        if (!windowDelta(dog_buf_, dog_window_, dog_d, dog_dyaw, &dog_wall0))
            return Verdict::NO_INFO;
        if (!intervalDelta(lio_buf_, dog_wall0, lio_d, lio_dyaw))
            return Verdict::NO_INFO;

        bool bad = false;
        const char *why = "";
        if (std::fabs(lio_d - dog_d) > dog_bad_abs_ + dog_bad_rel_k_ * dog_d)
        {
            bad = true;
            why = "位移分歧";
        }
        else if (lio_d > 0.8 && dog_d < 0.15)
        {
            bad = true;
            why = "LIO走狗未走";
        }
        else if (std::fabs(wrapAngle(lio_dyaw - dog_dyaw)) > dog_bad_yaw_ &&
                 std::fabs(dog_dyaw) < 0.2)
        {
            bad = true;
            why = "偏航分歧";
        }
        if (bad)
        {
            healthy_cnt_ = 0;
            if (!lio_bad_)
                latch(why, lio_d, dog_d);
            return Verdict::BAD;
        }
        if (lio_bad_)
        {
            // P0-3 解除迟滞：窗口数与时间跨度双条件（旧版 10 窗≈1s，会震荡）
            if (healthy_cnt_ == 0)
                healthy_since_wall_ = now();
            ++healthy_cnt_;
            if (healthy_cnt_ >= dog_unlatch_checks_ &&
                (now() - healthy_since_wall_).seconds() >= dog_unlatch_span_)
                unlatch();
        }
        return Verdict::GOOD;
    }

    void latch(const char *why, double lio_d, double dog_d)
    {
        lio_bad_ = true;
        latches_++;
        have_dr_last_ = false; // 新一轮死推，冻结保活基准作废
        latched_wall_ = now();
        if (agree_valid_)
        {
            // 用"最后一致"锚点：裁决晚于发病，锚点最多回退一个窗口
            T_mb_anchor_ = T_mb_agree_;
            dog_anchor_ = dog_agree_;
        }
        else
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mb_anchor_ = T_mo_ * lio_buf_.back().T; // 冷启动兜底
            dog_anchor_ = dog_buf_.back();
        }
        RCLCPP_ERROR(get_logger(),
                     "[watchdog] %s: LIO Δ%.2fm vs 狗 Δ%.2fm → 位姿切四足死推(第%d次)",
                     why, lio_d, dog_d, latches_);
        publish_health_now();
    }

    void unlatch()
    {
        // 位姿连续：按当前死推位姿反推 T_mo，无跳变交还 LIO+NDT
        Eigen::Affine3d T_mb_dr = T_mb_anchor_ * dog_anchor_.T.inverse() * dog_buf_.back().T;
        Eigen::Affine3d T_ob = lio_buf_.back().T;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mo_ = T_mb_dr * T_ob.inverse();
        }
        lio_bad_ = false;
        healthy_cnt_ = 0;
        have_dr_last_ = false;
        agree_valid_ = false; // 窗口重新积累后再刷锚点
        RCLCPP_WARN(get_logger(),
                    "[watchdog] 分歧解除 → 回归 LIO+NDT（T_mo 按死推位姿续接，无跳变）");
        publish_health_now();
    }

    // 死推位姿输出: T_mb = T_mb_anchor * inv(T_dog_anchor) * T_dog_now；
    // 冻结保活 = 最后发布的死推位姿（R5 修复：旧版回跳初始锚点）
    void publishDR(const StampedPose &dog_now, bool frozen, const rclcpp::Time &stamp)
    {
        Eigen::Affine3d T_mb = frozen ? (have_dr_last_ ? T_mb_dr_last_ : T_mb_anchor_)
                                       : T_mb_anchor_ * dog_anchor_.T.inverse() * dog_now.T;
        if (!frozen)
        {
            T_mb_dr_last_ = T_mb; // 冻结保活的"最后有效输出"
            have_dr_last_ = true;
        }
        nav_msgs::msg::Odometry out;
        out.header.stamp = stamp;
        out.header.frame_id = map_frame_;
        out.child_frame_id = body_frame_;
        out.pose.pose.position.x = T_mb.translation().x();
        out.pose.pose.position.y = T_mb.translation().y();
        out.pose.pose.position.z = T_mb.translation().z();
        Eigen::Quaterniond q(T_mb.rotation());
        out.pose.pose.orientation.w = q.w();
        out.pose.pose.orientation.x = q.x();
        out.pose.pose.orientation.y = q.y();
        out.pose.pose.orientation.z = q.z();
        if (!frozen)
        {
            // 狗 twist 视为机体系 → map 系；病态期不再采信 LIO 速度
            Eigen::Vector3d vb(last_dog_msg_.twist.twist.linear.x,
                               last_dog_msg_.twist.twist.linear.y,
                               last_dog_msg_.twist.twist.linear.z);
            Eigen::Vector3d vw = T_mb.rotation() * vb;
            out.twist.twist.linear.x = vw.x();
            out.twist.twist.linear.y = vw.y();
            out.twist.twist.linear.z = vw.z();
            out.twist.twist.angular.z = last_dog_msg_.twist.twist.angular.z;
        }
        pub_loc_->publish(out);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = body_frame_;
        tf.transform.translation.x = T_mb.translation().x();
        tf.transform.translation.y = T_mb.translation().y();
        tf.transform.translation.z = T_mb.translation().z();
        tf.transform.rotation = out.pose.pose.orientation;
        tfb_->sendTransform(tf);
        // 病态期不发 map->camera_init：LIO 坐标系与 map 的关系已不可信
    }

    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr m)
    {
        // 里程计首帧 → 初始化 T_mo（只覆盖 x/y/yaw，roll/pitch/z 沿用）
        if (!have_init_)
        {
            Eigen::Affine3d T_ob0 = odom2affine(*m);
            Eigen::Affine3d T_mb = T_ob0;
            Eigen::Vector3d t = T_mb.translation();
            t.x() = init_x_;
            t.y() = init_y_;
            T_mb.translation() = t;
            // 只替换 yaw（把里程计朝向绕 z 旋到 init_yaw）
            Eigen::Matrix3d R = T_mb.rotation();
            double yaw = std::atan2(R(1, 0), R(0, 0));
            double roll = std::atan2(R(2, 1), R(2, 2));
            double pitch = std::asin(-R(2, 0));
            Eigen::Matrix3d Rz;
            Rz = Eigen::AngleAxisd(init_yaw_deg_ * M_PI / 180.0 - yaw, Eigen::Vector3d::UnitZ());
            T_mb.linear() = Rz * R;
            (void)roll;
            (void)pitch;
            {
                std::lock_guard<std::mutex> lk(t_mo_mutex_);
                T_mo_ = T_mb * T_ob0.inverse();
                T_mo_init_ = T_mo_;
            }
            have_init_ = true;
            resume_deadline_wall_ = now(); // 续接判决宽限从初始化起算
            RCLCPP_INFO(get_logger(), "[init] T_map_odom 初始化完成");
        }

        last_odom_ = *m;
        have_odom_ = true;
        last_odom_wall_ = now();

        Eigen::Affine3d T_ob = odom2affine(*m);
        double t = stamp2sec(m->header.stamp);
        pushPose(lio_buf_, t, last_odom_wall_.seconds(), T_ob, 2.0 * dog_window_); // 看门狗的 LIO 差分缓冲

        // 看门狗裁决（仅在有狗流时生效；无狗数据时行为与旧版一致）
        Verdict v = Verdict::NO_INFO;
        if (use_dog_odom_ && have_dog_)
            v = eval_consistency();

        if (!lio_bad_)
        {
            // 发布组合位姿 map->body（T_mo 可能正被后台匹配线程更新）
            Eigen::Affine3d T_mb;
            {
                std::lock_guard<std::mutex> lk(t_mo_mutex_);
                T_mb = T_mo_ * T_ob;
            }
            if (v == Verdict::GOOD)
            {
                // 持续刷新"最后一致"锚点：将来发病时死推从这里起步
                T_mb_agree_ = T_mb;
                dog_agree_ = dog_buf_.back();
                agree_valid_ = true;
            }

            nav_msgs::msg::Odometry out;
            out.header.stamp = m->header.stamp;
            out.header.frame_id = map_frame_;
            out.child_frame_id = body_frame_;
            out.pose.pose.position.x = T_mb.translation().x();
            out.pose.pose.position.y = T_mb.translation().y();
            out.pose.pose.position.z = T_mb.translation().z();
            Eigen::Quaterniond q(T_mb.rotation());
            out.pose.pose.orientation.w = q.w();
            out.pose.pose.orientation.x = q.x();
            out.pose.pose.orientation.y = q.y();
            out.pose.pose.orientation.z = q.z();
            // 速度: body 系 twist -> map 系
            Eigen::Vector3d vb(m->twist.twist.linear.x, m->twist.twist.linear.y,
                               m->twist.twist.linear.z);
            Eigen::Vector3d vw = T_mb.rotation() * vb;
            out.twist.twist.linear.x = vw.x();
            out.twist.twist.linear.y = vw.y();
            out.twist.twist.linear.z = vw.z();
            out.twist.twist.angular = m->twist.twist.angular;
            pub_loc_->publish(out);

            // TF: map->body
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = m->header.stamp;
            tf.header.frame_id = map_frame_;
            tf.child_frame_id = body_frame_;
            tf.transform.translation.x = T_mb.translation().x();
            tf.transform.translation.y = T_mb.translation().y();
            tf.transform.translation.z = T_mb.translation().z();
            tf.transform.rotation = out.pose.pose.orientation;
            tfb_->sendTransform(tf);

            // TF: map->camera_init (漂移修正量)
            geometry_msgs::msg::TransformStamped tf2;
            tf2.header.stamp = m->header.stamp;
            tf2.header.frame_id = map_frame_;
            tf2.child_frame_id = odom_frame_;
            Eigen::Quaterniond q2;
            {
                std::lock_guard<std::mutex> lk(t_mo_mutex_);
                tf2.transform.translation.x = T_mo_.translation().x();
                tf2.transform.translation.y = T_mo_.translation().y();
                tf2.transform.translation.z = T_mo_.translation().z();
                q2 = Eigen::Quaterniond(T_mo_.rotation());
            }
            tf2.transform.rotation.x = q2.x();
            tf2.transform.rotation.y = q2.y();
            tf2.transform.rotation.z = q2.z();
            tfb_->sendTransform(tf2);
        }
        else
        {
            // 病态期：on_dog 以狗流速率发死推位姿；此处仅处理狗流断供的冻结保活
            double dog_age = (now() - last_dog_wall_).seconds();
            if (dog_age > 0.5)
                publishDR(dog_anchor_, true, m->header.stamp);
        }
        // R5: 健康心跳移交 1Hz 墙钟定时器（odom 停了健康也得继续说话）
    }

    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
    {
        if (!have_odom_ || lio_bad_)
            return; // 病态 LIO 的点不入子图，恢复后子图干净、NDT 立即可信
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*m, *pc);
        {
            std::lock_guard<std::mutex> lk(submap_mutex_);
            submap_.push_back(pc);
            while ((int)submap_.size() > submap_frames_)
                submap_.pop_front();
        }
        // NDT 单次 ~1s，放后台线程跑，别阻塞执行器（否则 /loc/pose 掉到 1Hz）
        if ((int)submap_.size() < submap_frames_ || ++frame_cnt_ % match_every_ != 0)
            return;
        if (match_running_.exchange(true))
            return; // 上一轮还没跑完
        if (match_thread_.joinable())
            match_thread_.join(); // 已结束，立即回收
        match_thread_ = std::thread([this] { run_match(); });
    }

    void run_match()
    {
        // 子图指针快照（点云内容不可变，锁内只拷指针列表）
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> snap;
        {
            std::lock_guard<std::mutex> lk(submap_mutex_);
            snap.assign(submap_.begin(), submap_.end());
        }
        if ((int)snap.size() >= submap_frames_)
            match(std::move(snap));
        match_running_ = false;
    }

    void match(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> snap)
    {
        // 拼子图（已同在 odom 系）
        pcl::PointCloud<pcl::PointXYZI>::Ptr acc(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto &c : snap)
            *acc += *c;
        pcl::PointCloud<pcl::PointXYZI>::Ptr src = vox(acc, scan_vox_);
        if (src->size() < 50)
            return;

        // 初始猜测就是 T_mo 本身：子图点已在 odom(camera_init) 系，
        // p_map ≈ T_mo * p_odom（勿乘 T_body——那会把 46° 俯仰卷进猜测）
        Eigen::Matrix4d guess;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            guess = T_mo_.matrix();
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
        ndt_.setInputSource(src);
        ndt_.align(*out, guess.cast<float>());

        matches_++;
        double fitness = ndt_.getFitnessScore();
        Eigen::Affine3d T_new(Eigen::Matrix4d(ndt_.getFinalTransformation().cast<double>()));

        // 失配判据: 未收敛 / fitness 超限 / 相对猜测跳变超限
        double jump = (T_new.translation() - Eigen::Affine3d(guess).translation()).norm();
        bool ok = ndt_.hasConverged() && fitness < fitness_limit_ && jump < jump_limit_;
        Eigen::Affine3d T_mo_after, T_mo_init_snap;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            if (ok)
            {
                // P0 修复(11-06 BBS 重种子被踩回旧盆地): NDT 单次 ~1s，期间
                // reseed/resume/init 可能已改写 T_mo_ —— 本轮结果基于旧猜测，
                // 提交会把新种子一键踩回旧盆地(漂移=(0.19,-0.39)实测即此)。
                // 只在 T_mo_ 仍等于猜测时提交；否则丢弃本轮，下轮从新 T_mo 出发。
                if (T_mo_.matrix().isApprox(guess, 1e-12))
                {
                    T_mo_ = T_new;
                    good_matches_++;
                }
                else
                {
                    RCLCPP_WARN(get_logger(),
                                "[match] T_mo 在本轮计算中被改写(reseed/续接/初始化)，丢弃本轮结果");
                }
            }
            T_mo_after = T_mo_;
            T_mo_init_snap = T_mo_init_; // resume/reseed 会在执行器里改它，读取同锁
        }
        if (matches_ % 50 == 0)
        {
            Eigen::Affine3d d = T_mo_after * T_mo_init_snap.inverse();
            RCLCPP_INFO(get_logger(),
                        "[match] %d/%d ok | fitness=%.3f jump=%.3f | T_mo 漂移=(%.2f,%.2f,%.2f)m",
                        good_matches_, matches_, fitness, jump, d.translation().x(),
                        d.translation().y(), d.translation().z());
        }
    }

    // ---- R5/P0-2: 健康状态 ----
    // 格式 "STATE|reason|age_s"。规划器取 '|' 前缀判准入，人读全文。
    std::string compute_health() const
    {
        char buf[96];
        double since_start = (now() - start_wall_).seconds();
        if (!have_init_)
        {
            std::snprintf(buf, sizeof buf, "INITIALIZING|waiting_odom|%.1f", since_start);
            return buf;
        }
        if (resume_candidate_ && !resume_attempted_ &&
            (now() - resume_deadline_wall_).seconds() <= resume_grace_)
        {
            std::snprintf(buf, sizeof buf, "INITIALIZING|resume_pending|%.1f", since_start);
            return buf;
        }
        double odom_age = (now() - last_odom_wall_).seconds();
        if (lio_bad_)
        {
            double dog_age = have_dog_ ? (now() - last_dog_wall_).seconds() : 1e9;
            if (!have_dog_ || dog_age > 2.0)
            {
                std::snprintf(buf, sizeof buf, "LOST|dead_reckon_dog_gap|%.1f", dog_age);
                return buf;
            }
            std::snprintf(buf, sizeof buf, "DEGRADED|dead_reckoning|%.1f",
                          (now() - latched_wall_).seconds());
            return buf;
        }
        if (odom_age > 2.0)
        {
            std::snprintf(buf, sizeof buf, "STALE|odom_gap|%.1f", odom_age);
            return buf;
        }
        if (use_dog_odom_ && !have_dog_)
        {
            std::snprintf(buf, sizeof buf, "OK|no_dog_watchdog|%.1f", odom_age);
            return buf;
        }
        std::snprintf(buf, sizeof buf, "OK|lio_ndt|%.1f", odom_age);
        return buf;
    }

    void publish_health_now()
    {
        std_msgs::msg::String h;
        h.data = compute_health();
        pub_health_->publish(h); // pub_health_ 构造即建（不再依赖 use_dog_odom_）
    }

    void on_health_timer()
    {
        publish_health_now(); // 1Hz 心跳：与数据流解耦，odom 停也照说 STALE/LOST
    }

    // ---- P0-1: 续接判决（狗流首帧 vs 落盘狗位姿）----
    void try_resume(const Eigen::Affine3d &dog_now)
    {
        resume_attempted_ = true;
        double dp = (dog_now.translation() - saved_dog_pose_.translation()).head<2>().norm();
        double dy = std::fabs(wrapAngle(yawOf(dog_now) - yawOf(saved_dog_pose_)));
        if (dp < resume_dog_tol_ && dy < resume_dog_yaw_)
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mo_ = saved_T_mo_;
            T_mo_init_ = saved_T_mo_;
            agree_valid_ = false;
            RCLCPP_INFO(get_logger(),
                        "[resume] 狗位姿与上次会话差 %.2fm/%.1f° → 续接 T_mo（跳过 known-start）",
                        dp, dy * 180.0 / M_PI);
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "[resume] 狗已移动 %.2fm/%.1f°（或狗里程计已重置）→ 维持 known-start",
                        dp, dy * 180.0 / M_PI);
        }
        publish_health_now();
    }

    // ---- P0-1: 持久化 ----
    void save_persist()
    {
        if (!have_init_ || lio_bad_)
            return; // 只落盘健康态 T_mo；死推期 T_mo 不可信，保留上一份好值
        Eigen::Matrix4d Tm;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            Tm = T_mo_.matrix();
        }
        std::string tmp = persist_path_ + ".tmp";
        {
            std::ofstream f(tmp);
            if (!f)
                return;
            f << "# loc_persist v1\n";
            f << "map_file " << map_file_ << "\n";
            f << "wall " << std::time(nullptr) << "\n";
            f << "t_mo";
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c)
                    f << " " << Tm(r, c);
            f << "\n";
            if (have_dog_)
            {
                Eigen::Affine3d D = odom2affine(last_dog_msg_);
                Eigen::Quaterniond q(D.rotation());
                f << "dog " << D.translation().x() << " " << D.translation().y() << " "
                  << D.translation().z() << " " << q.x() << " " << q.y() << " " << q.z()
                  << " " << q.w() << "\n";
            }
            else
                f << "dog none\n";
        }
        std::rename(tmp.c_str(), persist_path_.c_str());
    }

    void load_persist()
    {
        std::ifstream f(persist_path_);
        if (!f)
            return; // 没有落盘文件：全新会话，known-start 语义不变
        std::string tag, saved_map, first;
        double wall = 0;
        Eigen::Affine3d Tm = Eigen::Affine3d::Identity();
        Eigen::Affine3d dog = Eigen::Affine3d::Identity();
        bool have_tmo = false, have_dog_pose = false;
        while (f >> tag)
        {
            if (tag == "map_file")
                f >> saved_map;
            else if (tag == "wall")
                f >> wall;
            else if (tag == "t_mo")
            {
                Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 4; ++c)
                        f >> m(r, c);
                if (!f.fail())
                {
                    Tm = Eigen::Affine3d(m);
                    have_tmo = true;
                }
            }
            else if (tag == "dog")
            {
                f >> first;
                if (first == "none")
                    continue;
                std::array<double, 7> v;
                v[0] = std::stod(first);
                for (int i = 1; i < 7; ++i)
                    f >> v[i];
                Eigen::Quaterniond q(v[6], v[3], v[4], v[5]); // w x y z
                if (q.norm() < 0.5)
                    continue;
                dog = Eigen::Affine3d::Identity();
                dog.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
                dog.linear() = q.normalized().toRotationMatrix();
                have_dog_pose = true;
            }
        }
        double age_h = (std::time(nullptr) - wall) / 3600.0;
        if (have_tmo && have_dog_pose && saved_map == map_file_ && age_h < persist_max_age_h_)
        {
            resume_candidate_ = true;
            saved_T_mo_ = Tm;
            saved_dog_pose_ = dog;
            RCLCPP_INFO(get_logger(),
                        "[resume] 落盘 %.1fh 前：有续接候选，等狗流验证（%.2fm/%.1f° 内才续接）",
                        age_h, resume_dog_tol_, resume_dog_yaw_ * 180.0 / M_PI);
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "[resume] 落盘不符（地图%s 龄%.1fh 狗位姿%s）→ 走 known-start",
                        saved_map == map_file_ ? "匹配" : "不匹配", age_h,
                        have_dog_pose ? "有" : "无");
        }
    }

    // ---- D: /initialpose 重定位注入（BBS 种子 / RViz 2D Pose Estimate 人工兜底）----
    void on_initialpose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m)
    {
        if (!have_init_ || !have_odom_)
        {
            RCLCPP_WARN(get_logger(), "[reseed] 初始化前收到 /initialpose，忽略");
            return;
        }
        if (lio_bad_)
        {
            RCLCPP_WARN(get_logger(),
                        "[reseed] 看门狗死推中，/initialpose 不生效（死推位姿正确，解除时按死推接管）");
            return;
        }
        const auto &p = m->pose.pose.position;
        const auto &q = m->pose.pose.orientation;
        Eigen::Affine3d T_ob = odom2affine(last_odom_);
        Eigen::Affine3d T_mb_prev = Eigen::Affine3d::Identity();
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mb_prev = T_mo_ * T_ob;
        }
        Eigen::Matrix3d Rmsg = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        double yaw_new = std::atan2(Rmsg(1, 0), Rmsg(0, 0));
        double yaw_prev = yawOf(T_mb_prev);
        Eigen::Affine3d T_mb_new = T_mb_prev;
        T_mb_new.translation().x() = p.x;
        T_mb_new.translation().y() = p.y;
        T_mb_new.linear() = Eigen::AngleAxisd(wrapAngle(yaw_new - yaw_prev),
                                              Eigen::Vector3d::UnitZ()) *
                             T_mb_prev.rotation(); // 只改 yaw，z/roll/pitch 沿用
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mo_ = T_mb_new * T_ob.inverse();
            T_mo_init_ = T_mo_;
        }
        agree_valid_ = false; // 锚点作废，窗口重新积累
        RCLCPP_WARN(get_logger(),
                    "[reseed] T_mo ← /initialpose T_mb=(%.2f, %.2f, yaw %.1f°)，NDT 由此精修",
                    p.x, p.y, yaw_new * 180.0 / M_PI);
        publish_health_now();
    }

    static Eigen::Affine3d odom2affine(const nav_msgs::msg::Odometry &m)
    {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        const auto &p = m.pose.pose.position;
        const auto &q = m.pose.pose.orientation;
        T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        return T;
    }

    // ---- 参数 ----
    std::string map_file_, map_frame_, odom_frame_, body_frame_;
    double init_x_, init_y_, init_yaw_deg_;
    int submap_frames_, ndt_iter_;
    double scan_vox_, map_vox_, ndt_res_, ndt_step_, fitness_limit_, jump_limit_;

    // ---- 狗四足航位推算状态（默认单线程执行器内访问，无锁）----
    bool use_dog_odom_ = true, have_dog_ = false;
    double dog_window_ = 2.0, dog_bad_abs_ = 0.5, dog_bad_rel_k_ = 0.25;
    double dog_bad_yaw_ = 0.8, dog_jump_speed_ = 1.5;
    double dog_unlatch_span_ = 3.0;
    int dog_unlatch_checks_ = 30, healthy_cnt_ = 0, latches_ = 0, dog_resets_ = 0;
    std::deque<StampedPose> lio_buf_, dog_buf_;
    rclcpp::Time last_dog_wall_{0, 0, RCL_ROS_TIME};
    bool lio_bad_ = false, agree_valid_ = false;
    Eigen::Affine3d T_mb_agree_{Eigen::Affine3d::Identity()};
    Eigen::Affine3d T_mb_anchor_{Eigen::Affine3d::Identity()};
    Eigen::Affine3d T_mb_dr_last_{Eigen::Affine3d::Identity()};
    bool have_dr_last_ = false; // R5: 死推期"最后发布的位姿"（冻结保活基准）
    StampedPose dog_agree_{0.0, Eigen::Affine3d::Identity()};
    StampedPose dog_anchor_{0.0, Eigen::Affine3d::Identity()};
    nav_msgs::msg::Odometry last_dog_msg_;
    rclcpp::Time start_wall_{0, 0, RCL_ROS_TIME}, last_odom_wall_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latched_wall_{0, 0, RCL_ROS_TIME}, healthy_since_wall_{0, 0, RCL_ROS_TIME};

    // ---- P0-1 持久化续接 ----
    std::string persist_path_;
    double persist_max_age_h_ = 12.0, resume_dog_tol_ = 0.5, resume_dog_yaw_ = 0.3, resume_grace_ = 1.5;
    bool resume_candidate_ = false, resume_attempted_ = false;
    Eigen::Affine3d saved_T_mo_{Eigen::Affine3d::Identity()};
    Eigen::Affine3d saved_dog_pose_{Eigen::Affine3d::Identity()};
    rclcpp::Time resume_deadline_wall_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr health_timer_, persist_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    // ---- 状态 ----
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ds_{new pcl::PointCloud<pcl::PointXYZI>};
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> submap_;
    std::mutex submap_mutex_;              // 保护 submap_ 队列（执行器写/匹配线程快照）
    std::mutex t_mo_mutex_;                // 保护 T_mo_（匹配线程写/执行器读）
    std::atomic<bool> match_running_{false};
    std::thread match_thread_;
    nav_msgs::msg::Odometry last_odom_;
    Eigen::Affine3d T_mo_{Eigen::Affine3d::Identity()};
    Eigen::Affine3d T_mo_init_{Eigen::Affine3d::Identity()};
    bool have_init_ = false, have_odom_ = false;
    int frame_cnt_ = 0, match_every_ = 2, matches_ = 0, good_matches_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dog_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_loc_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_health_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
