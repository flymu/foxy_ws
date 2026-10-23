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
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
        dog_jump_speed_ = declare_parameter<double>("dog_jump_speed", 1.5);  // 狗流复位检测(m/s)
        dog_unlatch_checks_ = declare_parameter<int>("dog_unlatch_checks", 10);

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
            pub_health_ = create_publisher<std_msgs::msg::String>("/loc/health", 10);
        }
        pub_loc_ = create_publisher<nav_msgs::msg::Odometry>("/loc/pose", 10);
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(),
                    "map_localization 就绪: init=(%.2f, %.2f, %.0f deg) submap=%d 帧 | 狗航位推算看门狗%s",
                    init_x_, init_y_, init_yaw_deg_, submap_frames_,
                    use_dog_odom_ ? "开(等 /go2/odom)" : "关");
    }

    ~MapLocalizationNode() override
    {
        if (match_thread_.joinable())
            match_thread_.join(); // 等后台匹配退出，避免析构竞态
    }

private:
    struct StampedPose
    {
        double t;
        Eigen::Affine3d T;
        StampedPose(double t_, const Eigen::Affine3d &T_) : t(t_), T(T_) {}
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

    static void pushPose(std::deque<StampedPose> &buf, double t,
                         const Eigen::Affine3d &T, double keep)
    {
        buf.emplace_back(t, T);
        while (!buf.empty() && buf.front().t < t - keep)
            buf.pop_front();
    }

    // 最近 win 秒的内部差分（用各自流自身时间戳，跨度不足不判）
    static bool windowDelta(const std::deque<StampedPose> &buf, double win,
                            double &dpos, double &dyaw)
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
        Eigen::Vector3d dp = newest.T.translation() - old->T.translation();
        dpos = std::hypot(dp.x(), dp.y());
        dyaw = wrapAngle(yawOf(newest.T) - yawOf(old->T));
        return true;
    }

    // ---- 狗四足航位推算回调（dog_odom 系全位姿，只用相对差分）----
    void on_dog(nav_msgs::msg::Odometry::ConstSharedPtr m)
    {
        if (!use_dog_odom_)
            return;
        double t = stamp2sec(m->header.stamp);
        if (t <= 0.0)
            return;
        Eigen::Affine3d T = odom2affine(*m);
        // 复位/野值检测：瞬移速度超限（standup/recovery 会重置狗里程计原点）
        if (!dog_buf_.empty() && t > dog_buf_.back().t)
        {
            double dt = t - dog_buf_.back().t;
            double dp = (T.translation() - dog_buf_.back().T.translation()).head<2>().norm();
            if (dp / dt > dog_jump_speed_)
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
        pushPose(dog_buf_, t, T, 2.0 * dog_window_);
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
        if (!windowDelta(lio_buf_, dog_window_, lio_d, lio_dyaw) ||
            !windowDelta(dog_buf_, dog_window_, dog_d, dog_dyaw))
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
        if (lio_bad_ && ++healthy_cnt_ >= dog_unlatch_checks_)
            unlatch();
        return Verdict::GOOD;
    }

    void latch(const char *why, double lio_d, double dog_d)
    {
        lio_bad_ = true;
        latches_++;
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
        set_health("DEAD_RECKONING");
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
        agree_valid_ = false; // 窗口重新积累后再刷锚点
        RCLCPP_WARN(get_logger(),
                    "[watchdog] 分歧解除 → 回归 LIO+NDT（T_mo 按死推位姿续接，无跳变）");
        set_health("OK");
    }

    // 死推位姿输出: T_mb = T_mb_anchor * inv(T_dog_anchor) * T_dog_now；frozen 则锚点冻结
    void publishDR(const StampedPose &dog_now, bool frozen, const rclcpp::Time &stamp)
    {
        Eigen::Affine3d T_mb = frozen ? T_mb_anchor_
                                       : T_mb_anchor_ * dog_anchor_.T.inverse() * dog_now.T;
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
            RCLCPP_INFO(get_logger(), "[init] T_map_odom 初始化完成");
        }

        last_odom_ = *m;
        have_odom_ = true;

        Eigen::Affine3d T_ob = odom2affine(*m);
        double t = stamp2sec(m->header.stamp);
        pushPose(lio_buf_, t, T_ob, 2.0 * dog_window_); // 看门狗的 LIO 差分缓冲

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

        // 健康状态低频心跳（状态切换时 set_health 已即时发布）
        if (use_dog_odom_ && ++health_cnt_ % 100 == 0)
        {
            std_msgs::msg::String h;
            h.data = health_;
            pub_health_->publish(h);
        }
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
        Eigen::Affine3d T_mo_after;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            if (ok)
            {
                T_mo_ = T_new;
                good_matches_++;
            }
            T_mo_after = T_mo_;
        }
        if (matches_ % 50 == 0)
        {
            Eigen::Affine3d d = T_mo_after * T_mo_init_.inverse();
            RCLCPP_INFO(get_logger(),
                        "[match] %d/%d ok | fitness=%.3f jump=%.3f | T_mo 漂移=(%.2f,%.2f,%.2f)m",
                        good_matches_, matches_, fitness, jump, d.translation().x(),
                        d.translation().y(), d.translation().z());
        }
    }

    void set_health(const std::string &s)
    {
        health_ = s;
        if (!pub_health_)
            return;
        std_msgs::msg::String h;
        h.data = s;
        pub_health_->publish(h);
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
    int dog_unlatch_checks_ = 10, healthy_cnt_ = 0, latches_ = 0, dog_resets_ = 0;
    std::deque<StampedPose> lio_buf_, dog_buf_;
    rclcpp::Time last_dog_wall_{0, 0, RCL_ROS_TIME};
    bool lio_bad_ = false, agree_valid_ = false;
    Eigen::Affine3d T_mb_agree_{Eigen::Affine3d::Identity()};
    Eigen::Affine3d T_mb_anchor_{Eigen::Affine3d::Identity()};
    StampedPose dog_agree_{0.0, Eigen::Affine3d::Identity()};
    StampedPose dog_anchor_{0.0, Eigen::Affine3d::Identity()};
    nav_msgs::msg::Odometry last_dog_msg_;
    std::string health_ = "OK";
    int health_cnt_ = 0;

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
