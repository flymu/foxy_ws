// map_localization_node.cpp — 在已建地图上定位：Point-LIO 里程计 + NDT 地图匹配
// 订阅: /Odometry (Point-LIO, camera_init 系), /cloud_registered (camera_init 系点)
// 发布: /loc/pose (nav_msgs/Odometry, map 系, 10Hz), TF: map->body, map->camera_init
// 原理: /cloud_registered 已在 odom(camera_init) 系 → 累积 ~0.8s 子图 S_od；
//       NDT 解 T_map_odom 使 M_map ≈ T_mo * S_od；位姿 = T_mo * T_odom_body。
//       已知起点初始化（参数 init_x/y/yaw 覆盖里程计起点的平移+偏航，roll/pitch/z 沿用
//       里程计首帧——dog 每次从同一位置出发，巡检场景够用；全局绑架重定位留待后续）。
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
        pub_loc_ = create_publisher<nav_msgs::msg::Odometry>("/loc/pose", 10);
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(),
                    "map_localization 就绪: init=(%.2f, %.2f, %.0f deg) submap=%d 帧",
                    init_x_, init_y_, init_yaw_deg_, submap_frames_);
    }

    ~MapLocalizationNode() override
    {
        if (match_thread_.joinable())
            match_thread_.join(); // 等后台匹配退出，避免析构竞态
    }

private:
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

        if (!have_init_)
            return;

        // 发布组合位姿 map->body（T_mo 可能正被后台匹配线程更新）
        Eigen::Affine3d T_ob = odom2affine(*m);
        Eigen::Affine3d T_mb;
        {
            std::lock_guard<std::mutex> lk(t_mo_mutex_);
            T_mb = T_mo_ * T_ob;
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

    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
    {
        if (!have_odom_)
            return;
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_loc_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
