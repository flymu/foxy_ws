// point_lio_node.cpp — de-ROS Point-LIO core 的 ROS2 (Foxy) 包装节点
// 订阅: /livox/lidar (livox_ros_driver2/msg/CustomMsg, xfer_format=1)
//       /livox/imu   (sensor_msgs/Imu, Mid360 内置 IMU, 加速度 g 单位)
// 发布: /Odometry (nav_msgs/Odometry), /cloud_registered (PointCloud2),
//       /path (nav_msgs/Path), TF: <odom_frame> -> <body_frame>
// 主循环逐行移植自 fork 的 src/offline_player.cpp（该循环已在本机 Mid360
// 数据上验证），把文件读写换成话题收发：
//  - CustomMsg -> LivoxScan 转换在回调里做，Preprocess::process 完成点级
//    时间戳/滤波（fork 中此前只声明未实现，见 src/preprocess_livox.cpp）
//  - IMU 缓存到 imu_in_，工作线程独占访问 core 全局 imu_deque
//  - 每帧等待帧尾 IMU 到齐（最多 20ms），不足则继续（丢点容错）
//  - 退出时保存 global_map.pcd / global_map_ds.pcd / trajectory.txt
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <ctime>
#include <deque>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "li_initialization.h"
#include "config_reader.h"
#include "Estimator.h"
#include "sensor_types.h"
#include "preprocess.h"

namespace fs = std::filesystem;

using rclcpp::Node;

// ---------- 工具 ----------
static Eigen::Vector3d rot_to_euler_deg(const M3D &R)
{
    Eigen::Vector3d e;
    e(1) = asin(-std::max(-1.0, std::min(1.0, R(2, 0))));
    if (fabs(cos(e(1))) > 1e-6)
    {
        e(0) = atan2(R(2, 1), R(2, 2));
        e(2) = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        e(0) = atan2(-R(1, 2), R(1, 1));
        e(2) = 0.0;
    }
    return e * 180.0 / M_PI;
}

// ---------- 地图增量（从 offline_player.cpp 移植，含体素去重与离群点剔除） ----------
static void MapIncremental()
{
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i)
    {
        PointType &point_world = feats_down_world->points[i];

        // 剔除变换伪影（x≈0,y≈0 或荒谬的 z）
        float xy_norm_sq = point_world.x * point_world.x + point_world.y * point_world.y;
        if (xy_norm_sq < 1e-6f || std::fabs(point_world.z) > 100.0f)
        {
            continue;
        }

        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++)
            {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
            {
                points_to_add.emplace_back(point_world);
            }
        }
        else
        {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
}

class PointLioNode : public Node
{
public:
    PointLioNode() : Node("point_lio")
    {
        // ---- 参数 ----
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<std::string>("odom_frame", "camera_init");
        this->declare_parameter<std::string>("body_frame", "body");
        this->declare_parameter<std::string>("map_save_path", "~/point_lio_maps");
        std::string cfg = this->get_parameter("config_file").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        body_frame_ = this->get_parameter("body_frame").as_string();
        save_dir_ = this->get_parameter("map_save_path").as_string();
        if (save_dir_.rfind("~", 0) == 0)
            save_dir_ = std::string(getenv("HOME")) + save_dir_.substr(1);
        if (cfg.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "必须给 config_file 参数");
            throw std::runtime_error("no config_file");
        }

        // ---- core 初始化（与 offline_player main 一致，顺序不能乱） ----
        p_pre.reset(new Preprocess());
        p_imu.reset(new ImuProcess());
        try
        {
            readParametersYAML(cfg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "读配置失败: %s", e.what());
            throw;
        }
        RCLCPP_INFO(this->get_logger(), "config: %s", cfg.c_str());

        p_imu->imu_en = imu_en;
        p_imu->lidar_type = lidar_type;
        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
        G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);

        ivox_ = std::make_shared<IVoxType>(ivox_options_);

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        if (extrinsic_est_en)
        {
            if (!use_imu_as_input)
            {
                kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
                kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
            }
            else
            {
                kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
                kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
            }
        }

        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));

        kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
        Eigen::Matrix<double, 24, 24> P_init;
        reset_cov(P_init);
        kf_input.change_P(P_init);
        Eigen::Matrix<double, 30, 30> P_init_output;
        reset_cov_output(P_init_output);
        // 允许滤波器自适应重力模长（消费级 IMU 标度偏差，见 offline_player 注释）
        P_init_output.block<3, 3>(21, 21) = Eigen::Matrix3d::Identity() * 0.01;
        kf_output.change_P(P_init_output);
        Q_input = process_noise_cov_input();
        Q_output = process_noise_cov_output();

        feats_down_body.reset(new PointCloudXYZI());
        feats_down_world.reset(new PointCloudXYZI());
        normvec.reset(new PointCloudXYZI(100000, 1));

        global_map_.reset(new PointCloudXYZI());
        path_msg_.header.frame_id = odom_frame_;

        // ---- ROS 侧 ----
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();  // 兼容可靠/尽力两种发布者，且丢旧保新
        sub_lidar_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, qos, std::bind(&PointLioNode::on_lidar, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos, std::bind(&PointLioNode::on_imu, this, std::placeholders::_1));
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        worker_ = std::thread(&PointLioNode::worker, this);
        RCLCPP_INFO(this->get_logger(),
                    "point_lio ready: %s(%.2f Hz), %s | use_imu_as_input=%d, extr_T=[%.4f %.4f %.4f]",
                    lid_topic.c_str(), 1.0 / lidar_time_inte, imu_topic.c_str(),
                    (int)use_imu_as_input, extrinT[0], extrinT[1], extrinT[2]);
    }

    ~PointLioNode()
    {
        stop_ = true;
        lq_cv_.notify_all();
        if (worker_.joinable())
            worker_.join();
        save_results();
    }

private:
    // ---------- 回调: 只做轻量转换并入队 ----------
    void on_lidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        auto scan = std::make_shared<LivoxScan>();
        scan->timestamp = static_cast<double>(msg->timebase) * 1e-9;
        scan->point_num = msg->point_num;
        scan->points.reserve(msg->points.size());
        for (const auto &p : msg->points)
        {
            LivoxPoint lp;
            lp.x = p.x;
            lp.y = p.y;
            lp.z = p.z;
            lp.reflectivity = static_cast<uint8_t>(p.reflectivity);
            lp.tag = p.tag;
            lp.line = p.line;
            lp.offset_time = p.offset_time;
            scan->points.push_back(lp);
        }
        std::lock_guard<std::mutex> lk(lq_mtx_);
        if (lidar_queue_.size() >= 8)  // 防积压: 丢最旧
        {
            lidar_queue_.pop_front();
            dropped_frames_++;
        }
        lidar_queue_.push_back(std::move(scan));
        lq_cv_.notify_one();
    }

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto d = std::make_shared<ImuData>();
        d->timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        d->angular_velocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        d->linear_acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        std::lock_guard<std::mutex> lk(imu_mtx_);
        imu_in_.push_back(d);
    }

    void drain_imu()
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        while (!imu_in_.empty())
        {
            imu_deque.push_back(imu_in_.front());
            imu_in_.pop_front();
        }
    }

    // ---------- 工作线程: offline_player 主循环的逐行移植 ----------
    void worker()
    {
        auto log = this->get_logger();

        // 跨帧状态（对应 offline_player main 循环外的局部变量）
        bool flg_first_scan = true;
        bool init_map_done = false;
        int frame_count = 0;
        int dead_frames = 0;
        bool in_blackout = false;
        int segment = 0;
        V3D last_tracked_pos = V3D::Zero();
        int frames_processed = 0;
        double total_time = 0.0;
        PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
        PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

        while (!stop_)
        {
            LivoxScan::ConstPtr scan;
            {
                std::unique_lock<std::mutex> lk(lq_mtx_);
                lq_cv_.wait(lk, [&] { return stop_ || !lidar_queue_.empty(); });
                if (stop_)
                    break;
                scan = lidar_queue_.front();
                lidar_queue_.pop_front();
            }

            // IMU 入 imu_deque（core 全局只由本线程访问）
            drain_imu();
            // 等帧尾 IMU 到齐（最多 ~20ms），没到也继续（容忍丢帧）
            if (imu_en)
            {
                double scan_end = scan->timestamp + 0.102;
                for (int w = 0; w < 4; ++w)
                {
                    if (!imu_deque.empty() && imu_deque.back()->timestamp >= scan_end - 0.002)
                        break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    drain_imu();
                }
            }

            PointCloudXYZI::Ptr cloud_ptr(new PointCloudXYZI());
            p_pre->process(scan, cloud_ptr);
            if (cloud_ptr->empty())
            {
                frame_count++;
                continue;
            }

            double cloud_time = scan->timestamp;

            MeasureGroup meas;
            meas.lidar = cloud_ptr;
            meas.lidar_beg_time = cloud_time;

            double end_time = 0.0;
            for (const auto &pt : cloud_ptr->points)
                end_time = std::max(end_time, (double)pt.curvature);
            lidar_end_time = meas.lidar_beg_time + end_time / 1000.0;
            meas.lidar_last_time = lidar_end_time;

            // IMU 同步: 仅初始化阶段打包（对应原 sync_packages 行为）
            if (imu_en && p_imu->imu_need_init_ && !imu_deque.empty())
            {
                double imu_time = imu_deque.front()->timestamp;
                imu_next = *(imu_deque.front());
                while (imu_time < lidar_end_time)
                {
                    meas.imu.emplace_back(imu_deque.front());
                    imu_last = imu_next;
                    imu_deque.pop_front();
                    if (imu_deque.empty())
                        break;
                    imu_time = imu_deque.front()->timestamp;
                    imu_next = *(imu_deque.front());
                }
            }

            if (flg_first_scan)
            {
                first_lidar_time = meas.lidar_beg_time;
                flg_first_scan = false;
                if (imu_en)
                {
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    while (!imu_deque.empty() && meas.lidar_beg_time > imu_next.timestamp)
                    {
                        imu_deque.pop_front();
                        if (imu_deque.empty())
                            break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                }
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            // 1. IMU 初始化 / 去畸变
            p_imu->Process(meas, feats_undistort);
            if (feats_undistort->empty())
            {
                frame_count++;
                continue;
            }

            // 2. 降采样 + 按点时间排序 + 压缩成时间组
            if (space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
            }
            else
            {
                *feats_down_body = *feats_undistort;
            }
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            feats_down_size = feats_down_body->points.size();
            if (feats_down_size < 5)
            {
                frame_count++;
                continue;
            }
            time_seq = time_compressing<int>(feats_down_body);

            // 3. 首次重力对齐（保持地图不倾斜）
            if (!p_imu->after_imu_init_)
            {
                if (!p_imu->imu_need_init_)
                {
                    V3D tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;

                    // 重力模长按实测重新标定（消除恒定自由落体偏差）
                    V3D g_state = kf_output.x_.gravity;
                    double g_meas_mps2 = p_imu->mean_acc.norm() * G_m_s2 / acc_norm;
                    kf_output.x_.gravity = g_state / g_state.norm() * g_meas_mps2;
                    kf_input.x_.gravity = kf_output.x_.gravity;
                    RCLCPP_INFO(log, "[Init] |gravity| rescaled %.4f -> %.4f m/s^2", g_state.norm(), g_meas_mps2);

                    kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;

                    // 陀螺零偏用静止段均值播种
                    kf_input.x_.bg = p_imu->mean_gyr;
                    kf_output.x_.bg = p_imu->mean_gyr;
                    RCLCPP_INFO(log, "[Init] gyro bias seeded: |.|=%.5f rad/s", p_imu->mean_gyr.norm());

                    Eigen::Vector3d e = rot_to_euler_deg(rot_init);
                    RCLCPP_INFO(log, "[Init] attitude rpy(deg): %.2f %.2f %.2f", e(0), e(1), e(2));
                }
                else
                {
                    frame_count++;
                    continue;
                }
            }

            // 4. 首帧初始化局部地图
            if (!init_map_done)
            {
                feats_down_world->resize(feats_undistort->size());
                for (size_t i = 0; i < feats_undistort->size(); i++)
                {
                    pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++)
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if ((int)init_feats_world->size() < init_map_size)
                {
                    init_map_done = false;
                }
                else
                {
                    ivox_->AddPoints(init_feats_world->points);
                    init_feats_world.reset(new PointCloudXYZI());
                    init_map_done = true;
                    RCLCPP_INFO(log, "[Init] local map initialized (%d frames skipped)", frame_count);
                }
                frame_count++;
                continue;
            }

            // 5. 逐点准备量测模型
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            crossmat_list.resize(feats_down_size);
            pbody_list.resize(feats_down_size);
            for (size_t i = 0; i < feats_down_size; i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                               feats_down_body->points[i].y,
                               feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (!extrinsic_est_en)
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list[i] = point_crossmat;
                }
            }

            // 6. 逐点迭代 ESKF 更新（对应 offline_player 第 697-878 行）
            effct_feat_num = 0;
            idx = -1;
            double pcl_beg_time = meas.lidar_beg_time;

            if (use_imu_as_input)
            {
                for (k = 0; k < (int)time_seq.size(); k++)
                {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame)
                    {
                        while (!imu_deque.empty() && time_current > imu_next.timestamp)
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                                break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                        }
                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current;
                        input_in.gyro << imu_last.angular_velocity(0), imu_last.angular_velocity(1), imu_last.angular_velocity(2);
                        input_in.acc << imu_last.linear_acceleration(0), imu_last.linear_acceleration(1), imu_last.linear_acceleration(2);
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    }

                    while (!imu_deque.empty() && time_current > imu_next.timestamp)
                    {
                        imu_deque.pop_front();

                        input_in.gyro << imu_last.angular_velocity(0), imu_last.angular_velocity(1), imu_last.angular_velocity(2);
                        input_in.acc << imu_last.linear_acceleration(0), imu_last.linear_acceleration(1), imu_last.linear_acceleration(2);
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        double dt = imu_last.timestamp - t_last;

                        double dt_cov = imu_last.timestamp - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = imu_last.timestamp;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                        t_last = imu_last.timestamp;

                        if (imu_deque.empty())
                            break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }

                    double dt = time_current - t_last;
                    t_last = time_current;

                    if (!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);

                    if (feats_down_size < 1)
                    {
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified())
                    {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                        float xy_sq = point_world_j.x * point_world_j.x + point_world_j.y * point_world_j.y;
                        if (xy_sq < 1e-6f || std::fabs(point_world_j.z) > 100.0f)
                        {
                            point_world_j.x = point_world_j.y = point_world_j.z = std::numeric_limits<float>::quiet_NaN();
                        }
                    }
                    idx = idx + time_seq[k];
                }
            }
            else
            {
                for (k = 0; k < (int)time_seq.size(); k++)
                {
                    PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame)
                    {
                        if (imu_en)
                        {
                            while (!imu_deque.empty() && time_current > imu_next.timestamp)
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            angvel_avr << imu_last.angular_velocity(0), imu_last.angular_velocity(1), imu_last.angular_velocity(2);
                            acc_avr << imu_last.linear_acceleration(0), imu_last.linear_acceleration(1), imu_last.linear_acceleration(2);
                        }
                        is_first_frame = false;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }

                    if (imu_en && !imu_deque.empty())
                    {
                        bool last_imu = imu_next.timestamp == imu_deque.front()->timestamp;
                        while (imu_next.timestamp < time_predict_last_const && !imu_deque.empty())
                        {
                            if (!last_imu)
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                break;
                            }
                            else
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                        bool imu_comes = time_current > imu_next.timestamp;
                        while (imu_comes)
                        {
                            angvel_avr << imu_next.angular_velocity(0), imu_next.angular_velocity(1), imu_next.angular_velocity(2);
                            acc_avr << imu_next.linear_acceleration(0), imu_next.linear_acceleration(1), imu_next.linear_acceleration(2);

                            double dt = imu_next.timestamp - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = imu_next.timestamp;

                            {
                                double dt_cov = imu_next.timestamp - time_update_last;
                                if (dt_cov > 0.0)
                                {
                                    time_update_last = imu_next.timestamp;
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    kf_output.update_iterated_dyn_share_IMU();
                                }
                            }
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                                break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current > imu_next.timestamp;
                        }
                    }

                    double dt = time_current - time_predict_last_const;
                    if (!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    time_predict_last_const = time_current;

                    if (feats_down_size < 1)
                    {
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified())
                    {
                        idx = idx + time_seq[k];
                        continue;
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j = feats_down_body->points[idx + j + 1];
                        PointType &point_world_j = feats_down_world->points[idx + j + 1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    idx += time_seq[k];
                }
            }
            feats_down_size = feats_down_world->size();

            // 7. 帧质量门控 + 入图（盲区阻尼防速度发散，见 offline_player 注释）
            const int min_effective = std::max(20, (int)feats_down_size / 5);
            const bool frame_tracked = (feats_down_size > 4) && (effct_feat_num >= min_effective);
            if (frame_tracked)
            {
                if (in_blackout)
                {
                    segment++;
                    in_blackout = false;
                    RCLCPP_WARN(log, "blackout ended at t=%.3f, map segment now %d",
                                meas.lidar_beg_time - first_lidar_time, segment);
                }
                MapIncremental();
                dead_frames = 0;
                last_tracked_pos = use_imu_as_input ? kf_input.x_.pos : kf_output.x_.pos;
            }
            else
            {
                if (dead_frames == 0)
                    RCLCPP_WARN(log, "matching blind at t=%.3f (eff=%d/%zu), damping velocity",
                                meas.lidar_beg_time - first_lidar_time, effct_feat_num, feats_down_size);
                const double damp = (dead_frames < 10) ? 0.9 : (dead_frames < 30) ? 0.7 : 0.4;
                if (use_imu_as_input)
                    kf_input.x_.vel *= damp;
                else
                    kf_output.x_.vel *= damp;
                if (dead_frames >= 30)
                {
                    const double pos_damp = (dead_frames < 100) ? 0.95 : 0.8;
                    if (use_imu_as_input)
                        kf_input.x_.pos = kf_input.x_.pos * pos_damp + last_tracked_pos * (1.0 - pos_damp);
                    else
                        kf_output.x_.pos = kf_output.x_.pos * pos_damp + last_tracked_pos * (1.0 - pos_damp);
                }
                dead_frames++;
                if (dead_frames > 100)
                    in_blackout = true;
                if (dead_frames == 400)
                    RCLCPP_ERROR(log, "tracking lost 400 consecutive frames - still trying to re-acquire");
            }

            // 8. 输出
            V3D pos = use_imu_as_input ? kf_input.x_.pos : kf_output.x_.pos;
            M3D rot = use_imu_as_input ? kf_input.x_.rot : kf_output.x_.rot;
            V3D vel = use_imu_as_input ? kf_input.x_.vel : kf_output.x_.vel;
            Eigen::Vector3d e = rot_to_euler_deg(rot);

            if (frame_count % 10 == 0)
            {
                RCLCPP_INFO(log, "Frame %4d: t=%7.3f pts=%5zu eff=%5d | pos=[%8.3f %8.3f %8.3f] rpy=[%6.1f %6.1f %6.1f] | map=%zu%s",
                            frame_count, meas.lidar_beg_time - first_lidar_time, feats_down_size,
                            effct_feat_num, pos(0), pos(1), pos(2), e(0), e(1), e(2), ivox_->NumValidGrids(),
                            (dropped_frames_ ? " (lag!)" : ""));
            }
            traj_lines_.push_back(fmt_traj(meas.lidar_beg_time, pos, e, vel, effct_feat_num));

            publish_state(meas.lidar_beg_time, pos, rot, vel, feats_down_world);

            if (frame_tracked)
            {
                PointCloudXYZI::Ptr valid_world(new PointCloudXYZI());
                valid_world->reserve(feats_down_world->size());
                for (const auto &pt : feats_down_world->points)
                {
                    float xy_norm = pt.x * pt.x + pt.y * pt.y;
                    if (xy_norm < 1e-6f)
                        continue;
                    if (std::fabs(pt.z) > 100.0f)
                        continue;
                    valid_world->push_back(pt);
                }
                *global_map_ += *valid_world;
            }

            total_time += (meas.lidar_last_time - meas.lidar_beg_time);
            frames_processed++;
            frame_count++;
        }

        RCLCPP_INFO(log, "worker exit: %d frames processed, %d dropped, %.1f s lidar time",
                    frames_processed, dropped_frames_.load(), total_time);
    }

    // ---------- 发布 ----------
    void publish_state(double t, const V3D &pos, const M3D &rot, const V3D &vel,
                       const PointCloudXYZI::Ptr &world)
    {
        (void)vel;
        int64_t sec = static_cast<int64_t>(t);
        uint32_t nsec = static_cast<uint32_t>((t - sec) * 1e9);

        Eigen::Quaterniond q(rot);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp.sec = sec;
        odom.header.stamp.nanosec = nsec;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = body_frame_;
        odom.pose.pose.position.x = pos(0);
        odom.pose.pose.position.y = pos(1);
        odom.pose.pose.position.z = pos(2);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        V3D vel_body = rot.transpose() * vel;
        odom.twist.twist.linear.x = vel_body(0);
        odom.twist.twist.linear.y = vel_body(1);
        odom.twist.twist.linear.z = vel_body(2);
        pub_odom_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = odom.header.stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = body_frame_;
        tf.transform.translation.x = pos(0);
        tf.transform.translation.y = pos(1);
        tf.transform.translation.z = pos(2);
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tfb_->sendTransform(tf);

        if (world && !world->empty())
        {
            sensor_msgs::msg::PointCloud2 pc;
            pcl::toROSMsg(*world, pc);
            pc.header.stamp = odom.header.stamp;
            pc.header.frame_id = odom_frame_;
            pub_cloud_->publish(pc);
        }

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = odom.header.stamp;
        ps.header.frame_id = odom_frame_;
        ps.pose = odom.pose.pose;
        path_msg_.poses.push_back(ps);
        if (path_msg_.poses.size() > 4000)
            path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + 1000);
        path_msg_.header.stamp = odom.header.stamp;
        pub_path_->publish(path_msg_);
    }

    static std::string fmt_traj(double t_rel, const V3D &pos, const Eigen::Vector3d &e,
                                const V3D &vel, int eff)
    {
        char buf[256];
        snprintf(buf, sizeof(buf), "%.6f %.6f %.6f %.6f %.4f %.4f %.4f %.4f %.4f %.4f %d",
                 t_rel, pos(0), pos(1), pos(2), e(0), e(1), e(2),
                 vel(0), vel(1), vel(2), eff);
        return std::string(buf);
    }

    // ---------- 退出保存 ----------
    void save_results()
    {
        if (global_map_->empty() && traj_lines_.empty())
            return;
        try
        {
            fs::create_directories(save_dir_);
            std::string base = save_dir_ + "/lio_" + std::to_string(static_cast<long>(time(nullptr)));
            if (!global_map_->empty())
            {
                std::string f = base + "_global_map.pcd";
                if (pcl::io::savePCDFileBinary(f, *global_map_) >= 0)
                    RCLCPP_INFO(this->get_logger(), "saved map: %s (%zu pts)", f.c_str(), global_map_->size());
                PointCloudXYZI::Ptr ds(new PointCloudXYZI());
                downSizeFilterMap.setInputCloud(global_map_);
                downSizeFilterMap.filter(*ds);
                std::string fds = base + "_global_map_ds.pcd";
                if (pcl::io::savePCDFileBinary(fds, *ds) >= 0)
                    RCLCPP_INFO(this->get_logger(), "saved ds map: %s (%zu pts)", fds.c_str(), ds->size());
            }
            std::string tf = base + "_trajectory.txt";
            std::ofstream ofs(tf);
            if (ofs)
            {
                ofs << "# t x y z roll_deg pitch_deg yaw_deg vx vy vz eff_feat\n";
                for (auto &l : traj_lines_)
                    ofs << l << "\n";
                RCLCPP_INFO(this->get_logger(), "saved traj: %s (%zu poses)", tf.c_str(), traj_lines_.size());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "save failed: %s", e.what());
        }
    }

    // ---- 成员 ----
    std::string odom_frame_ = "camera_init";
    std::string body_frame_ = "body";
    std::string save_dir_;

    std::mutex lq_mtx_;
    std::condition_variable lq_cv_;
    std::deque<LivoxScan::ConstPtr> lidar_queue_;
    std::mutex imu_mtx_;
    std::deque<ImuData::Ptr> imu_in_;
    std::atomic<int> dropped_frames_{0};
    std::atomic<bool> stop_{false};
    std::thread worker_;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    Eigen::Matrix<double, 24, 24> Q_input;
    Eigen::Matrix<double, 30, 30> Q_output;

    PointCloudXYZI::Ptr global_map_;
    std::vector<std::string> traj_lines_;
    nav_msgs::msg::Path path_msg_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointLioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
