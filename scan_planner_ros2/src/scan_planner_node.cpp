// scan_planner_node.cpp — SCAN-Planner standalone 的 ROS2 (Foxy) 包装
// 订阅: /loc/pose (map 系位姿+速度, 来自 map_localization),
//       /Odometry + /cloud_registered (camera_init 系, 用于把点云搬到 map 系),
//       /goal_pose (geometry_msgs/PoseStamped, RViz "2D Goal Pose" 兼容),
//       /loc/health (std_msgs/String "STATE|reason|age_s", R5 三态健康)
// 发布: /cmd_vel (geometry_msgs/Twist, body 系, 即 Go2 桥的输入格式),
//       /plan (nav_msgs/Path, 采样后的 B 样条轨迹)
// 驱动: 100Hz 定时器——10Hz 位姿间用 map 系速度积分插值(外推封顶), 喂 setOdometry 后 step()
// B3 门控(REVIEW R5): /loc/health 非 OK / 超时 2.5s / loc 位姿超龄 2.0s / 无健康流
//   → fail-closed: 清目标+停轨迹跟踪+显式零速下发, 不喂 odom 不 step;
//   恢复后 goal 复位, 等操作员重新发目标(不自动追旧目标)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>

#include <scan_planner/scan_planner.h>
#include <scan_planner/uniform_bspline.h>

using namespace std::chrono_literals;
using scan_planner::ScanPlanner;
using scan_planner::PlannerParams;
using scan_planner::ControllerParams;
using scan_planner::CmdVel;

class ScanPlannerNode : public rclcpp::Node
{
public:
    ScanPlannerNode() : Node("scan_planner")
    {
        goal_x_ = declare_parameter<double>("goal_x", -1e9);
        goal_y_ = declare_parameter<double>("goal_y", -1e9);
        navi_mode_ = declare_parameter<int>("navi_mode", 1);
        cloud_vox_ = declare_parameter<double>("cloud_vox", 0.05);
        max_vx_ = declare_parameter<double>("max_vx", 0.75);
        // B3: 门控参数（现场可调）
        health_timeout_ = declare_parameter<double>("health_timeout", 2.5);
        loc_age_max_ = declare_parameter<double>("loc_pose_age_max", 2.0);
        interp_cap_ = declare_parameter<double>("interp_cap", 0.25);
        // 11-06: 重开需 OK 持续 ≥1s——健康在 OK/DEGRADED 边界抖动时门控
        // 会 1Hz 反复开关(每沿都 clearTarget)，狗会顿挫；fail-closed 方向
        // 无迟滞(关要快)，只有重开方向加保持。
        reopen_hold_ = declare_parameter<double>("reopen_hold", 1.0);

        // ---- 参数（为 Go2+Mid360 地面场景改无人机默认值）----
        PlannerParams p;
        p.fsm.navi_mode = navi_mode_;
        p.grid_map.need_extrinsic = false; // 我们直接喂 body 位姿
        p.grid_map.cloud_is_world = true;  // 点已转到 map 系
        // 规划平面 = 传感器 z + body_height。0.4(无人机默认)恰好切在桌面高度(+0.4m)
        // 且下坡时 odom z 到 -0.68，平面会低于地图 z 下界 → 全被判成障碍。
        // 0.1 ≈ 狗身中部；地图 z 下界放到 -1.0 盖住最低地形。
        p.fsm.body_height = 0.1;
        p.grid_map.body_height = 0.1;
        p.grid_map.ground_height = -1.0;
        ControllerParams cp;
        cp.max_vx = max_vx_;
        planner_.init(p, cp);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        sub_loc_ = create_subscription<nav_msgs::msg::Odometry>(
            "/loc/pose", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_loc(std::move(m)); });
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", qos,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(std::move(m)); });
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", qos,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(std::move(m)); });
        sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
                goal_x_ = m->pose.position.x;
                goal_y_ = m->pose.position.y;
                goal_sent_ = false; // tick 里重试装填（门控关闭期也收，等重开后生效）
                RCLCPP_INFO(get_logger(), "收到目标 (%.2f, %.2f)%s", goal_x_, goal_y_,
                            gate_closed_ ? "（门控关闭中，暂存待重开）" : "");
            });
        sub_health_ = create_subscription<std_msgs::msg::String>(
            "/loc/health", rclcpp::QoS(10),
            [this](std_msgs::msg::String::ConstSharedPtr m) { on_health(std::move(m)); });

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_plan_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        timer_ = create_wall_timer(10ms, [this]() { tick(); }); // 100 Hz
        last_tick_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "scan_planner 就绪 (navi_mode=%d), 等待 /loc/pose 与 /goal_pose",
                    navi_mode_);
    }

private:
    void on_health(std_msgs::msg::String::ConstSharedPtr m)
    {
        const std::string &s = m->data;                  // "STATE|reason|age_s"
        std::string state = s.substr(0, s.find('|'));   // 只取 STATE 前缀
        if (state.rfind("OK", 0) == 0)
        {
            if (!ok_running_) // 刚转为 OK：重开保持计时起表
            {
                ok_running_ = true;
                ok_since_ = std::chrono::steady_clock::now();
            }
        }
        else
            ok_running_ = false; // 任何非 OK 立即作废保持计时
        health_state_ = std::move(state);
        last_health_t_ = std::chrono::steady_clock::now();
        have_health_ = true;
    }

    static double age_s(const std::chrono::steady_clock::time_point &t,
                         const std::chrono::steady_clock::time_point &now_)
    {
        return std::chrono::duration<double>(now_ - t).count();
    }

    void on_loc(nav_msgs::msg::Odometry::ConstSharedPtr m)
    {
        loc_pos_ = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y,
                                   m->pose.pose.position.z);
        loc_quat_ = Eigen::Quaterniond(m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                                       m->pose.pose.orientation.y, m->pose.pose.orientation.z)
                        .normalized();
        loc_vel_ = Eigen::Vector3d(m->twist.twist.linear.x, m->twist.twist.linear.y,
                                   m->twist.twist.linear.z); // map 系
        have_loc_ = true;
        last_loc_t_ = std::chrono::steady_clock::now();
        interp_t_ = 0.0; // 从最新位姿重新起积分
    }

    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr m)
    {
        const auto &p = m->pose.pose.position;
        const auto &q = m->pose.pose.orientation;
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        T_odom_body_ = T;
        have_odom_ = true;
    }

    void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
    {
        if (!have_loc_ || !have_odom_)
            return;
        if (gate_closed_)
            return; // B3: 门控期不喂地图——坏帧配出来的"障碍"会在恢复后毒害规划
        // camera_init -> map（用最近一次定位修正 + 里程计）
        Eigen::Affine3d T_map_body = Eigen::Affine3d::Identity();
        T_map_body.translation() = loc_pos_;
        T_map_body.linear() = loc_quat_.toRotationMatrix();
        Eigen::Affine3d T_mo = T_map_body * T_odom_body_.inverse();

        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*m, *pc);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setLeafSize(cloud_vox_, cloud_vox_, cloud_vox_);
        vg.setInputCloud(pc);
        vg.filter(*ds);

        std::vector<Eigen::Vector3d> pts;
        pts.reserve(ds->size());
        for (const auto &pt : ds->points)
            pts.push_back(T_mo * Eigen::Vector3d(pt.x, pt.y, pt.z));
        planner_.updateSensor(loc_pos_, loc_quat_, pts);
    }

    void tick()
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_tick_).count();
        last_tick_ = now;
        ++tick_cnt_;

        // ---- B3: 定位健康门控（fail-closed）----
        double h_age = have_health_ ? age_s(last_health_t_, now) : 1e9;
        double l_age = have_loc_ ? age_s(last_loc_t_, now) : 1e9;
        // 重开须 OK 已持续 ≥reopen_hold_（见构造函数注释）；关闭方向无迟滞
        bool ok_enough = ok_running_ && age_s(ok_since_, now) >= reopen_hold_;
        bool closed = h_age > health_timeout_ || l_age > loc_age_max_ || !ok_enough;
        if (closed)
        {
            if (!gate_closed_)
            {
                gate_closed_ = true;
                planner_.clearTarget(); // 弃目标 + 停控制器对旧轨迹的跟踪
                goal_x_ = -1e9;         // 恢复后不自动追旧目标，等 /goal_pose 重发
                goal_sent_ = false;
                RCLCPP_WARN(get_logger(),
                            "[gate] 定位门控关闭 (health=%s h_age=%.1fs l_age=%.1fs) "
                            "→ 清目标+零速保持",
                            health_state_.empty() ? "无流" : health_state_.c_str(),
                            std::min(h_age, 999.0), std::min(l_age, 999.0));
            }
            geometry_msgs::msg::Twist tw; // 显式零 Twist（比沉默强：下游见零不见冻结）
            pub_cmd_->publish(tw);
            if (tick_cnt_ % 200 == 0)
                RCLCPP_INFO(get_logger(), "[gate] 保持关闭 (health=%s)",
                            health_state_.empty() ? "无流" : health_state_.c_str());
            return; // 不喂 odom、不 step：FSM 冻结在 WAIT_TARGET
        }
        if (gate_closed_)
        {
            gate_closed_ = false;
            interp_t_ = 0.0;
            RCLCPP_INFO(get_logger(),
                        "[gate] 定位门控重开 (health=%s, OK已持续%.1fs) → 等待目标",
                        health_state_.c_str(), age_s(ok_since_, now));
        }
        if (!have_loc_)
            return;

        // 10Hz 位姿间插值: map 系速度积分；外推封顶 interp_cap_（默认 0.25s）——
        // 定位断流时位置估计不得按旧速度无限奔跑（REVIEW R5 外推无界修复）
        interp_pos_ = loc_pos_ + loc_vel_ * interp_t_;
        interp_t_ = std::min(interp_t_ + dt, interp_cap_);
        planner_.setOdometry(interp_pos_, loc_quat_, loc_vel_);

        // 目标装填：setGoal 必须在 setOdometry 之后——standalone 在无里程计时
        // 会直接丢弃目标（WARN + return），本节点曾因此卡死在 INIT
        if (!goal_sent_ && goal_x_ > -1e8 && tick_cnt_ % 20 == 0)
        {
            planner_.setGoal(goal_x_, goal_y_);
            if (planner_.hasTarget())
            {
                goal_sent_ = true;
                RCLCPP_INFO(get_logger(), "目标已生效 (%.2f, %.2f), FSM 开始规划",
                            goal_x_, goal_y_);
            }
        }

        const CmdVel &cmd = planner_.step();

        geometry_msgs::msg::Twist tw;
        tw.linear.x = cmd.vx;
        tw.linear.y = cmd.vy;
        tw.angular.z = cmd.vyaw;
        pub_cmd_->publish(tw);

        // 新轨迹 → /plan
        scan_planner::TrajOutput out;
        if (planner_.getTrajectory(out) && out.pos_pts.size() >= 6)
        {
            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            path.header.stamp = this->now();
            Eigen::MatrixXd cpts(3, out.pos_pts.size());
            for (size_t i = 0; i < out.pos_pts.size(); ++i)
                cpts.col(i) = out.pos_pts[i];
            Eigen::VectorXd knots(out.knots.size());
            for (size_t i = 0; i < out.knots.size(); ++i)
                knots(i) = out.knots[i];
            double ts = out.knots.size() >= 2 ? out.knots[1] - out.knots[0] : 0.1;
            scan_planner::UniformBspline bs(cpts, out.order, ts);
            bs.setKnot(knots);
            for (double t = 0.0; t <= out.duration; t += 0.1)
            {
                Eigen::VectorXd p = bs.evaluateDeBoorT(t);
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose.position.x = p(0);
                ps.pose.position.y = p(1);
                ps.pose.position.z = p(2);
                path.poses.push_back(ps);
            }
            pub_plan_->publish(path);
            traj_count_++;
        }

        // 状态日志 2s 一次
        if (tick_cnt_ % 200 == 0)
        {
            const char *st[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
                                "EXEC_TRAJ", "EMERGENCY_STOP"};
            RCLCPP_INFO(get_logger(),
                        "[planner] %s traj=%d target=%d cmd=(%.2f,%.2f,%.2f) pos=(%.2f,%.2f)",
                        st[(int)planner_.fsmState()], traj_count_, (int)planner_.hasTarget(),
                        cmd.vx, cmd.vy, cmd.vyaw, interp_pos_.x(), interp_pos_.y());
        }
    }

    double goal_x_, goal_y_, cloud_vox_, max_vx_;
    double health_timeout_ = 2.5, loc_age_max_ = 2.0, interp_cap_ = 0.25, reopen_hold_ = 1.0;
    int navi_mode_;
    ScanPlanner planner_;

    Eigen::Vector3d loc_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d loc_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d interp_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond loc_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Affine3d T_odom_body_ = Eigen::Affine3d::Identity();
    double interp_t_ = 0.0;
    bool have_loc_ = false, have_odom_ = false, goal_sent_ = false;
    bool have_health_ = false, gate_closed_ = false; // B3: fail-closed 门控状态
    bool ok_running_ = false;                          // OK 是否正在连续保持
    std::string health_state_;                        // "OK"/"INITIALIZING"/...
    std::chrono::steady_clock::time_point last_health_t_, last_loc_t_, ok_since_;
    int tick_cnt_ = 0, traj_count_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_loc_, sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_plan_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_tick_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // standalone 库默认 WARN——INFO 级有 clearTarget/FINISH 等关键状态日志，
    // 现场排"不走"必须能看到（11-06: [clearTarget] 生效日志因默认级别被吞）
    scan_planner::set_log_level(scan_planner::LogLevel::INFO);
    rclcpp::spin(std::make_shared<ScanPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
