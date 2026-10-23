// scan_planner_node.cpp — SCAN-Planner standalone 的 ROS2 (Foxy) 包装
// 订阅: /loc/pose (map 系位姿+速度, 来自 map_localization),
//       /Odometry + /cloud_registered (camera_init 系, 用于把点云搬到 map 系),
//       /goal_pose (geometry_msgs/PoseStamped, RViz "2D Goal Pose" 兼容)
// 发布: /cmd_vel (geometry_msgs/Twist, body 系, 即 Go2 桥的输入格式),
//       /plan (nav_msgs/Path, 采样后的 B 样条轨迹)
// 驱动: 100Hz 定时器——10Hz 位姿间用 map 系速度积分插值, 喂 setOdometry 后 step()
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
                goal_sent_ = false; // tick 里重试装填
                RCLCPP_INFO(get_logger(), "收到目标 (%.2f, %.2f)", goal_x_, goal_y_);
            });

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_plan_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        timer_ = create_wall_timer(10ms, [this]() { tick(); }); // 100 Hz
        last_tick_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "scan_planner 就绪 (navi_mode=%d), 等待 /loc/pose 与 /goal_pose",
                    navi_mode_);
    }

private:
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
        if (!have_loc_)
            return;

        // 10Hz 位姿间插值: map 系速度积分
        interp_pos_ = loc_pos_ + loc_vel_ * interp_t_;
        interp_t_ += dt;
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
        if (++tick_cnt_ % 200 == 0)
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
    int navi_mode_;
    ScanPlanner planner_;

    Eigen::Vector3d loc_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d loc_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d interp_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond loc_quat_ = Eigen::Quaterniond::Identity();
    Eigen::Affine3d T_odom_body_ = Eigen::Affine3d::Identity();
    double interp_t_ = 0.0;
    bool have_loc_ = false, have_odom_ = false, goal_sent_ = false;
    int tick_cnt_ = 0, traj_count_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_loc_, sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_plan_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_tick_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
