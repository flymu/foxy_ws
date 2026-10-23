#ifndef FAST_LIO_H
#define FAST_LIO_H

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>

#include <common_lib.h>
#include <lio_types.h>

/// All tunable parameters. Defaults match the values the original ROS node
/// used when a parameter was absent from the config.
struct FastLIOConfig
{
  // --- common ---
  bool   time_sync_en = false;
  double time_offset_lidar_to_imu = 0.0;

  // --- preprocess ---
  int    lidar_type = AVIA;
  int    scan_line = 16;
  double blind = 0.01;
  int    timestamp_unit = US;
  int    scan_rate = 10;
  int    point_filter_num = 2;
  bool   feature_extract_enable = false;

  // --- mapping ---
  int    max_iteration = 4;
  double filter_size_surf = 0.5;
  double filter_size_map = 0.5;
  double cube_side_length = 200.0;
  float  det_range = 300.0f;
  double fov_degree = 180.0;
  double gyr_cov = 0.1;
  double acc_cov = 0.1;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  bool   extrinsic_est_en = true;
  std::vector<double> extrinsic_T = {0.0, 0.0, 0.0};
  std::vector<double> extrinsic_R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  /// Anchor the world frame to gravity at initialization, so world +z points
  /// up regardless of how the sensor is mounted. Set false to reproduce
  /// upstream FAST-LIO, whose world frame is the IMU's initial attitude.
  bool   gravity_align = true;
  /// Stop re-estimating the gravity direction after initialization. With
  /// gravity_align the direction is fixed by construction (world -z), so any
  /// further change is estimation error: violent periodic motion (legged
  /// robots) violates the white-noise process model and the filter folds
  /// sustained acceleration error into the gravity direction, which couples
  /// horizontal motion into z. Implemented by clearing the gravity
  /// cross-covariance before each EKF update, so the update gain acts on it
  /// exactly zero. Set false to keep the upstream behaviour of estimating
  /// gravity as a live S2 state throughout the run.
  bool   gravity_freeze = true;
  /// Detect divergence (matched-feature count collapses while residuals rise,
  /// or the filter holds a velocity the IMU contradicts) and recover: zero the
  /// velocity and accel bias, drop the map and re-seed it from the next scan.
  /// Upstream just skips the update silently while the map keeps absorbing
  /// badly transformed points, so it cannot self-recover.
  bool   recovery_en = true;
  /// Platform speed envelope, m/s. The watchdog treats a velocity held above
  /// this for ~1 s as divergence (match collapse can hide behind an
  /// already-poisoned map, but a physical speed limit cannot lie). 0 disables.
  /// Set it to the platform's real maximum, with headroom: a walking robot
  /// ~2 m/s, a car ~35. The default disables the check since FAST-LIO does
  /// not know its carrier.
  double max_platform_speed = 0.0;

  // --- output / debug ---
  bool   dense_publish_en = true;   // full-resolution registered cloud
  bool   runtime_pos_log = false;   // write timing/state logs under Log/

  /// Load from a YAML file laid out like the files in config/.
  /// Throws std::runtime_error if the file cannot be read or parsed.
  static FastLIOConfig fromYamlFile(const std::string &path);
};

/// The estimated state at the end of a LiDAR scan.
struct LioState
{
  double timestamp = 0.0;          // end time of the scan, seconds
  V3D pos = V3D::Zero();           // IMU position, world frame
  Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();  // IMU attitude
  V3D vel = V3D::Zero();
  V3D bg = V3D::Zero();            // gyro bias
  V3D ba = V3D::Zero();            // accel bias
  V3D grav = V3D::Zero();
  V3D lidar_pos = V3D::Zero();     // LiDAR position, world frame
  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
  // --- health / divergence watchdog ---
  int    effct_feat_num = 0;       // matched features this scan
  double res_mean_last = 0.0;      // mean point-to-plane residual, m
  int    recoveries = 0;           // divergence recoveries so far this run
};

/// ROS-free FAST-LIO odometry and mapping.
///
/// Feed data with addImu() plus one of the addLidar* overloads, then call
/// processOnce() repeatedly. Results arrive through the callbacks. The class is
/// safe to feed from a sensor thread while another thread calls processOnce();
/// it is not safe to call processOnce() from two threads at once.
class FastLIO
{
 public:
  using StateCallback = std::function<void(const LioState &)>;
  /// Registered cloud in the world frame, with the scan's end timestamp.
  using CloudCallback = std::function<void(const PointCloudXYZI::Ptr &, double)>;

  explicit FastLIO(const FastLIOConfig &config);
  ~FastLIO();

  FastLIO(const FastLIO &) = delete;
  FastLIO &operator=(const FastLIO &) = delete;

  void addImu(const ImuData &imu);

  /// Livox frame (lidar_type == AVIA).
  void addLidar(const LivoxCustomMsg &msg);
  /// Spinning LiDAR or simulator frame. The point layout must match
  /// lidar_type: ouster_ros::Point for OUST64, velodyne_ros::Point for VELO16,
  /// pcl::PointXYZI for MARSIM.
  void addLidar(const pcl::PCLPointCloud2 &cloud, double timestamp);

  /// Process at most one scan. Returns true if a scan was consumed, false if
  /// there is not yet enough buffered data.
  bool processOnce();

  void setStateCallback(StateCallback cb);
  /// Registered scan in the world frame.
  void setCloudCallback(CloudCallback cb);
  /// Undistorted scan in the IMU body frame.
  void setBodyCloudCallback(CloudCallback cb);

  const LioState &state() const;
  bool initialized() const;
  /// Number of points currently in the ikd-Tree map.
  int mapSize() const;

  /// Opaque implementation detail; forward-declared publicly so that the
  /// translation unit implementing it can name the type.
  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

#endif  // FAST_LIO_H
