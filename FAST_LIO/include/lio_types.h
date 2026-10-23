#ifndef LIO_TYPES_H
#define LIO_TYPES_H

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

enum LID_TYPE { AVIA = 1, VELO16, OUST64, MARSIM };
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };

/// A single IMU sample. Replaces sensor_msgs::Imu.
struct ImuData
{
  double timestamp = 0.0;                 // seconds
  V3D angular_velocity = V3D::Zero();     // rad/s, body frame
  V3D linear_acceleration = V3D::Zero();  // m/s^2 (or g, rescaled during init)
};

typedef std::shared_ptr<ImuData> ImuDataPtr;
typedef std::shared_ptr<const ImuData> ImuDataConstPtr;

/// One point of a Livox frame. Replaces livox_ros_driver::CustomPoint.
struct LivoxPoint
{
  float x = 0.f, y = 0.f, z = 0.f;
  uint8_t reflectivity = 0;
  uint8_t tag = 0;
  uint8_t line = 0;
  uint32_t offset_time = 0;  // ns relative to the frame timestamp
};

/// One Livox frame. Replaces livox_ros_driver::CustomMsg.
struct LivoxCustomMsg
{
  double timestamp = 0.0;  // seconds, time of the first point
  uint32_t point_num = 0;
  std::vector<LivoxPoint> points;
};

/// The preintegrated LiDAR state at the time of an IMU measurement.
/// Replaces the fast_lio/Pose6D ROS message.
struct Pose6D
{
  double offset_time = 0.0;  // offset w.r.t. the first lidar point
  double acc[3] = {0, 0, 0};  // preintegrated acceleration, global frame
  double gyr[3] = {0, 0, 0};  // unbiased angular velocity, body frame
  double vel[3] = {0, 0, 0};  // preintegrated velocity, global frame
  double pos[3] = {0, 0, 0};  // preintegrated position, global frame
  double rot[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // preintegrated rotation
};

/// A LiDAR scan paired with the IMU samples spanning it.
struct MeasureGroup
{
  MeasureGroup() : lidar(new PointCloudXYZI()) {}

  double lidar_beg_time = 0.0;
  double lidar_end_time = 0.0;
  PointCloudXYZI::Ptr lidar;
  std::deque<ImuDataConstPtr> imu;
};

#endif  // LIO_TYPES_H
