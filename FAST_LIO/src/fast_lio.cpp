// FAST-LIO core: ROS-free odometry and mapping.
//
// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "fast_lio.h"

#include <omp.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"
#include "lio_log.h"
#include "preprocess.h"
#include "voxel_grid.h"

namespace {

constexpr double kInitTime = 0.1;
constexpr double kLaserPointCov = 0.001;
constexpr float kMovThreshold = 1.5f;
constexpr int kMaxPointsPerScan = 100000;

}  // namespace

struct FastLIO::Impl
{
  explicit Impl(const FastLIOConfig &cfg);

  void addImu(const ImuData &imu);
  void addLidar(const LivoxCustomMsg &msg);
  void addLidar(const pcl::PCLPointCloud2 &cloud, double timestamp);
  bool processOnce();

  bool syncPackages(MeasureGroup &meas);
  void lasermapFovSegment();
  void mapIncremental();
  void hShareModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
  void publishResults();

  void pointBodyToWorld(PointType const *pi, PointType *po) const;
  void pointBodyToWorld(const V3F &pi, V3F &po) const;
  void pointBodyLidarToIMU(PointType const *pi, PointType *po) const;

  FastLIOConfig cfg;

  // --- input buffers ---
  std::mutex mtx_buffer;
  std::deque<double> time_buffer;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer;
  std::deque<ImuDataConstPtr> imu_buffer;
  double last_timestamp_lidar = 0.0;
  double last_timestamp_imu = -1.0;
  double timediff_lidar_wrt_imu = 0.0;
  bool timediff_set_flg = false;
  bool lidar_pushed = false;
  double lidar_mean_scantime = 0.0;
  int scan_num = 0;

  // --- algorithm state ---
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  vect3 pos_lid;
  KD_TREE<PointType> ikdtree;
  std::shared_ptr<Preprocess> p_pre;
  std::shared_ptr<ImuProcess> p_imu;

  PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()};
  PointCloudXYZI::Ptr normvec{new PointCloudXYZI(kMaxPointsPerScan, 1)};
  PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI(kMaxPointsPerScan, 1)};
  PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(kMaxPointsPerScan, 1)};

  std::vector<PointVector> Nearest_Points;
  std::vector<BoxPointType> cub_needrm;
  std::vector<bool> point_selected_surf;
  std::vector<float> res_last;

  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;

  bool flg_first_scan = true;
  bool flg_EKF_inited = false;
  int feats_down_size = 0;
  int effct_feat_num = 0;
  double first_lidar_time = 0.0;
  double lidar_end_time = 0.0;
  double res_mean_last = 0.05;
  double total_residual = 0.0;
  double match_time = 0.0;
  double solve_time = 0.0;
  double kdtree_incremental_time = 0.0;
  double kdtree_delete_time = 0.0;
  int kdtree_delete_counter = 0;
  int add_point_size = 0;

  // --- divergence watchdog ---
  // A diverging run first loses its matches: effct_feat_num collapses while
  // res_mean_last rises, several scans before the velocity estimate runs
  // away. The watchdog tracks exponentially smoothed baselines of both and
  // escalates when both move the wrong way at once. A second, independent
  // signal catches slow-burn divergence where the matcher still agrees with
  // the (already poisoned) map: the IMU says the sensor is essentially static
  // while the filter keeps a nonzero velocity, which is provably wrong.
  static constexpr int kBadScanLimit = 15;     // consecutive bad scans to trigger (~1.5 s at 10 Hz)
  static constexpr double kFeatRatio = 0.35;   // bad scan: matches < 35% of their EMA
  static constexpr double kResRatio = 1.5;     // ... AND residual > 1.5x its EMA
  static constexpr double kVelAbsurd = 10.0;   // m/s; a backstop once speed already ran away
  // Slow-burn phantom walks: after the platform truly stops (deceleration
  // transient over), a quiet IMU can no longer justify any held velocity.
  // Real motion keeps the IMU noisy (leg impacts, wheel vibration), so six
  // full seconds of quiet IMU with |vel| > 0.5 m/s means the velocity is a
  // leftover lie -- the map is being dragged with the pose. 60 scans is
  // longer than any legitimately smooth ride this code has to handle, and
  // longer than the synthetic constvel regression (6 s), which genuinely
  // moves at a constant 0.5 m/s on a permanently quiet IMU.
  static constexpr int kStaticLimit = 60;      // consecutive quiet-IMU scans to trigger (~6 s)
  static constexpr double kStaticAcc = 0.4;    // m/s^2 rms over a scan window => quiet
  static constexpr double kStaticGyr = 0.05;   // rad/s rms over a scan window => quiet
  static constexpr double kStaticVel = 0.5;    // m/s; a held velocity above this while quiet is a lie
  static constexpr int kSpeedLimit = 10;       // scans held above max_platform_speed to trigger (~1 s)
  double feat_ema = 0.0;                       // EMA of effct_feat_num (0 = not seeded yet)
  double res_ema = 0.05;                       // EMA of res_mean_last
  int bad_scans = 0;
  int static_scans = 0;
  int speed_scans = 0;
  int recoveries = 0;
  void updateWatchdog(const MeasureGroup &meas);
  void recover();

  // --- outputs ---
  LioState state_out;
  StateCallback state_cb;
  CloudCallback cloud_cb;
  CloudCallback body_cloud_cb;

  // --- timing log ---
  int frame_num = 0;
  double aver_time_consu = 0.0, aver_time_icp = 0.0, aver_time_match = 0.0;
  double aver_time_incre = 0.0, aver_time_solve = 0.0;
};

// The IKFoM filter takes a plain function pointer for the measurement model, so
// the call has to be routed through a static pointer to the active instance.
// This is set only for the duration of the (synchronous) filter update.
static thread_local FastLIO::Impl *g_active_impl = nullptr;

static void h_share_model_trampoline(state_ikfom &s,
                                     esekfom::dyn_share_datastruct<double> &ekfom_data)
{
  g_active_impl->hShareModel(s, ekfom_data);
}

FastLIO::Impl::Impl(const FastLIOConfig &c)
    : cfg(c), p_pre(new Preprocess()), p_imu(new ImuProcess())
{
  point_selected_surf.assign(kMaxPointsPerScan, true);
  res_last.assign(kMaxPointsPerScan, -1000.0f);

  p_pre->lidar_type = cfg.lidar_type;
  p_pre->N_SCANS = cfg.scan_line;
  p_pre->blind = cfg.blind;
  p_pre->time_unit = cfg.timestamp_unit;
  p_pre->SCAN_RATE = cfg.scan_rate;
  p_pre->point_filter_num = cfg.point_filter_num;
  p_pre->feature_enabled = cfg.feature_extract_enable;

  if (cfg.extrinsic_T.size() != 3 || cfg.extrinsic_R.size() != 9)
  {
    LIO_ERROR("extrinsic_T must have 3 elements and extrinsic_R 9; got %zu and %zu",
              cfg.extrinsic_T.size(), cfg.extrinsic_R.size());
    LIO_ASSERT(cfg.extrinsic_T.size() == 3 && cfg.extrinsic_R.size() == 9);
  }
  V3D Lidar_T_wrt_IMU(VEC_FROM_ARRAY(cfg.extrinsic_T));
  M3D Lidar_R_wrt_IMU;
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(cfg.extrinsic_R);

  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov));
  p_imu->set_acc_cov(V3D(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov));
  p_imu->set_gyr_bias_cov(V3D(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov));
  p_imu->lidar_type = cfg.lidar_type;
  p_imu->gravity_align = cfg.gravity_align;

  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model_trampoline,
                    cfg.max_iteration, epsi);
}

void FastLIO::Impl::addImu(const ImuData &imu_in)
{
  auto imu = std::make_shared<ImuData>(imu_in);
  imu->timestamp -= cfg.time_offset_lidar_to_imu;
  if (cfg.time_sync_en && std::abs(timediff_lidar_wrt_imu) > 0.1)
  {
    imu->timestamp += timediff_lidar_wrt_imu;
  }

  std::lock_guard<std::mutex> lock(mtx_buffer);
  if (imu->timestamp < last_timestamp_imu)
  {
    LIO_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }
  last_timestamp_imu = imu->timestamp;
  imu_buffer.push_back(imu);
}

void FastLIO::Impl::addLidar(const LivoxCustomMsg &msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  if (msg.timestamp < last_timestamp_lidar)
  {
    LIO_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
    time_buffer.clear();
  }
  last_timestamp_lidar = msg.timestamp;

  if (!cfg.time_sync_en && std::abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
      !imu_buffer.empty() && !lidar_buffer.empty())
  {
    printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
           last_timestamp_imu, last_timestamp_lidar);
  }

  if (cfg.time_sync_en && !timediff_set_flg &&
      std::abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
  {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);
}

void FastLIO::Impl::addLidar(const pcl::PCLPointCloud2 &cloud, double timestamp)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  if (timestamp < last_timestamp_lidar)
  {
    LIO_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
    time_buffer.clear();
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(cloud, timestamp, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(timestamp);
  last_timestamp_lidar = timestamp;
}

bool FastLIO::Impl::syncPackages(MeasureGroup &meas)
{
  std::lock_guard<std::mutex> lock(mtx_buffer);
  if (lidar_buffer.empty() || imu_buffer.empty()) return false;

  /*** push a lidar scan ***/
  if (!lidar_pushed)
  {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();

    if (meas.lidar->points.size() <= 1)  // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      LIO_WARN("Too few input point cloud!");
    }
    else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    }
    else
    {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime +=
          (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
    }
    if (cfg.lidar_type == MARSIM) lidar_end_time = meas.lidar_beg_time;

    meas.lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) return false;

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->timestamp;
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
  {
    imu_time = imu_buffer.front()->timestamp;
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void FastLIO::Impl::pointBodyToWorld(PointType const *pi, PointType *po) const
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
               state_point.pos);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void FastLIO::Impl::pointBodyToWorld(const V3F &pi, V3F &po) const
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
               state_point.pos);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void FastLIO::Impl::pointBodyLidarToIMU(PointType const *pi, PointType *po) const
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void FastLIO::Impl::lasermapFovSegment()
{
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  V3D pos_LiD = pos_lid;
  if (!Localmap_Initialized)
  {
    for (int i = 0; i < 3; i++)
    {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cfg.cube_side_length / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cfg.cube_side_length / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++)
  {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= kMovThreshold * cfg.det_range ||
        dist_to_map_edge[i][1] <= kMovThreshold * cfg.det_range)
      need_move = true;
  }
  if (!need_move) return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist = std::max((cfg.cube_side_length - 2.0 * kMovThreshold * cfg.det_range) * 0.5 * 0.9,
                            double(cfg.det_range * (kMovThreshold - 1)));
  for (int i = 0; i < 3; i++)
  {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= kMovThreshold * cfg.det_range)
    {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
    else if (dist_to_map_edge[i][1] <= kMovThreshold * cfg.det_range)
    {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);

  double delete_begin = omp_get_wtime();
  if (!cub_needrm.empty()) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void FastLIO::Impl::mapIncremental()
{
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  const double filter_size_map_min = cfg.filter_size_map;
  for (int i = 0; i < feats_down_size; i++)
  {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    /* decide if need add to map */
    if (!Nearest_Points[i].empty() && flg_EKF_inited)
    {
      const PointVector &points_near = Nearest_Points[i];
      bool need_add = true;
      PointType mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
      {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
      {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist)
        {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
    }
    else
    {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  ikdtree.Add_Points(PointToAdd, true);
  ikdtree.Add_Points(PointNoNeedDownsample, false);
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time = omp_get_wtime() - st_time;
}

void FastLIO::Impl::updateWatchdog(const MeasureGroup &meas)
{
  // Called once per scan after the update, with effct_feat_num and
  // res_mean_last set for that scan. A scan counts as bad only when BOTH
  // signals move against their own baselines: slow or stationary segments
  // legitimately starve the matcher of features, but there the residuals stay
  // low, while divergence drives the two apart.
  const bool suspect =
      feats_down_size >= 20 &&
      feat_ema > 0 &&
      effct_feat_num < kFeatRatio * feat_ema &&
      res_mean_last > kResRatio * std::max(res_ema, 0.05);

  if (suspect)
  {
    bad_scans++;
  }
  else
  {
    // Seed the baseline on the first observation and on healthy scans only,
    // so bad scans cannot drag the baseline down to meet them.
    const double a = 0.2;
    feat_ema = (feat_ema > 0) ? (1 - a) * feat_ema + a * effct_feat_num : effct_feat_num;
    res_ema = (1 - a) * res_ema + a * res_mean_last;
    bad_scans = 0;
  }

  // Quiet-IMU phantom-walk check: with the platform genuinely at rest the IMU
  // is silent, and any velocity the filter still holds is provably a lie --
  // the pose is walking away while the map gets dragged along. This catches
  // slow-burn divergence the suspect test misses, because the matcher keeps
  // agreeing with the map it itself poisoned. Real motion always shows in
  // the IMU (leg impacts, wheel vibration), and a truly constant-velocity
  // ride only differs at kStaticLimit's horizon -- the trigger demands six
  // full seconds of quiet AND a held velocity, which no real platform
  // produces while actually moving.
  double acc_var = 0.0, gyr_var = 0.0;
  if (!meas.imu.empty())
  {
    const size_t n = meas.imu.size();
    V3D acc_mean = V3D::Zero(), gyr_mean = V3D::Zero();
    for (const auto &s : meas.imu)
    {
      acc_mean += s->linear_acceleration;
      gyr_mean += s->angular_velocity;
    }
    acc_mean /= n;
    gyr_mean /= n;
    // Specific force of a static sensor is exactly -gravity, so the
    // fluctuation about the mean -- not the mean itself -- is the motion
    // signal. Discretization noise is ~0.01 m/s^2, vibration an order more.
    for (const auto &s : meas.imu)
    {
      acc_var += (s->linear_acceleration - acc_mean).squaredNorm();
      gyr_var += (s->angular_velocity - gyr_mean).squaredNorm();
    }
    acc_var /= n;
    gyr_var /= n;
  }
  const double acc_rms = std::sqrt(acc_var);
  const double gyr_rms = std::sqrt(gyr_var);
  const bool imu_quiet = !meas.imu.empty() && acc_rms < kStaticAcc && gyr_rms < kStaticGyr;
  const double speed = state_point.vel.norm();
  if (imu_quiet && speed > kStaticVel)
  {
    static_scans++;
  }
  else
  {
    static_scans = 0;
  }

  // Platform speed envelope: unlike the tests above, this one cannot be
  // fooled by a poisoned map -- it is a physical impossibility argument, so
  // it catches divergence that still "matches" perfectly. Ten consecutive
  // scans (~1 s) ride through single-frame glitches and short bursts.
  const bool over_speed = cfg.max_platform_speed > 0.0 && speed > cfg.max_platform_speed;
  speed_scans = over_speed ? speed_scans + 1 : 0;

  if (!cfg.recovery_en) return;

  const bool runaway = speed > kVelAbsurd;
  const bool collapsed = bad_scans >= kBadScanLimit;
  const bool phantom = static_scans >= kStaticLimit;
  const bool overspeed = speed_scans >= kSpeedLimit;
  if (runaway || collapsed || phantom || overspeed)
  {
    const char *why = runaway ? "velocity runaway" : collapsed ? "match collapse"
                          : phantom             ? "quiet-imu phantom walk"
                                               : "platform speed breach";
    LIO_WARN("divergence detected (%s): %d/%d bad scans, %d/%d quiet-vel scans, "
             "%d/%d over-speed scans, matches %d (baseline %.1f), residual %.4f "
             "(baseline %.4f), |vel| %.3f m/s (imu rms acc %.3f m/s^2, gyr %.4f rad/s)"
             " -- recovering",
             why, bad_scans, kBadScanLimit, static_scans, kStaticLimit,
             speed_scans, kSpeedLimit,
             effct_feat_num, feat_ema, res_mean_last, res_ema, speed,
             acc_rms, gyr_rms);
    if (recoveries >= 3)
    {
      LIO_WARN("already recovered %d times this run; the data may be unusable, "
               "or the thresholds need retuning", recoveries);
    }
    recover();
  }
}

void FastLIO::Impl::recover()
{
  // Keep position, attitude and gravity: the sensor is rigidly mounted, so its
  // gravity estimate stays valid and re-estimating it mid-motion would be
  // wrong. Zero the states that ran away and loosen their covariance.
  state_ikfom s = kf.get_x();
  s.vel = V3D::Zero();
  s.ba = V3D::Zero();
  kf.change_x(s);

  // state_ikfom DOF layout: pos 0, rot 3, offset_R_L_I 6, offset_T_L_I 9,
  // vel 12, bg 15, ba 18, grav 21.
  auto P = kf.get_P();
  P.block<3, 3>(12, 12) *= 100.0;
  P.block<3, 3>(18, 18) *= 100.0;
  kf.change_P(P);

  // Drop the map: it has absorbed badly transformed points, which is what
  // poisons every later match. The next scan re-seeds it through the
  // Root_Node == nullptr branch in processOnce().
  ikdtree.Clear();
  Localmap_Initialized = false;

  recoveries++;
  bad_scans = 0;
  static_scans = 0;
  speed_scans = 0;
  feat_ema = 0.0;   // reseed from the next healthy scan
  res_ema = 0.05;
}

void FastLIO::Impl::hShareModel(state_ikfom &s,
                                esekfom::dyn_share_datastruct<double> &ekfom_data)
{
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

  /** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (int i = 0; i < feats_down_size; i++)
  {
    PointType &point_body = feats_down_body->points[i];
    PointType &point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = Nearest_Points[i];

    if (ekfom_data.converge)
    {
      /** Find the closest surfaces in the map **/
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS
                                   ? false
                                   : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
    }

    if (!point_selected_surf[i]) continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f))
    {
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s_score = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s_score > 0.9)
      {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++)
  {
    if (point_selected_surf[i])
    {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1)
  {
    ekfom_data.valid = false;
    LIO_WARN("No Effective Points!");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);  // 23
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; i++)
  {
    const PointType &laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (cfg.extrinsic_est_en)
    {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
          VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    }
    else
    {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time += omp_get_wtime() - solve_start_;
}

void FastLIO::Impl::publishResults()
{
  state_out.timestamp = lidar_end_time;
  state_out.pos = state_point.pos;
  state_out.rot = Eigen::Quaterniond(state_point.rot.coeffs()[3], state_point.rot.coeffs()[0],
                                     state_point.rot.coeffs()[1], state_point.rot.coeffs()[2]);
  state_out.vel = state_point.vel;
  state_out.bg = state_point.bg;
  state_out.ba = state_point.ba;
  state_out.grav = V3D(state_point.grav[0], state_point.grav[1], state_point.grav[2]);
  state_out.lidar_pos = V3D(pos_lid[0], pos_lid[1], pos_lid[2]);
  state_out.effct_feat_num = effct_feat_num;
  state_out.res_mean_last = res_mean_last;
  state_out.recoveries = recoveries;

  // Reorder the filter's covariance into [position, rotation] blocks, matching
  // what the original node put on the odometry message.
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++)
  {
    int k = i < 3 ? i + 3 : i - 3;
    state_out.pose_cov(i, 0) = P(k, 3);
    state_out.pose_cov(i, 1) = P(k, 4);
    state_out.pose_cov(i, 2) = P(k, 5);
    state_out.pose_cov(i, 3) = P(k, 0);
    state_out.pose_cov(i, 4) = P(k, 1);
    state_out.pose_cov(i, 5) = P(k, 2);
  }

  if (state_cb) state_cb(state_out);

  if (cloud_cb)
  {
    PointCloudXYZI::Ptr src(cfg.dense_publish_en ? feats_undistort : feats_down_body);
    int size = src->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
      pointBodyToWorld(&src->points[i], &laserCloudWorld->points[i]);
    }
    cloud_cb(laserCloudWorld, lidar_end_time);
  }

  if (body_cloud_cb)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
      pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }
    body_cloud_cb(laserCloudIMUBody, lidar_end_time);
  }
}

bool FastLIO::Impl::processOnce()
{
  if (!syncPackages(Measures)) return false;

  if (flg_first_scan)
  {
    first_lidar_time = Measures.lidar_beg_time;
    p_imu->first_lidar_time = first_lidar_time;
    flg_first_scan = false;
    return true;
  }

  match_time = 0;
  solve_time = 0;
  double t0 = omp_get_wtime();

  p_imu->Process(Measures, kf, feats_undistort);
  state_point = kf.get_x();
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

  if (feats_undistort == nullptr || feats_undistort->empty())
  {
    LIO_WARN("No point, skip this scan!");
    return true;
  }

  flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) >= kInitTime;

  /*** Segment the map in lidar FOV ***/
  lasermapFovSegment();

  /*** downsample the feature points in a scan ***/
  voxelGridDownsample(*feats_undistort, cfg.filter_size_surf, *feats_down_body);
  double t1 = omp_get_wtime();
  feats_down_size = feats_down_body->points.size();

  /*** initialize the map kdtree ***/
  if (ikdtree.Root_Node == nullptr)
  {
    if (feats_down_size > 5)
    {
      ikdtree.set_downsample_param(cfg.filter_size_map);
      feats_down_world->resize(feats_down_size);
      for (int i = 0; i < feats_down_size; i++)
      {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
      }
      ikdtree.Build(feats_down_world->points);
    }
    return true;
  }

  /*** ICP and iterated Kalman filter update ***/
  if (feats_down_size < 5)
  {
    LIO_WARN("No point, skip this scan!");
    return true;
  }

  if (feats_down_size > kMaxPointsPerScan)
  {
    LIO_WARN("Scan has %d points after downsampling, above the %d limit; extra points are dropped."
             " Increase filter_size_surf to avoid this.",
             feats_down_size, kMaxPointsPerScan);
    feats_down_size = kMaxPointsPerScan;
  }

  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  Nearest_Points.resize(feats_down_size);

  /*** iterated state estimation ***/
  double t_update_start = omp_get_wtime();
  double solve_H_time = 0;
  if (cfg.gravity_freeze)
  {
    // Detach the gravity direction from this update. Its cross-covariance
    // with the other states is what lets the measurement update move the S2
    // state; zeroing the off-diagonal rows/columns makes the update gain
    // exactly zero on those two dimensions, so gravity stays where
    // gravity_align put it. The 2x2 diagonal block is kept so P stays
    // invertible for the information-form solve inside the update. predict()
    // rebuilds the cross terms each frame, so they are cleared here, right
    // before the update they would act through.
    auto P = kf.get_P();
    P.template block<2, 23>(21, 0).setZero();
    P.template block<23, 2>(0, 21).setZero();
    P(21, 21) = P(22, 22) = 1e-5;
    kf.change_P(P);
  }
  g_active_impl = this;
  kf.update_iterated_dyn_share_modified(kLaserPointCov, solve_H_time);
  g_active_impl = nullptr;
  state_point = kf.get_x();
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  double t_update_end = omp_get_wtime();

  updateWatchdog(Measures);
  if (ikdtree.Root_Node == nullptr)
  {
    // A watchdog recovery just dropped the map: skip adding this scan's
    // points so the diverged transform cannot poison the fresh map, and let
    // the next scan seed it through the Root_Node == nullptr branch above.
    publishResults();
    return true;
  }

  /*** add the feature points to map kdtree ***/
  double t3 = omp_get_wtime();
  mapIncremental();
  double t5 = omp_get_wtime();

  publishResults();

  if (cfg.runtime_pos_log)
  {
    frame_num++;
    aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
    aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
    aver_time_match = aver_time_match * (frame_num - 1) / frame_num + match_time / frame_num;
    aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + kdtree_incremental_time / frame_num;
    aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
    printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
           "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f \n",
           t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
           aver_time_icp);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Public facade
// ---------------------------------------------------------------------------

FastLIO::FastLIO(const FastLIOConfig &config) : impl_(new Impl(config)) {}
FastLIO::~FastLIO() = default;

void FastLIO::addImu(const ImuData &imu) { impl_->addImu(imu); }
void FastLIO::addLidar(const LivoxCustomMsg &msg) { impl_->addLidar(msg); }
void FastLIO::addLidar(const pcl::PCLPointCloud2 &cloud, double timestamp)
{
  impl_->addLidar(cloud, timestamp);
}
bool FastLIO::processOnce() { return impl_->processOnce(); }

void FastLIO::setStateCallback(StateCallback cb) { impl_->state_cb = std::move(cb); }
void FastLIO::setCloudCallback(CloudCallback cb) { impl_->cloud_cb = std::move(cb); }
void FastLIO::setBodyCloudCallback(CloudCallback cb) { impl_->body_cloud_cb = std::move(cb); }

const LioState &FastLIO::state() const { return impl_->state_out; }
bool FastLIO::initialized() const { return impl_->flg_EKF_inited; }
int FastLIO::mapSize() const { return impl_->ikdtree.size(); }
