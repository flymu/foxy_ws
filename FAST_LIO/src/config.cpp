#include "fast_lio.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

// Read `node[key]` into `out` if present, leaving `out` at its default
// otherwise. Mirrors the behaviour of ros::NodeHandle::param.
template <typename T>
void getIfPresent(const YAML::Node &node, const char *key, T &out)
{
  if (!node) return;
  const YAML::Node child = node[key];
  if (child && !child.IsNull()) out = child.as<T>();
}

}  // namespace

FastLIOConfig FastLIOConfig::fromYamlFile(const std::string &path)
{
  FastLIOConfig cfg;
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception &e)
  {
    throw std::runtime_error("failed to load config '" + path + "': " + e.what());
  }

  const YAML::Node common = root["common"];
  getIfPresent(common, "time_sync_en", cfg.time_sync_en);
  getIfPresent(common, "time_offset_lidar_to_imu", cfg.time_offset_lidar_to_imu);

  const YAML::Node pre = root["preprocess"];
  getIfPresent(pre, "lidar_type", cfg.lidar_type);
  getIfPresent(pre, "scan_line", cfg.scan_line);
  getIfPresent(pre, "blind", cfg.blind);
  getIfPresent(pre, "timestamp_unit", cfg.timestamp_unit);
  getIfPresent(pre, "scan_rate", cfg.scan_rate);
  getIfPresent(pre, "point_filter_num", cfg.point_filter_num);
  getIfPresent(pre, "feature_extract_enable", cfg.feature_extract_enable);

  const YAML::Node mapping = root["mapping"];
  getIfPresent(mapping, "max_iteration", cfg.max_iteration);
  getIfPresent(mapping, "filter_size_surf", cfg.filter_size_surf);
  getIfPresent(mapping, "filter_size_map", cfg.filter_size_map);
  getIfPresent(mapping, "cube_side_length", cfg.cube_side_length);
  getIfPresent(mapping, "det_range", cfg.det_range);
  getIfPresent(mapping, "fov_degree", cfg.fov_degree);
  getIfPresent(mapping, "gyr_cov", cfg.gyr_cov);
  getIfPresent(mapping, "acc_cov", cfg.acc_cov);
  getIfPresent(mapping, "b_gyr_cov", cfg.b_gyr_cov);
  getIfPresent(mapping, "b_acc_cov", cfg.b_acc_cov);
  getIfPresent(mapping, "extrinsic_est_en", cfg.extrinsic_est_en);
  getIfPresent(mapping, "extrinsic_T", cfg.extrinsic_T);
  getIfPresent(mapping, "extrinsic_R", cfg.extrinsic_R);
  getIfPresent(mapping, "gravity_align", cfg.gravity_align);
  getIfPresent(mapping, "gravity_freeze", cfg.gravity_freeze);
  getIfPresent(mapping, "recovery_en", cfg.recovery_en);
  getIfPresent(mapping, "max_platform_speed", cfg.max_platform_speed);

  const YAML::Node publish = root["publish"];
  getIfPresent(publish, "dense_publish_en", cfg.dense_publish_en);

  getIfPresent(root, "runtime_pos_log_enable", cfg.runtime_pos_log);

  if (cfg.extrinsic_T.size() != 3)
    throw std::runtime_error("mapping/extrinsic_T must have exactly 3 elements");
  if (cfg.extrinsic_R.size() != 9)
    throw std::runtime_error("mapping/extrinsic_R must have exactly 9 elements");

  return cfg;
}
