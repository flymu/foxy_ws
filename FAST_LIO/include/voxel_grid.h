#ifndef LIO_VOXEL_GRID_H
#define LIO_VOXEL_GRID_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <lio_types.h>

/// Voxel-grid downsampling for PointXYZINormal clouds.
///
/// This replaces pcl::VoxelGrid, which lives in PCL's optional `filters`
/// module, so the core library needs only pcl_common. Semantics match
/// pcl::VoxelGrid's defaults: points are bucketed into cubic voxels of side
/// `leaf`, and each occupied voxel contributes one point whose every field is
/// the mean of the points in it (pcl::VoxelGrid's downsample_all_data_ = true).
/// Averaging `curvature` matters here — FAST-LIO stores the per-point time
/// offset in that field.
///
/// Non-finite points are dropped. Output order is by voxel index, which is
/// deterministic but differs from the input order.
inline void voxelGridDownsample(const PointCloudXYZI &input, float leaf,
                                PointCloudXYZI &output)
{
  output.clear();
  if (input.points.empty()) return;

  if (!(leaf > 0.f))
  {
    output = input;
    return;
  }

  const float inv_leaf = 1.0f / leaf;

  struct Key
  {
    std::int64_t i, j, k;
    bool operator==(const Key &o) const { return i == o.i && j == o.j && k == o.k; }
  };
  struct KeyHash
  {
    size_t operator()(const Key &key) const
    {
      // Three odd primes keep neighbouring voxels from colliding.
      return static_cast<size_t>(key.i * 73856093L ^ key.j * 19349663L ^ key.k * 83492791L);
    }
  };
  struct Accum
  {
    double x = 0, y = 0, z = 0;
    double intensity = 0;
    double normal_x = 0, normal_y = 0, normal_z = 0;
    double curvature = 0;
    std::uint32_t count = 0;
  };

  std::unordered_map<Key, Accum, KeyHash> voxels;
  voxels.reserve(input.points.size() / 2 + 1);

  for (const PointType &p : input.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    Key key;
    key.i = static_cast<std::int64_t>(std::floor(p.x * inv_leaf));
    key.j = static_cast<std::int64_t>(std::floor(p.y * inv_leaf));
    key.k = static_cast<std::int64_t>(std::floor(p.z * inv_leaf));

    Accum &a = voxels[key];
    a.x += p.x;
    a.y += p.y;
    a.z += p.z;
    a.intensity += p.intensity;
    a.normal_x += p.normal_x;
    a.normal_y += p.normal_y;
    a.normal_z += p.normal_z;
    a.curvature += p.curvature;
    a.count++;
  }

  std::vector<const std::pair<const Key, Accum> *> ordered;
  ordered.reserve(voxels.size());
  for (const auto &kv : voxels) ordered.push_back(&kv);
  std::sort(ordered.begin(), ordered.end(),
            [](const std::pair<const Key, Accum> *a, const std::pair<const Key, Accum> *b) {
              if (a->first.k != b->first.k) return a->first.k < b->first.k;
              if (a->first.j != b->first.j) return a->first.j < b->first.j;
              return a->first.i < b->first.i;
            });

  output.points.reserve(ordered.size());
  for (const auto *kv : ordered)
  {
    const Accum &a = kv->second;
    const double inv_n = 1.0 / a.count;
    PointType p;
    p.x = static_cast<float>(a.x * inv_n);
    p.y = static_cast<float>(a.y * inv_n);
    p.z = static_cast<float>(a.z * inv_n);
    p.intensity = static_cast<float>(a.intensity * inv_n);
    p.normal_x = static_cast<float>(a.normal_x * inv_n);
    p.normal_y = static_cast<float>(a.normal_y * inv_n);
    p.normal_z = static_cast<float>(a.normal_z * inv_n);
    p.curvature = static_cast<float>(a.curvature * inv_n);
    output.points.push_back(p);
  }
  output.height = 1;
  output.width = static_cast<std::uint32_t>(output.points.size());
  output.is_dense = true;
}

#endif  // LIO_VOXEL_GRID_H
