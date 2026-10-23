// Assemble per-scan HBA pcds into one registered map using a pose file.
//
//   ./assemble_map --data_path <dir> --pose <dir>/pose.json --out <map.pcd>
//                 [--voxel 0.05] [--fill_num 5]
//
// <dir>/pcd/<num>.pcd holds one LiDAR-frame cloud per scan (as written by
// fastlio_export_hba); each pose line "tx ty tz qw qx qy qz" places scan i in
// the world frame. Used both with the initial poses (map_before.pcd) and with
// HBA's optimized pose.json (map_after.pcd).

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tools.hpp"
#include "mypcl.hpp"
#include "timer.hpp"

int main(int argc, char** argv)
{
  std::string data_path, pose_file, out_file = "map.pcd";
  double voxel_size = 0.05;
  int fill_num = 5;

  for(int i = 1; i < argc; i++)
  {
    if(!strcmp(argv[i], "--data_path") && i+1 < argc) data_path = argv[++i];
    else if(!strcmp(argv[i], "--pose") && i+1 < argc) pose_file = argv[++i];
    else if(!strcmp(argv[i], "--out") && i+1 < argc) out_file = argv[++i];
    else if(!strcmp(argv[i], "--voxel") && i+1 < argc) voxel_size = atof(argv[++i]);
    else if(!strcmp(argv[i], "--fill_num") && i+1 < argc) fill_num = atoi(argv[++i]);
    else
    {
      printf("usage: assemble_map --data_path <dir> --pose <pose.json> --out <map.pcd> "
             "[--voxel 0.05] [--fill_num 5]\n");
      return 1;
    }
  }
  if(data_path.empty() || pose_file.empty())
  {
    printf("usage: assemble_map --data_path <dir> --pose <pose.json> --out <map.pcd> "
           "[--voxel 0.05] [--fill_num 5]\n");
    return 1;
  }
  if(!data_path.empty() && data_path.back() != '/') data_path += "/";

  std::vector<mypcl::pose> pose_vec = mypcl::read_pose(pose_file);
  printf("loaded %zu poses\n", pose_vec.size());
  if(pose_vec.empty())
  {
    printf("error: no poses in %s\n", pose_file.c_str());
    return 1;
  }

  pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
  double t0 = now_sec();
  for(size_t i = 0; i < pose_vec.size(); i++)
  {
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
    mypcl::loadPCD(data_path, fill_num, pc, i, "pcd/");
    if(pc->points.empty())
    {
      printf("error: cannot read scan %zu from %spcd/\n", i, data_path.c_str());
      return 1;
    }
    pcl::PointCloud<PointType>::Ptr pc_world(new pcl::PointCloud<PointType>);
    mypcl::transform_pointcloud(*pc, *pc_world, pose_vec[i].t, pose_vec[i].q);
    map = mypcl::append_cloud(map, *pc_world);
    if((i+1) % 200 == 0)
      printf("scan %zu/%zu, map %zu pts, %.1fs\n", i+1, pose_vec.size(),
             map->points.size(), now_sec()-t0);
  }

  printf("total %zu pts, downsampling at %.2f ... ", map->points.size(), voxel_size);
  downsample_voxel(*map, voxel_size);
  printf("%zu pts\n", map->points.size());

  map->width = map->points.size();
  map->height = 1;
  if(pcl::io::savePCDFileBinary(out_file, *map) < 0)
  {
    printf("error: cannot write %s\n", out_file.c_str());
    return 1;
  }
  printf("wrote %s (%zu pts) in %.1fs\n", out_file.c_str(), map->points.size(),
         now_sec()-t0);
  return 0;
}
