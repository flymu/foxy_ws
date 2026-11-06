// ndt_basin_repro.cpp — 离线复现 map_localization_node 的 NDT 行为, 验证盆地拉回/分辨率假设
// 用法: ./ndt_basin_repro <map.pcd> <scan.pcd> <res> <gx> <gy> <gyaw_deg> [gz=-0.25]
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>

typedef pcl::PointXYZI PT;

static pcl::PointCloud<PT>::Ptr load(const std::string& p)
{
    pcl::PointCloud<PT>::Ptr c(new pcl::PointCloud<PT>);
    if (pcl::io::loadPCDFile(p, *c) < 0) { fprintf(stderr, "load fail %s\n", p.c_str()); exit(1); }
    return c;
}
static pcl::PointCloud<PT>::Ptr vox(pcl::PointCloud<PT>::Ptr in, float leaf)
{
    pcl::PointCloud<PT>::Ptr out(new pcl::PointCloud<PT>);
    pcl::VoxelGrid<PT> vg; vg.setLeafSize(leaf, leaf, leaf); vg.setInputCloud(in); vg.filter(*out);
    return out;
}

int main(int argc, char** argv)
{
    if (argc < 7) { fprintf(stderr, "usage: %s map scan res gx gy gyaw [gz]\n", argv[0]); return 9; }
    const double res = atof(argv[3]);
    const double gx = atof(argv[4]), gy = atof(argv[5]), gyaw = atof(argv[6]);
    const double gz = argc > 7 ? atof(argv[7]) : -0.25;

    auto map_ds = vox(load(argv[1]), 0.2f);
    auto src = vox(load(argv[2]), 0.1f);
    fprintf(stderr, "[load] map_ds=%zu src=%zu\n", map_ds->size(), src->size());

    pcl::NormalDistributionsTransform<PT, PT> ndt;
    ndt.setResolution(res);
    ndt.setStepSize(0.1);
    ndt.setMaximumIterations(30);
    ndt.setTransformationEpsilon(0.01);
    ndt.setInputTarget(map_ds);
    ndt.setInputSource(src);

    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    const double a = gyaw * M_PI / 180.0;
    guess(0,0)=cos(a); guess(0,1)=-sin(a); guess(1,0)=sin(a); guess(1,1)=cos(a);
    guess(0,3)=gx; guess(1,3)=gy; guess(2,3)=gz;

    pcl::PointCloud<PT> out;
    ndt.align(out, guess);
    const Eigen::Matrix4f T = ndt.getFinalTransformation();
    const double yaw = atan2(T(1,0), T(0,0)) * 180.0 / M_PI;
    printf("[res=%.2f] from(%.2f,%.2f,%.1f deg) -> (%.3f, %.3f, %.3f) yaw=%.2f deg  conv=%d fitness=%.4f\n",
           res, gx, gy, gyaw, T(0,3), T(1,3), T(2,3), yaw,
           (int)ndt.hasConverged(), ndt.getFitnessScore());
    return 0;
}
