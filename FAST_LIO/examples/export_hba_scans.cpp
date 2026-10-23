// Export per-scan HBA input from a FAST-LIO offline run.
//
// Runs the same odometry pipeline as fastlio_offline, but instead of a TUM
// trajectory and one accumulated map, it writes the per-scan undistorted
// point clouds in the LiDAR frame (at each scan's end time) plus the
// matching world-frame poses, in exactly the layout the HBA mapper expects:
//
//   <out_dir>/pcd/<seq %05d>.pcd   one cloud per processed scan, LiDAR frame
//   <out_dir>/pose.json           "tx ty tz qw qx qy qz" per line, same order
//   <out_dir>/pose_init.json      identical copy of pose.json (HBA overwrites
//                                 pose.json with its optimized result)
//   <out_dir>/times.txt           scan end timestamps, one per line
//   <out_dir>/pose_init.tum       TUM-format copy for trajectory comparison
//
// Pose convention: p_world = q * p_lidar + t with (q, t) from pose.json, i.e.
// T_world<-lidar at the scan's end time: R_WL = R_WI * R_LI,
// t_WL = R_WI * t_L + t_WI. The cloud callback inverts pointBodyToWorld:
// p_lidar = R_LI^T * (R_WI^T * (p_world - t_WI) - t_L).
//
// Scans without a published state (the two that seed the map/IMU init, the
// re-seed scan after a watchdog recovery, and any the IMU data cannot cover)
// emit nothing, so pose.json lines pair 1:1 with the pcd files by index.
//
//   ./fastlio_export_hba --config config/mid360_txt.yaml \
//                       --pcd_dir /data/scans --imu /data/imu.txt \
//                       --out_dir /data/hba_input [--fill_num 5]

#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <cstdlib>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fast_lio.h"

namespace {

struct Args
{
  std::string config;
  std::string pcd_dir;
  std::string imu_file;
  std::string out_dir;
  int fill_num = 5;
};

void printUsage(const char *argv0)
{
  std::cerr
      << "usage: " << argv0 << " --config <yaml> --pcd_dir <dir> --imu <file>\n"
      << "                  --out_dir <dir> [--fill_num <width>]\n\n"
      << "  --config   YAML config, e.g. config/mid360_txt.yaml\n"
      << "  --pcd_dir  directory of per-scan .pcd files named <timestamp>.pcd\n"
      << "  --imu      IMU text file: 'timestamp gx gy gz ax ay az' per line\n"
      << "  --out_dir  output directory (pcd/, pose.json, pose_init.json, ...)\n"
      << "  --fill_num zero-padding width for pcd file names (default 5)\n";
}

bool parseArgs(int argc, char **argv, Args &args)
{
  for (int i = 1; i < argc; i++)
  {
    const char *a = argv[i];
    auto next = [&](const char *name) -> const char * {
      if (i + 1 >= argc)
      {
        std::cerr << "error: " << name << " needs a value\n";
        return nullptr;
      }
      return argv[++i];
    };
    if (!strcmp(a, "--config")) { auto v = next(a); if (!v) return false; args.config = v; }
    else if (!strcmp(a, "--pcd_dir")) { auto v = next(a); if (!v) return false; args.pcd_dir = v; }
    else if (!strcmp(a, "--imu")) { auto v = next(a); if (!v) return false; args.imu_file = v; }
    else if (!strcmp(a, "--out_dir")) { auto v = next(a); if (!v) return false; args.out_dir = v; }
    else if (!strcmp(a, "--fill_num")) { auto v = next(a); if (!v) return false; args.fill_num = std::atoi(v); }
    else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { printUsage(argv[0]); return false; }
    else { std::cerr << "error: unknown argument '" << a << "'\n"; printUsage(argv[0]); return false; }
  }
  if (args.config.empty() || args.pcd_dir.empty() || args.imu_file.empty() ||
      args.out_dir.empty())
  {
    printUsage(argv[0]);
    return false;
  }
  return true;
}

struct ScanFile
{
  double timestamp;
  std::string path;
  bool operator<(const ScanFile &o) const { return timestamp < o.timestamp; }
};

/// Collect <timestamp>.pcd files, sorted by the timestamp in the filename.
bool listScans(const std::string &dir, std::vector<ScanFile> &out)
{
  DIR *d = opendir(dir.c_str());
  if (!d)
  {
    std::cerr << "error: cannot open directory '" << dir << "'\n";
    return false;
  }
  while (dirent *e = readdir(d))
  {
    std::string name = e->d_name;
    if (name.size() < 5 || name.compare(name.size() - 4, 4, ".pcd") != 0) continue;
    std::string stem = name.substr(0, name.size() - 4);
    try
    {
      size_t consumed = 0;
      double ts = std::stod(stem, &consumed);
      if (consumed != stem.size()) throw std::invalid_argument("trailing characters");
      out.push_back({ts, dir + "/" + name});
    }
    catch (const std::exception &)
    {
      std::cerr << "warning: skipping '" << name
                << "' (filename is not a timestamp)\n";
    }
  }
  closedir(d);
  std::sort(out.begin(), out.end());
  return true;
}

bool loadImu(const std::string &path, std::vector<ImuData> &out)
{
  std::ifstream f(path);
  if (!f)
  {
    std::cerr << "error: cannot open IMU file '" << path << "'\n";
    return false;
  }
  std::string line;
  size_t line_no = 0;
  while (std::getline(f, line))
  {
    line_no++;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    ImuData d;
    if (!(ss >> d.timestamp >> d.angular_velocity.x() >> d.angular_velocity.y() >>
          d.angular_velocity.z() >> d.linear_acceleration.x() >>
          d.linear_acceleration.y() >> d.linear_acceleration.z()))
    {
      std::cerr << "warning: skipping malformed IMU line " << line_no << "\n";
      continue;
    }
    out.push_back(d);
  }
  if (out.empty())
  {
    std::cerr << "error: no usable IMU samples in '" << path << "'\n";
    return false;
  }
  std::sort(out.begin(), out.end(),
            [](const ImuData &a, const ImuData &b) { return a.timestamp < b.timestamp; });
  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  Args args;
  if (!parseArgs(argc, argv, args)) return 1;

  FastLIOConfig cfg;
  try
  {
    cfg = FastLIOConfig::fromYamlFile(args.config);
  }
  catch (const std::exception &e)
  {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  if (cfg.lidar_type == AVIA)
  {
    std::cerr << "error: this exporter reads PCD files, which cannot carry "
                 "the Livox per-point fields AVIA needs. Use a PointCloud2-style "
                 "config (e.g. mid360_txt.yaml), or feed FastLIO::addLidar("
                 "LivoxCustomMsg) from your own reader.\n";
    return 1;
  }

  std::vector<ScanFile> scans;
  if (!listScans(args.pcd_dir, scans)) return 1;
  if (scans.empty())
  {
    std::cerr << "error: no <timestamp>.pcd files found in '" << args.pcd_dir << "'\n";
    return 1;
  }

  std::vector<ImuData> imu;
  if (!loadImu(args.imu_file, imu)) return 1;

  std::cout << "loaded " << scans.size() << " scans and " << imu.size()
            << " IMU samples\n";

  // Output directories: <out_dir> and <out_dir>/pcd (mkdir -p style: the
  // parent may not exist either).
  std::string pcd_dir = args.out_dir + "/pcd";
  {
    std::string cur;
    for (size_t i = 0; i < args.out_dir.size(); i++)
    {
      if (args.out_dir[i] == '/' && !cur.empty())
      {
        if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
        {
          std::cerr << "error: cannot create '" << cur << "': " << strerror(errno) << "\n";
          return 1;
        }
      }
      cur += args.out_dir[i];
    }
    if (mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST)
    {
      std::cerr << "error: cannot create '" << cur << "': " << strerror(errno) << "\n";
      return 1;
    }
  }
  if (mkdir(pcd_dir.c_str(), 0755) != 0 && errno != EEXIST)
  {
    std::cerr << "error: cannot create '" << pcd_dir << "'\n";
    return 1;
  }

  // LiDAR->IMU extrinsic from the config: p_imu = R_LI * p_lidar + t_L.
  Eigen::Matrix3d R_LI;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) R_LI(r, c) = cfg.extrinsic_R[r * 3 + c];
  Eigen::Vector3d t_L(cfg.extrinsic_T[0], cfg.extrinsic_T[1], cfg.extrinsic_T[2]);

  // state_cb fires immediately before cloud_cb inside the same publishResults()
  // call, so the latched state below is always the pose that produced the cloud.
  LioState last_state;
  FastLIO lio(cfg);
  lio.setStateCallback([&](const LioState &s) { last_state = s; });

  std::vector<std::string> pose_lines;
  std::vector<double> pose_times;
  size_t exported = 0;

  lio.setCloudCallback([&](const PointCloudXYZI::Ptr &world, double) {
    const Eigen::Quaterniond &q_WI = last_state.rot;
    const Eigen::Vector3d &t_WI = last_state.pos;
    const Eigen::Matrix3d R_WI = q_WI.toRotationMatrix();

    // World -> LiDAR frame at scan end (inverse of pointBodyToWorld).
    const Eigen::Matrix3d R_IW = R_WI.transpose();
    const Eigen::Matrix3d R_LI_W = R_LI.transpose();
    const Eigen::Matrix3d R_lidar_from_world = R_LI_W * R_IW;

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar(new pcl::PointCloud<pcl::PointXYZ>);
    lidar->resize(world->points.size());
    for (size_t i = 0; i < world->points.size(); i++)
    {
      const auto &pw = world->points[i];
      Eigen::Vector3d p_world(pw.x, pw.y, pw.z);
      Eigen::Vector3d p_imu = R_IW * (p_world - t_WI);
      Eigen::Vector3d p_lidar = R_LI_W * (p_imu - t_L);
      lidar->points[i].x = p_lidar(0);
      lidar->points[i].y = p_lidar(1);
      lidar->points[i].z = p_lidar(2);
    }
    lidar->width = lidar->points.size();
    lidar->height = 1;
    lidar->is_dense = true;

    std::ostringstream name;
    name << pcd_dir << "/" << std::setw(args.fill_num) << std::setfill('0')
         << exported << ".pcd";
    if (pcl::io::savePCDFileBinary(name.str(), *lidar) < 0)
    {
      std::cerr << "error: cannot write '" << name.str() << "'\n";
      std::exit(1);
    }

    // HBA pose for this scan: T_world<-lidar at scan end.
    Eigen::Matrix3d R_WL = R_WI * R_LI;
    Eigen::Vector3d t_WL = R_WI * t_L + t_WI;
    Eigen::Quaterniond q_WL(R_WL);
    q_WL.normalize();
    std::ostringstream line;
    line << std::setprecision(15) << t_WL(0) << " " << t_WL(1) << " " << t_WL(2)
         << " " << q_WL.w() << " " << q_WL.x() << " " << q_WL.y() << " "
         << q_WL.z();
    pose_lines.push_back(line.str());
    pose_times.push_back(last_state.timestamp);
    exported++;
  });

  size_t imu_idx = 0;
  size_t processed = 0;
  for (const ScanFile &scan : scans)
  {
    while (imu_idx < imu.size() && imu[imu_idx].timestamp <= scan.timestamp + 0.5)
    {
      lio.addImu(imu[imu_idx++]);
    }

    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(scan.path, cloud) < 0)
    {
      std::cerr << "warning: failed to read '" << scan.path << "', skipping\n";
      continue;
    }
    lio.addLidar(cloud, scan.timestamp);

    while (lio.processOnce()) processed++;
  }

  while (imu_idx < imu.size()) lio.addImu(imu[imu_idx++]);
  while (lio.processOnce()) processed++;

  // pose.json — one line per exported scan, WITH a trailing newline (HBA's
  // read_pose tolerates it after the eof fix; write_pose omits it, so the
  // trailing newline marks files this tool produced). setprecision(15) keeps
  // double round-trip precision; std::fixed would print meaningless digits
  // and lose the defaultfloat exponent form for timestamps.
  {
    std::ofstream pose(args.out_dir + "/pose.json", std::ios::trunc);
    pose << std::setprecision(15);
    for (const std::string &l : pose_lines) pose << l << "\n";
  }
  {
    std::ofstream pose(args.out_dir + "/pose_init.json", std::ios::trunc);
    pose << std::setprecision(15);
    for (const std::string &l : pose_lines) pose << l << "\n";
  }
  {
    std::ofstream times(args.out_dir + "/times.txt", std::ios::trunc);
    times << std::setprecision(15);
    for (double t : pose_times) times << t << "\n";
  }
  {
    // TUM: ts tx ty tz qx qy qz qw — parse the tx..qz from the stored lines.
    std::ofstream tum(args.out_dir + "/pose_init.tum", std::ios::trunc);
    tum << std::setprecision(15);
    for (size_t i = 0; i < pose_lines.size(); i++)
    {
      std::istringstream ss(pose_lines[i]);
      double tx, ty, tz, qw, qx, qy, qz;
      ss >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
      tum << pose_times[i] << " " << tx << " " << ty << " " << tz << " " << qx
          << " " << qy << " " << qz << " " << qw << "\n";
    }
  }

  std::cout << "processed " << processed << " scans, exported " << exported
            << " scan-pose pairs to " << args.out_dir << "\n";
  return 0;
}
