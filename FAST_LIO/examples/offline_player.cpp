// Offline FAST-LIO runner.
//
// Reads a directory of per-scan PCD files plus a whitespace-separated IMU text
// file, runs the odometry, and writes a TUM-format trajectory and (optionally)
// the accumulated map.
//
//   ./fastlio_offline --config config/velodyne.yaml \
//                     --pcd_dir /data/scans --imu /data/imu.txt \
//                     --traj Log/traj.txt --map PCD/scans.pcd
//
// IMU file format, one sample per line, time in seconds:
//   timestamp gx gy gz ax ay az
//
// Scan timestamps are taken from the PCD filenames, which must be the scan time
// in seconds (e.g. "1622825334.163313.pcd"). Files are processed in timestamp
// order.

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

#include "fast_lio.h"

namespace {

struct Args
{
  std::string config;
  std::string pcd_dir;
  std::string imu_file;
  std::string traj_out = "Log/traj.txt";
  std::string map_out;
};

void printUsage(const char *argv0)
{
  std::cerr
      << "usage: " << argv0 << " --config <yaml> --pcd_dir <dir> --imu <file>\n"
      << "                  [--traj <out.txt>] [--map <out.pcd>]\n\n"
      << "  --config   YAML config, e.g. config/velodyne.yaml\n"
      << "  --pcd_dir  directory of per-scan .pcd files named <timestamp>.pcd\n"
      << "  --imu      IMU text file: 'timestamp gx gy gz ax ay az' per line\n"
      << "  --traj     TUM-format trajectory output (default Log/traj.txt)\n"
      << "  --map      optional accumulated map written as a single .pcd\n";
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
    else if (!strcmp(a, "--traj")) { auto v = next(a); if (!v) return false; args.traj_out = v; }
    else if (!strcmp(a, "--map")) { auto v = next(a); if (!v) return false; args.map_out = v; }
    else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { printUsage(argv[0]); return false; }
    else { std::cerr << "error: unknown argument '" << a << "'\n"; printUsage(argv[0]); return false; }
  }
  if (args.config.empty() || args.pcd_dir.empty() || args.imu_file.empty())
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
    std::cerr << "error: this offline runner reads PCD files, which cannot carry "
                 "the Livox per-point fields AVIA needs. Use a PointCloud2-style "
                 "config (velodyne/ouster64/marsim), or feed FastLIO::addLidar("
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

  std::ofstream traj(args.traj_out);
  if (!traj)
  {
    std::cerr << "error: cannot write trajectory to '" << args.traj_out << "'\n";
    return 1;
  }
  traj << std::fixed << std::setprecision(6);

  PointCloudXYZI::Ptr accumulated(new PointCloudXYZI());
  const bool save_map = !args.map_out.empty();

  FastLIO lio(cfg);
  int prev_recoveries = 0;
  lio.setStateCallback([&](const LioState &s) {
    if (s.recoveries > prev_recoveries)
    {
      // Annotated events: TUM parsers skip '#' lines, so the trajectory stays
      // loadable while the file records when the watchdog fired.
      traj << "# RECOVERY " << s.recoveries << " at " << std::fixed
           << std::setprecision(3) << s.timestamp << "\n"
           << std::setprecision(6);
      prev_recoveries = s.recoveries;
    }
    traj << s.timestamp << " " << s.pos.x() << " " << s.pos.y() << " " << s.pos.z()
         << " " << s.rot.x() << " " << s.rot.y() << " " << s.rot.z() << " "
         << s.rot.w() << "\n";
  });
  if (save_map)
  {
    lio.setCloudCallback([&](const PointCloudXYZI::Ptr &cloud, double) {
      *accumulated += *cloud;
    });
  }

  size_t imu_idx = 0;
  size_t processed = 0;
  for (const ScanFile &scan : scans)
  {
    // Feed every IMU sample up to the end of this scan before the scan itself,
    // so the filter always has the interval it needs.
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

  // Drain whatever the last scans left buffered.
  while (imu_idx < imu.size()) lio.addImu(imu[imu_idx++]);
  while (lio.processOnce()) processed++;

  traj.close();
  std::cout << "processed " << processed << " scans, map has " << lio.mapSize()
            << " points\n";
  std::cout << "trajectory written to " << args.traj_out << "\n";

  if (save_map && !accumulated->empty())
  {
    accumulated->height = 1;
    accumulated->width = accumulated->size();
    pcl::PCDWriter writer;
    writer.writeBinary(args.map_out, *accumulated);
    std::cout << "map written to " << args.map_out << " (" << accumulated->size()
              << " points)\n";
  }

  return 0;
}
