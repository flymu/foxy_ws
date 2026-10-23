## Related Works and Extended Application

**SLAM:**

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
3. [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): A robust, real-time LiDAR-IMU extrinsic initialization and synchronization package..
4. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): The integration of FAST-LIO with **Re-localization** function module.
5. [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) | [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2): A state-of-art LiDAR-inertial-visual odometry (LIVO) system with high computational efficiency, robustness, and pixel-level accuracy.

**Control and Plan:**

1. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
2. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
3. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
4. [Bubble Planner](https://arxiv.org/abs/2202.12177): Planning High-speed Smooth Quadrotor Trajectories using Receding Corridors.

<!-- 10. [**FAST-LIVO**](https://github.com/hku-mars/FAST-LIVO): Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry. -->

## FAST-LIO
**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package address many key issues:
1. Fast iterated Kalman filter for odometry optimization;
2. Automaticaly initialized at most steady environments;
3. Parallel KD-Tree Search to decrease the computation;

## FAST-LIO 2.0 (2021-07-05 Update)
<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="left">
<img src="doc/real_experiment2.gif" width=49.6% />
<img src="doc/ulhkwh_fastlio.gif" width = 49.6% >
</div>

**Related video:**  [FAST-LIO2](https://youtu.be/2OvjGnxszf8),  [FAST-LIO1](https://youtu.be/iYCY6T79oNU)

**Pipeline:**
<div align="center">
<img src="doc/overview_fastlio2.svg" width=99% />
</div>

**New Features:**
1. Incremental mapping using [ikd-Tree](https://github.com/hku-mars/ikd-Tree), achieve faster speed and over 100Hz LiDAR rate.
2. Direct odometry (scan to map) on Raw LiDAR points (feature extraction can be disabled), achieving better accuracy.
3. Since no requirements for feature extraction, FAST-LIO2 can support many types of LiDAR including spinning (Velodyne, Ouster) and solid-state (Livox Avia, Horizon, MID-70) LiDARs, and can be easily extended to support more LiDARs.
4. Support external IMU.
5. Support ARM-based platforms including Khadas VIM3, Nivida TX2, Raspberry Pi 4B(8G RAM).

**Related papers**: 

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**Contributors**

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

<!-- <div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div> -->

## 0. This fork is ROS-free

This fork has no ROS dependency. It builds as a plain CMake project and ships as
a library (`libfastlio`) plus a small offline runner:

- **`libfastlio`** — the algorithm, with no I/O of its own. You push IMU samples
  and LiDAR scans in, and get poses and registered clouds out through callbacks.
- **`fastlio_offline`** — an example runner that replays a directory of PCD files
  and an IMU text file through the library.

ROS message types are replaced by plain structs in [lio_types.h](include/lio_types.h):
`ImuData` for `sensor_msgs/Imu`, `LivoxCustomMsg`/`LivoxPoint` for
`livox_ros_driver/CustomMsg`, and `pcl::PCLPointCloud2` for
`sensor_msgs/PointCloud2`. Logging goes to stderr through the macros in
[lio_log.h](include/lio_log.h) instead of `ROS_INFO`/`ROS_WARN`.

The launch files, `msg/`, `rviz_cfg/` and `package.xml` are gone. Everything the
launch files used to set with `<param>` now lives in the [config/](config/) YAML
files, so a config file alone fully describes a run.

## 1. Prerequisites

**Ubuntu >= 18.04** (or any Linux with a C++14 compiler). The distro PCL and
Eigen packages are enough.

| Dependency | Version | Notes |
| --- | --- | --- |
| CMake | >= 3.10 | |
| Eigen | >= 3.3.4 | header-only, [installation](http://eigen.tuxfamily.org/index.php?title=Main_Page) |
| PCL | >= 1.8 | only the `common` and `io` components are used |
| yaml-cpp | any recent | config parsing |
| OpenMP | optional | used for the parallel residual computation when present |

```
sudo apt install cmake libeigen3-dev libpcl-dev libyaml-cpp-dev
```

No `livox_ros_driver`, no ROS. Livox support is built in via the
`LivoxCustomMsg` struct.

## 2. Build

```
    git clone <this repository> FAST_LIO
    cd FAST_LIO
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j$(nproc)
```

This produces `build/libfastlio.a` and `build/fastlio_offline`. ikd-Tree is
vendored in [include/ikd-Tree/](include/ikd-Tree/), so no submodule init is
needed.

To install the library and headers somewhere:

```
    cmake --install build --prefix /usr/local
```

- If you want to use a custom build of PCL, point CMake at it with
  ```-DPCL_DIR={CUSTOM_PCL_PATH}/share/pcl-1.x```.

## 3. Configuration

Pick the YAML file matching your sensor and edit it: [avia.yaml](config/avia.yaml),
[horizon.yaml](config/horizon.yaml), [mid360.yaml](config/mid360.yaml),
[velodyne.yaml](config/velodyne.yaml), [ouster64.yaml](config/ouster64.yaml),
[marsim.yaml](config/marsim.yaml).

The parameters you are most likely to change:

| Key | Meaning |
| --- | --- |
| `preprocess/lidar_type` | 1 Livox serials, 2 Velodyne, 3 Ouster, 4 MARSIM |
| `preprocess/scan_line` | line count (16, 32 and 64 are tested) |
| `preprocess/timestamp_unit` | unit of the per-point time field — 0 s, 1 ms, 2 µs, 3 ns. Set from the **time** (Velodyne) or **t** (Ouster) field of your data |
| `preprocess/point_filter_num` | keep every Nth point |
| `mapping/extrinsic_T` | translational extrinsic |
| `mapping/extrinsic_R` | rotational extrinsic (rotation matrix only) |
| `mapping/extrinsic_est_en` | online extrinsic estimation |
| `mapping/gravity_align` | anchor the world frame to gravity at init, so world **+z** points up whatever the mounting angle. `false` reproduces upstream, whose world frame is the IMU's initial attitude |
| `mapping/gravity_freeze` | stop re-estimating the gravity direction after init (default **true**). A legged robot's gait walks the live estimate off vertical, which shows up as z-drift; freezing keeps the direction at the init value. `false` (with `gravity_align: false`) reproduces upstream |
| `mapping/recovery_en` | divergence watchdog: recover from divergence instead of running away (see below) |
| `mapping/max_platform_speed` | your carrier's physical speed limit, m/s. A velocity held above it for ~1 s is treated as divergence; `0` disables the check |
| `common/time_sync_en` | simple software time sync for Livox LiDARs |

Notes:

A. Make sure the IMU and LiDAR are **synchronized** — that's important.

B. The extrinsic is defined as the LiDAR's pose (position and rotation matrix)
in the **IMU body frame**, i.e. the IMU is the base frame. The values can be
found in your sensor's official manual.

C. We recommend setting **extrinsic_est_en** to false if the extrinsic is given.
For extrinsic initialization, see
[Robust Real-time LiDAR-inertial Initialization](https://github.com/hku-mars/LiDAR_IMU_Init).

D. Turn on `time_sync_en` **ONLY IF** external time synchronization is really
not possible, since the software time sync cannot guarantee accuracy.

E. If per-point timestamps are missing from your data, forward and backward
propagation cannot undistort the scan, and accuracy suffers.

F. `gravity_align` defaults to **true** in this fork, which differs from upstream.
The world frame is levelled at initialization: the minimal rotation taking the
measured up direction onto world +z is used as the initial attitude, and gravity
is set to `(0, 0, -G)`. Yaw stays arbitrary — it is unobservable. Upstream leaves
the initial attitude as identity, so a tilt-mounted sensor yields a correct but
visibly tilted map. Set it to `false` for bit-comparable upstream behaviour.

F'. `gravity_freeze` defaults to **true** in this fork, and needs the why. The
gravity direction is a live S2 state in the iterated EKF, and the measurement
update can rotate it whenever its cross-covariance with the other states is
nonzero. On smooth data that cross-talk is harmless. On a legged robot it is
not: the gait injects quasi-periodic accelerations (|acc| up to 46 m/s² on the
MID-360 dog capture) that violate the white-noise process model for hours of
scans, and the filter folds the sustained error into the gravity direction.
The effect is visible in a probe run: gravity tilt grows from 0° to 2.8° over
a 241 s walk while its covariance stays at ~1e-6 — confidently wrong — and a
2.8° tilt converts horizontal motion into z at tan(2.8°) ≈ 5 %, which over a
120 m loop leaves ~0.75 m of apparent z-drift by the time the dog returns to
its start point. Freezing works at the covariance level: the gravity
cross-covariance rows/columns are cleared before each update (the 2×2 diagonal
block is kept so the information-form solve stays well-conditioned), which
makes the update gain exactly zero on the gravity dimensions, so the direction
cannot move. This is not a flat-ground assumption: a slope tilts the robot,
not the world frame — validated on a synthetic 15° ramp sequence (rel-ATE
0.036 m over a 2 m climb, identical to the unfrozen run). On clean synthetic
data the freeze changes nothing (trajectories bit-identical), and on the dog
capture it collapses the loop-closing end-z spread from 0.72 m across runs
to 0.15 m, with the gravity tilt pinned at exactly 0° and the watchdog never
firing. Set `gravity_freeze: false` (with `gravity_align: false`) to recover
the upstream behaviour of estimating gravity throughout the run.

G. `recovery_en` (default **true**) arms a divergence watchdog with three
independent detectors. A diverging run first loses its map matches: the
matched-feature count collapses while the mean point-to-plane residual rises,
several scans **before** the velocity estimate runs away — the watchdog tracks
exponentially smoothed baselines of both signals and fires after 15 consecutive
suspicious scans (~1.5 s at 10 Hz). Slow-burn divergence, where matching still
"agrees" with an already-poisoned map, is caught physically instead: a velocity
held above `max_platform_speed` for ~1 s is impossible, and a velocity held
above 0.5 m/s for ~6 s while the IMU is silent means the platform is at rest
and the pose is walking away. A 10 m/s absolute backstop covers everything
else. Recovery zeroes velocity and the accel bias, inflates their covariances,
drops the map, and lets the next scan re-seed it — the trajectory continues
from the current pose. Recovery events are annotated in the TUM trajectory as
`# RECOVERY <n> at <t>` lines and counted in `LioState::recoveries`. Why
upstream cannot do this: when matching fails badly enough, the iterated EKF
silently skips the whole scan's measurement update while the map still absorbs
the badly transformed points, so the map is poisoned and matching only gets
worse — there is no path back. Runs that never trigger the watchdog are
numerically unaffected: detection only reads existing state.

H. **The ikd-Tree background rebuild thread was removed.** Upstream ikd-Tree
rebuilds large unbalanced subtrees on a detached thread, splicing the result
back into the tree while the main thread keeps searching and mutating. That
handoff protocol has real race windows: the recursion paths read `Rebuild_Ptr`
without any lock while the background thread clears it mid-splice, the old
subtree is freed after the protecting lock is released (a use-after-free), and
`rebuild_flag` is cleared outside its lock so operations can linger in the
replay logger and get replayed onto *unrelated* subtrees by the *next*
rebuild — which makes whole map blocks vanish. This matches the divergence
precursors observed when replaying the MID-360 capture: ~40 % of runs lost
their map matches at an unpredictable point and the velocity ran away, with a
different outcome on every replay of the same data. In this fork every subtree
rebuild runs inline on the calling thread. That is safe by construction:
FAST-LIO processes each scan in strictly phased steps — parallel matching,
then a join barrier, then serial map updates — so searches and mutations
never overlap and there is exactly one mutator. The lazily-deleted node flags
are `std::atomic` because the OpenMP matching workers still read them while
pushing deletions down. The per-node result is deterministic: the same input
data now yields the same tree, and 6 replays of the 241 s capture all stay
SANE with zero unexplained trajectory steps. The watchdog (note G) stays
armed as a safety net for non-threading failure modes such as single-frame
data anomalies. Rebuilds slower than 50 ms log one `WARN` line as a
latency guardrail; none was hit in the validation runs.

## 4. Run the offline example

`fastlio_offline` replays a directory of per-scan PCD files plus an IMU text
file:

```
    build/fastlio_offline --config config/velodyne.yaml \
                          --pcd_dir /path/to/scans \
                          --imu /path/to/imu.txt \
                          --traj Log/traj.txt \
                          --map Log/scans.pcd
```

| Flag | Meaning |
| --- | --- |
| `--config` | YAML config file |
| `--pcd_dir` | directory of per-scan `.pcd` files named `<timestamp>.pcd`, timestamp in seconds |
| `--imu` | text file, one sample per line: `timestamp gx gy gz ax ay az` (s, rad/s, m/s²) |
| `--traj` | TUM-format trajectory output (default `Log/traj.txt`) |
| `--map` | optional accumulated world-frame map, written as a single `.pcd` |

The point type in each PCD must match `lidar_type`: `ouster_ros::Point` for
Ouster, `velodyne_ros::Point` for Velodyne, `pcl::PointXYZI` for MARSIM.
`lidar_type: 1` (Livox) is **not** supported by this runner, because PCD cannot
carry the per-point Livox fields — feed `FastLIO::addLidar(const LivoxCustomMsg&)`
from your own reader instead.

`pcl_viewer scans.pcd` visualizes the saved map.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

### 4.1 Converting a Livox-SDK2 text dump

[tools/livox_txt_to_pcd.py](tools/livox_txt_to_pcd.py) converts the text dump
produced by the Livox-SDK2 sample recorder (`scan_NNNNNN_<ms>ms.txt` +
`imu_data.txt`) into the layout above:

```
    python3 tools/livox_txt_to_pcd.py --in ~/Livox-SDK2/my_slam_data --out /tmp/mid360
    build/fastlio_offline --config config/mid360_txt.yaml \
                          --pcd_dir /tmp/mid360/pcd --imu /tmp/mid360/imu.txt \
                          --traj Log/traj.txt --map PCD/scans.pcd
```

It writes `velodyne_ros::Point`-layout PCDs (so `lidar_type: 2`, per-point times
preserved for motion undistortion) and fixes two things in the dump:

- **Per-point timestamps.** Some recorder builds write each hardware packet's
  `time_interval` as the *per-point* step instead of dividing by the 96 points in
  a packet, so recorded offsets span ~145 ms per 100 ms frame and jump backwards
  at every packet boundary. The converter rescales those frames, which brings
  the span to 100.02 ms against the 100.09 ms the frame header declares.
  Later recorder builds already write correct per-point timestamps (zero
  backwards jumps, span matching the declared duration) — those frames are
  passed through untouched: rescaling them would collapse a 100 ms frame to
  under a millisecond and silently destroy motion undistortion and IMU
  windowing. The guard is automatic; `--no-fix-time` forces raw values for
  every frame.
- **Acceleration units.** The IMU header claims m/s² but MID-360 reports g (the
  static magnitude is 0.9964). The converter multiplies by 9.81; use
  `--acc-scale 1.0` if your dump is already in m/s².

### 4.2 Recovering the LiDAR-to-body extrinsic

[tools/estimate_body_extrinsic.py](tools/estimate_body_extrinsic.py) estimates
how a LiDAR is mounted relative to the robot it rides on, from a finished run's
trajectory, the IMU log and the scans:

```
    python3 tools/estimate_body_extrinsic.py --imu /tmp/mid360/imu.txt \
            --traj PCD/mid360_traj.txt --pcd_dir /tmp/mid360/pcd
```

Three of the six degrees of freedom are recoverable, and the tool is explicit
about which:

| DoF | Source | Strength |
| --- | --- | --- |
| roll, pitch | accelerometer mean over a stationary stretch gives body **+z**; the RANSAC ground-plane normal confirms it independently | strong — the two agree to ~0.5° |
| yaw | direction of travel, **assuming** the robot walks along its own +x | weak — an assumption, not a measurement (~1° bootstrap spread) |
| translation | — | not recoverable; measure it off the robot or the CAD model |

On the MID-360 capture above this gives a tilt of **47.1°** off vertical and
`roll = -0.3°, pitch = -47.1°, yaw = +3.5°` for the LiDAR-from-body rotation.
The roll/pitch pair repeats to ~0.05°; yaw moves by about a degree between runs,
which is the honest size of its uncertainty.

This is a *different* extrinsic from `mapping/extrinsic_R`, which relates the
LiDAR to its own built-in IMU — identity here, since both sit in the same
housing and tilt together.

### 4.3 Converting a ROS bag export

[tools/ros_csv_to_offline.py](tools/ros_csv_to_offline.py) converts a dataset
exported from a rosbag — an `imu_data.csv` (the `sensor_msgs/Imu` columns;
the orientation columns are optional and ignored either way) plus
a `pcd/` folder of per-frame ASCII `PointXYZI` clouds named
`frame_NNNNNN_<unix_seconds>.pcd`:

```
    python3 tools/ros_csv_to_offline.py --in /path/to/export --out /tmp/ros_dataset
    build/fastlio_offline --config config/mid360_txt.yaml \
                          --pcd_dir /tmp/ros_dataset/pcd --imu /tmp/ros_dataset/imu.txt \
                          --traj Log/traj.txt --map PCD/ros_map.pcd
```

It writes the `velodyne_ros::Point` layout (`lidar_type: 2`) and handles three
things the export dropped:

- **Acceleration units.** The CSV says m/s² but the median static norm says
  which it really is: ~1 means g (multiplied by 9.81), ~9.81 means m/s².
  `--acc-scale` overrides the auto-detection.
- **Per-point times.** A bag export has none. Raw MID-360 clouds are still in
  capture order — the non-repetitive rosette streams continuously and each
  ~113 ms file is one chunk of it — so a uniform spread over the frame
  interval reconstructs each point's capture time to within a few point steps,
  which is all the undistortion needs. `--span` forces a constant interval;
  `--stamp-shift` corrects exporters that stamped frame *ends*.
- **Raw vs. registered clouds.** The clouds must be the **raw sensor-frame
  topic**. If an upstream SLAM or odometry node already transformed them into
  a fixed world frame before they were saved, no LIO can run on the pair: the
  IMU measures the sensor's real motion while the clouds sit still, and the
  filter reconciles the conflict by inventing ghost velocity until the
  watchdog recovers, over and over. The converter screens for exactly that
  signature — consecutive clouds staying aligned (3 cm cell overlap) through
  windows where the gyro reports fast rotation — and fails loudly instead of
  producing a diverging run. If it fires, re-export the raw topic (ideally
  with per-point times, e.g. the `livox_ros_driver2/CustomMsg`); if the
  registered clouds are all you have, they are an already-built map —
  accumulate them at identity instead of running LIO.

## 5. Using the library

Include [fast_lio.h](include/fast_lio.h), push data in as it arrives, and call
`processOnce()` to run the filter whenever a full scan's worth of IMU data has
accumulated:

```cpp
#include <fast_lio.h>

FastLIO lio(FastLIOConfig::fromYamlFile("config/velodyne.yaml"));

lio.setStateCallback([](const LioState &s) {
  printf("%.6f %.4f %.4f %.4f\n", s.timestamp, s.pos.x(), s.pos.y(), s.pos.z());
});
lio.setCloudCallback([](const PointCloudXYZI::Ptr &cloud, double stamp) {
  // registered scan in the world frame
});

// from your IMU driver, at IMU rate:
ImuData imu;
imu.timestamp = t;                          // seconds
imu.angular_velocity = {wx, wy, wz};        // rad/s, body frame
imu.linear_acceleration = {ax, ay, az};     // m/s^2
lio.addImu(imu);

// from your LiDAR driver, once per scan:
lio.addLidar(cloud, scan_start_time);       // pcl::PCLPointCloud2
// ...or, for Livox:
lio.addLidar(livox_msg);                    // LivoxCustomMsg

// drain: returns false when no complete measurement group is ready yet
while (lio.processOnce()) {}
```

`FastLIO` buffers input under its own mutex, so `addImu`/`addLidar` may be called
from the sensor threads while `processOnce()` runs elsewhere. Callbacks fire on
the thread that calls `processOnce()`.

[examples/offline_player.cpp](examples/offline_player.cpp) is a complete working
example of this API.

## 6. Datasets

The upstream datasets are distributed as rosbags, so you will need to convert
them to the PCD-directory + IMU-text layout above (or write a small reader
against the library API).

### 6.1 Livox Avia
<div align="left">
<img src="doc/results/HKU_LG_Indoor.png" width=47% />
<img src="doc/results/HKU_MB_002.png" width = 51% >
</div>

Files: Can be downloaded from [google drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)

### 6.2 Velodyne HDL-32E

**NCLT Dataset**: Original bin file can be found [here](http://robots.engin.umich.edu/nclt/), along with upstream's [converted files](https://drive.google.com/drive/folders/1blQJuAB4S80NwZmpM6oALyHWvBljPSOE?usp=sharing).

## 7.Implementation on UAV
In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a small-scale quadrotor which can carry a Livox Avia LiDAR with 70 degree FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz Intel i7-8550U CPU and 8 G RAM, as shown in below.

The main structure of this UAV is 3d printed (Aluminum or PLA), the .stl file will be open-sourced in the future.

<div align="center">
    <img src="doc/uav01.jpg" width=40.5% >
    <img src="doc/uav_system.png" width=57% >
</div>

## 8.Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
