#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

/// Wall-clock seconds, replacing ros::Time::now().toSec() in the ROS-free
/// build. Steady clock: monotonic, so it is safe for the timing statistics.
inline double now_sec()
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

#endif
