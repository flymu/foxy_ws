#ifndef LIO_LOG_H
#define LIO_LOG_H

#include <cstdio>
#include <cstdlib>

// Minimal replacements for the ROS logging macros. Messages are written to
// stderr so they stay separated from the algorithm's stdout timing reports.
#define LIO_INFO(...)  do { fprintf(stderr, "[ INFO] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LIO_WARN(...)  do { fprintf(stderr, "[ WARN] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LIO_ERROR(...) do { fprintf(stderr, "[ERROR] " __VA_ARGS__); fputc('\n', stderr); } while (0)

#define LIO_ASSERT(cond)                                                      \
  do {                                                                        \
    if (!(cond)) {                                                            \
      fprintf(stderr, "[ERROR] assertion failed: %s (%s:%d)\n", #cond,        \
              __FILE__, __LINE__);                                            \
      std::abort();                                                           \
    }                                                                         \
  } while (0)

#endif  // LIO_LOG_H
