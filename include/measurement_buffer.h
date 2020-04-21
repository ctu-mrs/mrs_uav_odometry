#ifndef MEASUREMENT_BUFFER_H
#define MEASUREMENT_BUFFER_H

#include <vector>

namespace mrs_odometry
{

template <typename T>
struct MeasurementBuffer
{

  std::vector<T> data;

  MeasurementBuffer() {
    data.reserve(1000);
  }

};

}  // namespace mrs_odometry

#endif
