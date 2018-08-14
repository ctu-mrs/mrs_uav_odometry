#ifndef TRG_FILTER
#define TRG_FILTER

/* author: Daniel Hert */

#include <vector>
#include <ros/ros.h>

class RangeFilter {

public:
  RangeFilter(int buffer_size, double trg_max_valid_altitude, double trg_filter_max_difference);
  double getValue(double input, ros::Duration interval);

private:
  bool                isValid(double input, ros::Duration interval);
  std::vector<double> buffer;
  int                 buffer_size;
  int                 next;
  double              trg_max_valid_altitude;
  double              trg_filter_max_difference;
};

#endif
