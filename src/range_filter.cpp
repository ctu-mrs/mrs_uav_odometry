/* author: Daniel Hert */

#include <vector>
#include <ros/ros.h>
#include <range_filter.h>

RangeFilter::RangeFilter(int buffer_size, double trg_max_valid_altitude, double trg_filter_max_difference) {

  // maximum teraranger altitude that is considered valid
  this->trg_max_valid_altitude = trg_max_valid_altitude;

  // if new value is more different from the filter median than this value, it is classified as invalid
  this->trg_filter_max_difference = trg_filter_max_difference;

  this->buffer_size = buffer_size;

  buffer.resize(buffer_size);
  next = 0;
  ROS_INFO("[RangeFilter]: initialized, buffer size: %d", buffer_size);
}

double RangeFilter::getValue(double input, ros::Duration interval) {

  double range = input;

  // add new teraranger altitude to the filter buffer
  buffer[next] = range;
  next++;

  if (next == buffer_size) {
    next = 0;
  }

  // check if new teraranger altitude is valid
  if (!isValid(input, interval)) {
    return 0;
  }

  int    current_value  = 0;
  int    best_med_value = 9999;
  int    zero_counter   = 0;
  double ret_val        = 0;
  double median         = 0;

  // calculate the median from the filter buffer
  for (int i = 0; i < buffer_size; i++) {
    if (buffer[i] == 0) {
      zero_counter++;
    }
  }

  // if there is more zeroes in the buffer than half of the buffer size, the median value is automatically zero
  if (zero_counter < buffer_size / 2) {

    for (int i = 0; i < buffer_size; i++) {

      current_value = 0;

      if (buffer[i] == 0) {
        continue;
      }

      for (int j = 0; j < buffer_size; j++) {

        if (i == j) {
          continue;
        }

        if (buffer[i] > buffer[j]) {
          current_value++;
        } else if (buffer[i] < buffer[j]) {
          current_value--;
        }
      }

      if (abs(current_value) < best_med_value) {
        best_med_value = abs(current_value);
        median         = buffer[i];
      }
    }
  } else {
    median = 0;
  }

  // if a new value is not close to the median, it is discarted
  if (fabs(range - median) > trg_filter_max_difference) {
    ret_val = 0;
  } else {
    ret_val = range;
  }

  // check whether the output is finite
  if (std::isfinite(ret_val)) {
    return ret_val;
  } else {

    ROS_WARN("[RangeFilter]: received range is not a finite number!");
    return 0;
  }
}

// checks if teraranger altitude is a valid value
bool RangeFilter::isValid(double input, ros::Duration interval) {

  if ((input < trg_max_valid_altitude) && (input > 0) && (interval.toSec() < 1)) {

    return true;

  } else {

    return false;
  }
}
