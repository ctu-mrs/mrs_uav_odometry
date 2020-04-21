#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <std_msgs/Header.h>

namespace mrs_odometry
{

typedef enum
{

  POS_MAVROS,
  POS_HECTOR,
  VEL_MAVROS,
  VEL_OPTFLOW,
  TILT_MAVROS

} LateralMeasurementId_t;

struct LateralMeasurement
{

  std_msgs::Header header;

  LateralMeasurementId_t meas_id;

  double x;
  double y;

  double cov_x;
  double cov_y;

  LateralMeasurement(const std_msgs::Header& header, const LateralMeasurementId_t& meas_id, const double& x, const double& y, const double& cov_x,
                     const double& cov_y)
      : header(header), meas_id(meas_id), x(x), y(y), cov_x(cov_x), cov_y(cov_y){};

  LateralMeasurement(const std_msgs::Header& header, const LateralMeasurementId_t& meas_id, const double& x, const double& y)
      : LateralMeasurement(header, meas_id, x, y, 0, 0){};

};

}  // namespace mrs_odometry

#endif
