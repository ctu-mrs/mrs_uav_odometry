#ifndef ALTITUDEESTIMATOR_H_
#define ALTITUDESTIMATOR_H_

/* includes //{ */

/* #include <ros/ros.h> */

/* #include <Eigen/Dense> */

/* #include <mrs_odometry/EstimatorDiagnostics.h> */
/* #include <mrs_odometry/EstimatorOutput.h> */

#include "estimator.h"

//}

namespace mrs_odometry
{

namespace altitude
{
const std::string type = "ALTITUDE";
}

template <int n_states>
class AltitudeEstimator : public Estimator<n_states, 1> {

private:
public:
  AltitudeEstimator(const std::string& name, const std::string& frame_id) : Estimator<n_states, 1>(altitude::type, name, frame_id){};

  virtual ~AltitudeEstimator(void) {
  }


private:
  static const int _n_axes_   = 1;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

};

}  // namespace mrs_odometry

#endif
