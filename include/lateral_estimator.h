#ifndef LATERALESTIMATOR_H_
#define LATERALESTIMATOR_H_

/* includes //{ */

/* #include <ros/ros.h> */

/* #include <Eigen/Dense> */

#include "estimator.h"

//}

namespace mrs_odometry
{

namespace lateral
{
const std::string type = "LATERAL";
}

template <int n_states>
class LateralEstimator : public Estimator<n_states, 2> {

private:
public:
  LateralEstimator(const std::string& name, const std::string& frame_id) : Estimator<n_states, 2>(lateral::type, name, frame_id){};

  virtual ~LateralEstimator(void) {
  }

private:
  static const int _n_axes_   = 2;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

};

}  // namespace mrs_odometry

#endif
