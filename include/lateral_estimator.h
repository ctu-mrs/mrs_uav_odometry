#ifndef LATERALESTIMATOR_H_
#define LATERALESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <lateral_measurement.h>

//}

namespace mrs_odometry
{

typedef enum
{

  UNINITIALIZED_STATE,
  INITIALIZED_STATE,
  READY_STATE,
  STARTED_STATE,
  RUNNING_STATE,
  STOPPED_STATE,
  ERROR_STATE

} SMStates_t;

template <int n_states, int n_axes>
class LateralEstimator {

public:
  virtual ~LateralEstimator(void) {
  }

public:

  typedef Eigen::Matrix<double, n_axes, 1>          state_t;
  typedef Eigen::Matrix<double, n_axes, n_states>   states_t;

private:

  static const int _n_axes_ = n_axes;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

  // clang-format off
  const std::vector<std::string> _sm_state_names_ = {
    "UNINITIALIZED_STATE",
    "UNINITIALIZED_STATE",
    "INITIALIZED_STATE",
    "READY_STATE",
    "STARTED_STATE",
    "RUNNING_STATE",
    "STOPPED_STATE",
    "ERROR_STATE"};
  // clang-format on

public:
  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string &name) = 0;
  virtual bool start(void)                                                           = 0;
  virtual bool pause(void)                                                           = 0;
  virtual bool reset(void)                                                           = 0;

  virtual std::string getName(void) const = 0;

  virtual state_t getState(const int &state_id_in) = 0;
  virtual void  setState(const state_t &state_in, const int &state_id_in)      = 0;

  virtual states_t getStates(void)                   = 0;
  virtual void   setStates(const states_t &states_in) = 0;

  /* virtual void setInput(const Input &input_in)                   = 0; */
  /* virtual void setMeasurement(const Measurement &measurement_in) = 0; */

  /* virtual ProcessNoiseMatrix getProcessNoise()                                           = 0; */
  /* virtual void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0; */

  /* virtual double getMeasurementNoise(void)              = 0; */
  /* virtual void   setMeasurementNoise(double covariance) = 0; */

  // implemented
  /*//{ changeState() */
  void changeState(SMStates_t new_state) {

    previous_sm_state_ = current_sm_state_;
    current_sm_state_  = new_state;

    switch (current_sm_state_) {
      case UNINITIALIZED_STATE:
        break;
      case INITIALIZED_STATE:
        break;
      case READY_STATE:
        break;
      case RUNNING_STATE:
        break;
      case STOPPED_STATE:
        break;
    }

    ROS_INFO("[%s]: Switching sm state %s -> %s", getName().c_str(), _sm_state_names_[previous_sm_state_], _sm_state_names_[current_sm_state_]);
  }
  /*//}*/

  /*//{ isInState() */
  bool isInState(const SMStates_t &state_in) const {
    return state_in == current_sm_state_;
  }
  /*//}*/

  /*//{ isInitialized() */
  bool isInitialized() const {
    return current_sm_state_ != UNINITIALIZED_STATE;
  }
  /*//}*/

  /*//{ isReady() */
  bool isReady() const {
    return current_sm_state_ == READY_STATE;
  }
  /*//}*/

  /*//{ isRunning() */
  bool isRunning() const {
    return current_sm_state_ == RUNNING_STATE;
  }
  /*//}*/

  /*//{ isStopped() */
  bool isStopped() const {
    return current_sm_state_ == STOPPED_STATE;
  }
  /*//}*/

  /*//{ isError() */
  bool isError() const {
    return current_sm_state_ == ERROR_STATE;
  }
  /*//}*/

/*//{ stateIdToIndex() */
int stateIdToIndex(const int &axis_in, const int &state_id_in) const {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

};
}  // namespace mrs_odometry

#endif
