#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

/* includes //{ */

#include <ros/ros.h>

#include <Eigen/Dense>

#include <mrs_odometry/EstimatorDiagnostics.h>
#include <mrs_odometry/EstimatorOutput.h>

//}

namespace mrs_odometry
{

/*//{ typedef */
typedef enum
{

  POSITION,
  VELOCITY,
  ACCELERATION

} StateId_t;

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
/*//}*/

template <int n_states, int n_axes>
class Estimator {

public:
  typedef Eigen::Matrix<double, n_states, 1>        states_t;
  typedef Eigen::Matrix<double, n_states, n_states> covariance_t;

protected:
  ros::Publisher pub_output_;
  ros::Publisher pub_diagnostics_;

  const std::string type_;
  const std::string name_;
  const std::string frame_id_;

private:
  static const int _n_axes_   = n_axes;
  static const int _n_states_ = n_states;
  static const int _n_inputs_;
  static const int _n_measurements_;

  SMStates_t previous_sm_state_ = UNINITIALIZED_STATE;
  SMStates_t current_sm_state_  = UNINITIALIZED_STATE;

  // clang-format off
  const std::vector<std::string> _sm_state_names_ = {
    "UNINITIALIZED_STATE",
    "INITIALIZED_STATE",
    "READY_STATE",
    "STARTED_STATE",
    "RUNNING_STATE",
    "STOPPED_STATE",
    "ERROR_STATE"
  };
  // clang-format on

public:
  Estimator(const std::string &type, const std::string &name, const std::string &frame_id) : type_(type), name_(name), frame_id_(frame_id){};

  virtual ~Estimator(void) {
  }

  // virtual methods
  virtual void initialize(const ros::NodeHandle &parent_nh) = 0;
  virtual bool start(void)                                  = 0;
  virtual bool pause(void)                                  = 0;
  virtual bool reset(void)                                  = 0;

  virtual double getState(const int &state_idx_in) const                    = 0;
  virtual double getState(const int &state_id_in, const int &axis_in) const = 0;

  virtual void setState(const double &state_in, const int &state_idx_in)                    = 0;
  virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) = 0;

  virtual states_t getStates(void) const                = 0;
  virtual void     setStates(const states_t &states_in) = 0;

  virtual covariance_t getCovariance(void) const                 = 0;
  virtual void         setCovariance(const covariance_t &cov_in) = 0;

  // implemented methods
  // access methods
  std::string         getName(void) const;
  std::string         getType(void) const;
  std::string         getFrameId(void) const;
  std::string         getSmStateString(const SMStates_t &state) const;
  std::string         getCurrentSmStateString(void) const;
  std::vector<double> getStatesAsVector(void) const;
  std::vector<double> getCovarianceAsVector(void) const;

  // state machine methods
  bool changeState(SMStates_t new_state);
  bool isInState(const SMStates_t &state_in) const;
  bool isInitialized() const;
  bool isReady() const;
  bool isRunning() const;
  bool isStopped() const;
  bool isError() const;
  int  stateIdToIndex(const int &axis_in, const int &state_id_in) const;

  void publishDiagnostics() const;
  void publishOutput() const;
};

/*//{ method implementations */
/*//{ changeState() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::changeState(SMStates_t new_state) {

  // do not initialize if the pub is empty
  ros::Publisher empty_pub;
  if (new_state == INITIALIZED_STATE) {
    if (pub_output_ == empty_pub || pub_diagnostics_ == empty_pub) {
      ROS_ERROR("[%s]: cannot transition to %s - publishers are not initialized", getName().c_str(), getSmStateString(INITIALIZED_STATE).c_str());
      return false;
    }
  }

  previous_sm_state_ = current_sm_state_;
  current_sm_state_  = new_state;

  ROS_INFO("[%s]: Switching sm state %s -> %s", getName().c_str(), getSmStateString(previous_sm_state_).c_str(), getSmStateString(current_sm_state_).c_str());
  return true;
}
/*//}*/

/*//{ isInState() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isInState(const SMStates_t &state_in) const {
  return state_in == current_sm_state_;
}
/*//}*/

/*//{ isInitialized() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isInitialized() const {
  return !isInState(UNINITIALIZED_STATE);
}
/*//}*/

/*//{ isReady() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isReady() const {
  return isInState(READY_STATE);
}
/*//}*/

/*//{ isRunning() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isRunning() const {
  return isInState(RUNNING_STATE);
}
/*//}*/

/*//{ isStopped() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isStopped() const {
  return isInState(STOPPED_STATE);
}
/*//}*/

/*//{ isError() */
template <int n_states, int n_axes>
bool Estimator<n_states, n_axes>::isError() const {
  return isInState(ERROR_STATE);
}
/*//}*/

/*//{ stateIdToIndex() */
template <int n_states, int n_axes>
int Estimator<n_states, n_axes>::stateIdToIndex(const int &axis_in, const int &state_id_in) const {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

/*//{ getSmStateString() */
template <int n_states, int n_axes>
std::string Estimator<n_states, n_axes>::getSmStateString(const SMStates_t &state) const {
  return _sm_state_names_[state];
}
/*//}*/

/*//{ getCurrentSmStateName() */
template <int n_states, int n_axes>
std::string Estimator<n_states, n_axes>::getCurrentSmStateString(void) const {
  return getSmStateString(current_sm_state_);
}
/*//}*/

/*//{ getName() */
template <int n_states, int n_axes>
std::string Estimator<n_states, n_axes>::getName(void) const {
  return name_;
}
/*//}*/

/*//{ getType() */
template <int n_states, int n_axes>
std::string Estimator<n_states, n_axes>::getType(void) const {
  return type_;
}
/*//}*/

/*//{ getFrameId() */
template <int n_states, int n_axes>
std::string Estimator<n_states, n_axes>::getFrameId(void) const {
  return frame_id_;
}
/*//}*/

/*//{ getStatesAsvector() */
template <int n_states, int n_axes>
std::vector<double> Estimator<n_states, n_axes>::getStatesAsVector(void) const {
  const states_t      states = getStates();
  std::vector<double> states_vec;
  /* for (auto st : Eigen::MatrixXd::Map(states, states.size(), 1).rowwise()) { */
  /*   states_vec.push_back(*st); */
  /* } */
  for (int i = 0; i < states.size(); i++) {
    states_vec.push_back(states(i));
  }
  return states_vec;
}
/*//}*/

/*//{ getCovarianceAsvector() */
template <int n_states, int n_axes>
std::vector<double> Estimator<n_states, n_axes>::getCovarianceAsVector(void) const {
  const covariance_t  covariance = getCovariance();
  std::vector<double> covariance_vec;
  /* for (auto cov : covariance.reshaped<Eigen::RowMajor>(covariance.size())) { */
  /*   covariance_vec.push_back(*cov); */
  /* } */
  for (int i = 0; i < covariance.rows(); i++) {
    for (int j = 0; j < covariance.cols(); j++) {
      covariance_vec.push_back(covariance(i, j));
    }
  }
  return covariance_vec;
}
/*//}*/

/*//{ publishDiagnostics() */
template <int n_states, int n_axes>
void Estimator<n_states, n_axes>::publishDiagnostics() const {

  mrs_odometry::EstimatorDiagnostics msg;
  msg.header.stamp       = ros::Time::now();
  msg.header.frame_id    = getFrameId();
  msg.estimator_name     = getName();
  msg.estimator_type     = getType();
  msg.estimator_sm_state = getCurrentSmStateString();

  try {
    pub_diagnostics_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_diagnostics_.getTopic().c_str());
  }
}
/*//}*/

/*//{ publishOutput() */
template <int n_states, int n_axes>
void Estimator<n_states, n_axes>::publishOutput() const {

  mrs_odometry::EstimatorOutput msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = getFrameId();
  msg.state           = getStatesAsVector();
  msg.covariance      = getCovarianceAsVector();

  try {
    pub_output_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_output_.getTopic().c_str());
  }
}
/*//}*/
/*//}*/

}  // namespace mrs_odometry

#endif
