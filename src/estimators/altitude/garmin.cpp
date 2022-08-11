#define VERSION "0.0.6.0"
/* includes //{ */

#include "estimators/altitude/garmin.h"

//}

namespace mrs_odometry

{

/* initialize() //{*/
void Garmin::initialize(const ros::NodeHandle &parent_nh) {

  nh_ = parent_nh;

  // TODO load parameters

  // clang-format off
    dt_ = 0.01;

    A_ <<
      1, dt_,
      0, 1;

    B_ <<
      0,
      1;

    H_ <<
      1, 0;
      /* 0, 1; */

    Q_ <<
      1, 0,
      0, 1;

    R_ <<
      0.1;

  // clang-format on

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_       = 100;                                                                                     // TODO: parametrize
  timer_update_             = nh_.createTimer(ros::Rate(_update_timer_rate_), &Garmin::timerUpdate, this, false, false);  // not running after init
  _check_health_timer_rate_ = 1;                                                                                       // TODO: parametrize
  timer_check_health_       = nh_.createTimer(ros::Rate(_check_health_timer_rate_), &Garmin::timerCheckHealth, this);

  _critical_timeout_mavros_odom_ = 1.0;  // TODO: parametrize
  _critical_timeout_garmin_range_ = 1.0;  // TODO: parametrize


  // | --------------- subscribers initialization --------------- |
  
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = getName();
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in");
  sh_garmin_range_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "garmin_range_in");

  // | ---------------- publishers initialization --------------- |
  pub_output_      = nh_.advertise<mrs_odometry::EstimatorOutput>(getName() + "/output", 1);
  pub_diagnostics_ = nh_.advertise<mrs_odometry::EstimatorDiagnostics>(getName() + "/diagnostics", 1);

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    ROS_INFO("[%s]: Estimator initialized, version %s", getName().c_str(), VERSION);
  } else {
    ROS_INFO("[%s]: Estimator could not be initialized", getName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool Garmin::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", getName().c_str());
    return false;
  }
}
/*//}*/

/*//{ pause() */
bool Garmin::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool Garmin::reset(void) {

  if (!isInitialized()) {
    ROS_ERROR("[%s]: Cannot reset uninitialized estimator", getName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  // Initialize LKF state and covariance
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  // Instantiate the LKF itself
  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  ROS_INFO("[%s]: Estimator reset", getName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void Garmin::timerUpdate(const ros::TimerEvent &event) {


  if (!isInitialized()) {
    return;
  }

  // TODO input and measurement from actual data

  // prediction step
  u_t u = u_t::Zero();

  try {
    // Apply the prediction step
    {
      std::scoped_lock lock(mutex_lkf_);
      sc_ = lkf_->predict(sc_, u, Q_, dt_);
    }
  }
  catch (const std::exception &e) {
    // In case of error, alert the user
    ROS_ERROR("[%s]: LKF prediction failed: %s", getName().c_str(), e.what());
  }

  if (sh_mavros_odom_.newMsg()) {

    z_t z = z_t::Zero();

    nav_msgs::Odometry::ConstPtr mavros_odom_msg = sh_mavros_odom_.getMsg();

    /* z(0) = mavros_odom_msg->twist.twist.linear.z; */
    z(0) = mavros_odom_msg->pose.pose.position.z;

    try {
      // Apply the correction step
      {
        std::scoped_lock lock(mutex_lkf_);
        sc_ = lkf_->correct(sc_, z, R_);
      }
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
    ROS_ERROR("[%s]: LKF correction failed: %s", getName().c_str(), e.what());
    }
  }

  publishOutput();
  publishDiagnostics();
}
/*//}*/

/*//{ timerCheckHealth() */
void Garmin::timerCheckHealth(const ros::TimerEvent &event) {

  if (!isInitialized()) {
    return;
  }

  if (isInState(INITIALIZED_STATE)) {

    if (sh_mavros_odom_.hasMsg()) {
      changeState(READY_STATE);
      ROS_INFO("[%s]: Estimator is ready to start", getName().c_str());
    }
  }

  if (isInState(STARTED_STATE)) {
    ROS_INFO("[%s]: Estimator is waiting for convergence of LKF", getName().c_str());

    if (isConverged()) {
      ROS_INFO("[%s]: LKF converged", getName().c_str());
      changeState(RUNNING_STATE);
    }
  }
}
/*//}*/

/*//{ timeoutMavrosOdom() */
void Garmin::timeoutMavrosOdom(const std::string &topic, const ros::Time &last_msg, const int n_pubs) {
  ROS_ERROR_STREAM("[" << getName().c_str() << "]: Estimator has not received message from topic '" << topic << "' for "
                       << (ros::Time::now() - last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");

  if ((ros::Time::now() - last_msg).toSec() > _critical_timeout_mavros_odom_) {
    ROS_ERROR("[%s]: Estimator not healthy", getName().c_str());
    changeState(ERROR_STATE);
  }
}
/*//}*/

/*//{ isConverged() */
bool Garmin::isConverged() {

  // TODO: check convergence by rate of change of determinant

  return true;
}
/*//}*/

/*//{ getState() */
double Garmin::getState(const int &state_id_in, const int& axis_in) const {

  return getState(stateIdToIndex(state_id_in, axis_in));
}

double Garmin::getState(const int &state_idx_in) const {

  std::scoped_lock lock(mutex_lkf_);
  return sc_.x(state_idx_in);
}
/*//}*/

/*//{ setState() */
void Garmin::setState(const double &state_in, const int &state_id_in, const int &axis_in) {
    setState(state_in, stateIdToIndex(state_id_in, axis_in));
}

void Garmin::setState(const double &state_in, const int &state_idx_in) {
    std::scoped_lock lock(mutex_lkf_);
    sc_.x(state_idx_in) = state_in;
}
/*//}*/

/*//{ getStates() */
Garmin::states_t Garmin::getStates(void) const {
  std::scoped_lock lock(mutex_lkf_);
  return sc_.x;
}
/*//}*/

/*//{ setStates() */
void Garmin::setStates(const states_t &states_in) {
    std::scoped_lock lock(mutex_lkf_);
    sc_.x = states_in;
}
/*//}*/

/*//{ getCovariance() */
Garmin::covariance_t Garmin::getCovariance(void) const {
    std::scoped_lock lock(mutex_lkf_);
    return sc_.P;
}
/*//}*/

/*//{ setCovariance() */
void Garmin::setCovariance(const covariance_t& cov_in) {
    std::scoped_lock lock(mutex_lkf_);
    sc_.P = cov_in;
}
/*//}*/
};  // namespace mrs_odometry
