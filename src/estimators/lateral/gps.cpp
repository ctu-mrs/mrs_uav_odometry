#define VERSION "0.0.5.1"

/* includes //{ */

#include "estimators/lateral/gps.h"

//}

namespace mrs_odometry
{

/* initialize() //{*/
void Gps::initialize(const ros::NodeHandle &parent_nh) {

  nh_ = parent_nh;

  // TODO load parameters

  // clang-format off
    dt_ = 0.01;

    A_ <<
      1, 0, dt_, 0, std::pow(dt_, 2)/2, 0,
      0, 1, 0, dt_, 0, std::pow(dt_, 2)/2,
      0, 0, 1, 0, dt_, 0,
      0, 0, 0, 1, 0, dt_,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;

    B_ <<
      0, 0,
      0, 0,
      0, 0,
      0, 0,
      1, 0,
      0, 1;

    H_ <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0;

    Q_ <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1;

    R_ <<
      1, 0,
      0, 1;

  // clang-format on

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e6 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  sc_ = sc0;

  std::make_unique<lkf_t>(A_, B_, H_);

  // | ------------------ timers initialization ----------------- |
  _update_timer_rate_ = 100; // TODO: parametrize
  timer_update_       = nh_.createTimer(ros::Rate(_update_timer_rate_), &Gps::timerUpdate, this, false, false); // not running after init
  _check_health_timer_rate_ = 1; // TODO: parametrize
  timer_check_health_       = nh_.createTimer(ros::Rate(_check_health_timer_rate_), &Gps::timerCheckHealth, this);


  // | --------------- subscribers initialization --------------- |
  //
  // subscriber to mavros odometry
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Gps";
  shopts.no_message_timeout = ros::Duration(0.5);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_mavros_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "mavros_odom_in");
  /* sub_mavros_odom_ = nh_.subscribe("mavros_odom_in", 1, &Gps::callbackMavrosOdom, this, ros::TransportHints().tcpNoDelay()); */

  // | ------------------ finish initialization ----------------- |
  changeState(INITIALIZED_STATE);

  ROS_INFO("[Gps]: Estimator initialized, version %s", VERSION);
}
/*//}*/

/*//{ start() */
bool Gps::start(void) {

  if (isInState(READY_STATE)) {
    timer_update_.start();
    changeState(STARTED_STATE);
    return true;

  } else {
    ROS_WARN("[%s]: Estimator must be in READY_STATE to start it", ros::this_node::getName().c_str());
    return false;

  }
}
/*//}*/

/*//{ pause() */
bool Gps::pause(void) {
  
  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;

  }
}
/*//}*/

bool reset(void) = 0;

/* timerUpdate() //{*/
void Gps::timerUpdate(const ros::TimerEvent &event) {

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
    ROS_ERROR("LKF failed: %s", e.what());
  }

  if (sh_mavros_odom_.hasNewMsg()) {

    z_t                z               = z_t::Zero();
    nav_msgs::Odometry mavros_odom_msg = sh_mavros_odom_.getMsg();

    z(0) = mavros_odom_msg.pose.pose.position.x;
    z(1) = mavros_odom_msg.pose.pose.position.y;

    try {
      // Apply the correction step
      {
        std::scoped_lock lock(mutex_lkf_);
        sc_ = lkf_->correct(sc_, z, R_);
      }
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("LKF failed: %s", e.what());
    }
  }
}
/*//}*/

/*//{ timerCheckHealth() */
void Gps::timerCheckHealth(const ros::TimerEvent &event) {
  
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
void Gps::timeoutMavrosOdom(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
    {
      ROS_ERROR_STREAM("[" << getName().c_str() << "]: Estimator has not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
      
      if ((ros::Time::now()-last_msg).toSec() > _critical_timeout_mavros_odom_) {
        ROS_ERROR("[%s]: Estimator not healthy", getName().c_str());
      changeState(ERROR_STATE);
      }
    }
/*//}*/

/*//{ isConverged() */
bool Gps::isConverged() {
  
  // TODO: check convergence by rate of change of determinant
  
  return true;
  
}
/*//}*/

/*//{ callbackMavrosOdom() */
/* void Gps::callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg) { */

/*   if (got_odom_mavros_) { */

/*     odom_mavros_previous_    = odom_mavros_; */
/*     odom_mavros_             = *msg; */
/*     odom_mavros_last_update_ = ros::Time::now(); */

/*   } else { */


/*     odom_mavros_previous_ = *msg; */
/*     odom_mavros_          = *msg; */

/*     got_odom_mavros_         = true; */
/*     odom_mavros_last_update_ = ros::Time::now(); */
/*     return; */
/*   } */


/*   if (!isTimestampOK(odom_mavros_.header.stamp.toSec(), odom_mavros_previous_.header.stamp.toSec())) { */
/*     ROS_DEBUG_THROTTLE(1.0, "[Gps]: Mavros odom timestamp not OK, not fusing correction."); */
/*     return; */
/*   } */

/*   /1* //{ fuse mavros position *1/ */


/*   // Apply correction step to all state estimators */
/*   stateEstimatorsCorrection(pos_mavros_x, pos_mavros_y, "pos_mavros"); */
/*   /1* ROS_INFO("[Odometry]: Fusing mavros x pos: %f", pos_mavros_x); *1/ */

/*   ROS_WARN_ONCE("[Odometry]: Fusing mavros position"); */
/*   //} */

/* } */
/*//}*/

/*//{ getName() */
std::string Gps::getName(void) {
  return _name_;
}
/*//}*/

/*//{ getState() */
State_t Gps::getState(const StateId &state_id_in) {

  State_t state_out;
  {
    std::scoped_lock lock(mutex_lkf_);
    state_out(0) = sc_.x(stateIdToIndex(state_id_in, AXIS_X));
    state_out(1) = sc_.x(stateIdToIndex(state_id_in, AXIS_Y));
  }
  return state_out;
}
/*//}*/

/*//{ setState() */
void Gps::setState(const State_t &state_in, const StateId &state_id_in) {
  {
    std::scoped_lock lock(mutex_lkf_);
    sc_.x(stateIdToIndex(state_id_in, AXIS_X)) = state_in(AXIS_X);
    sc_.x(stateIdToIndex(state_id_in, AXIS_Y)) = state_in(AXIS_Y);
  }
}
/*//}*/

/*//{ getStates() */
States_t Gps::getStates(void) {

  States_t states_out;
  {
    std::scoped_lock lock(mutex_lkf_);
    states_out(AXIS_X, POSITION)     = sc_.x(stateIdToIndex(AXIS_X, POSITION));
    states_out(AXIS_Y, POSITION)     = sc_.x(stateIdToIndex(AXIS_Y, POSITION));
    states_out(AXIS_X, VELOCITY)     = sc_.x(stateIdToIndex(AXIS_X, VELOCITY));
    states_out(AXIS_Y, VELOCITY)     = sc_.x(stateIdToIndex(AXIS_Y, VELOCITY));
    states_out(AXIS_X, ACCELERATION) = sc_.x(stateIdToIndex(AXIS_X, ACCELERATION));
    states_out(AXIS_Y, ACCELERATION) = sc_.x(stateIdToIndex(AXIS_Y, ACCELERATION));
  }
  return state_out;
}
/*//}*/

/*//{ setStates() */
void Gps::setStates(const States &states_in) {

  {
    std::scoped_lock lock(mutex_lkf_);
    sc_.x(stateIdToIndex(AXIS_X, POSITION))     = states_in(AXIS_X, POSITION);
    sc_.x(stateIdToIndex(AXIS_Y, POSITION))     = states_in(AXIS_Y, POSITION);
    sc_.x(stateIdToIndex(AXIS_X, VELOCITY))     = states_in(AXIS_X, VELOCITY);
    sc_.x(stateIdToIndex(AXIS_Y, VELOCITY))     = states_in(AXIS_Y, VELOCITY);
    sc_.x(stateIdToIndex(AXIS_X, ACCELERATION)) = states_in(AXIS_X, ACCELERATION);
    sc_.x(stateIdToIndex(AXIS_Y, ACCELERATION)) = states_in(AXIS_Y, ACCELERATION);
  }
}
/*//}*/

/* void setInput(const Input &input_in)                                                             = 0; */
/* void setMeasurement(const Measurement &measurement_in, const MeasurementId_t &measurement_id_in) = 0; */

/* ProcessNoiseMatrix getProcessNoise()                                           = 0; */
/* void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0; */
/* double             getMeasurementNoise(void)                                   = 0; */
/* void               setMeasurementNoise(double covariance)                      = 0; */

/*//{ stateIdToIndex() */
int Gps::stateIdToIndex(const Axis_t &axis_in, const StateId &state_id_in) {
  return state_id_in * _n_axes_ + axis_in;
}
/*//}*/

};  // namespace mrs_odometry

#endif
