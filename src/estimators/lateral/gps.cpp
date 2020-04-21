#define VERSION "0.0.5.1"

/* includes //{ */

#include "estimators/lateral/gps.h"

//}

namespace mrs_odometry
{

/* initialize() //{*/
  void Gps::initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string uav_name) {

    nh_ = parent_nh;
    name_ = name;
    uav_name_ = uav_name;

    //TODO load parameters
    
    dt_ = 0.01;

    A_ << 
      1, 0, dt_, 0, dt_^2/2, 0,
      0, 1, 0, dt_, 0, dt_^2/2, 
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

          // | --------------- Kalman filter intialization -------------- |
    const x_t x0 = x_t::Zero();
    const P_t P0 = 1000 * P_t::Identity();
    const statecov_t sc0({x0, P0});
    sc_ = sc0;

    std::make_unique<lkf_t>(A_, B_, H_);

    // | ------------------ timers initialization ----------------- |
    _predict_timer_rate_ = 100;
    timer_predict_    = nh_.createTimer(ros::Rate(_predict_timer_rate_), &Gps::timerPredict, this, false, false);

    // | --------------- subscribers initialization --------------- |
    //
  // subscriber to mavros odometry
  sub_mavros_odom_ = nh_.subscribe("mavros_odom_in", 1, &Gps::callbackMavrosOdom, this, ros::TransportHints().tcpNoDelay());

    // | ------------------ finish initialization ----------------- |
    changeState(INITIALIZED_STATE);

    is_initialized_ = true;

    ROS_INFO("[Gps]: initialized, version %s", VERSION);
  }
/*//}*/

  bool start(void)                                                              = 0;
  bool pause(void)                                                              = 0;
  bool reset(void)                                                              = 0;

/* timerPredict() //{*/
  void Gps::timerPredict(const ros::TimerEvent &event) {

    // TODO input and measurement from actual data
    u_t u = u_t::Zero();
    z_t z = z_t::Zero();

    try
    {
      // Apply the prediction step
      sc_ = lkf_->predict(sc_, u, Q_, dt_);
      
      // Apply the correction step
      sc_ = lkf_->correct(sc_, z, R_);
    }
    catch (const std::exception& e)
    {
      // In case of error, alert the user
      ROS_ERROR("LKF failed: %s", e.what());
    }
  }
/*//}*/

  void Gps::callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg) {

    if (!is_initialized_)
      return;
      
  if (got_odom_mavros_) {

      odom_mavros_previous_         = odom_mavros_;
      odom_mavros_                  = *msg;
      odom_mavros_last_update_ = ros::Time::now();

  } else {


      odom_mavros_previous_ = *msg;
      odom_mavros_          = *msg;

    got_odom_mavros_         = true;
    odom_mavros_last_update_ = ros::Time::now();
    return;
  }


  if (!isTimestampOK(odom_mavros_.header.stamp.toSec(), odom_mavros_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Gps]: Mavros odom timestamp not OK, not fusing correction.");
    return;
  }

  /* //{ fuse mavros position */


    // Apply correction step to all state estimators
    stateEstimatorsCorrection(pos_mavros_x, pos_mavros_y, "pos_mavros");
    /* ROS_INFO("[Odometry]: Fusing mavros x pos: %f", pos_mavros_x); */

    ROS_WARN_ONCE("[Odometry]: Fusing mavros position");
    //}

    //}
  }

  std::string getName(void) = 0;

  State getState(const StateId &state_id_in) = 0;
  void  setState(const State &state_in)      = 0;

  States getStates(void)                    = 0;
  void   setStates(const States &states_in) = 0;

  void setInput(const Input &input_in)                                                           = 0;
  void setMeasurement(const Measurement &measurement_in, const MeasurementId_t &measurement_id_in) = 0;

  ProcessNoiseMatrix getProcessNoise()                                           = 0;
  void               setProcessNoise(const ProcessNoiseMatrix &process_noise_in) = 0;
  double             getMeasurementNoise(void)                                   = 0;
  void               setMeasurementNoise(double covariance)                      = 0;
};
}  // namespace mrs_odometry

#endif
