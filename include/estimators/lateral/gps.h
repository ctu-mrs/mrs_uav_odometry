#ifndef GPS_H_
#define GPS_H_

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "lateral_estimator.h"

//}

#define N_AXES 2
#define N_STATES 6
#define N_INPUTS 2
#define N_MEASUREMENTS 6

namespace mrs_odometry
{

using namespace mrs_lib;


  /* typedef //{ */

  /* typedef Eigen::Matrix<double, N_AXES, 1>          state_t; */
  /* typedef Eigen::Matrix<double, N_AXES, N_STATES>   states_t; */
  /* typedef Eigen::Matrix<double, N_AXES, 1>          Input_t; */
  /* typedef Eigen::Matrix<double, N_STATES, N_STATES> ProcessNoiseMatrix_t; */
  /* typedef LateralMeasurement                        Measurement_t; */

  typedef enum
  {

    POSITION,
    VELOCITY,
    ACCELERATION

  } StateId_t;

  typedef enum
  {

    AXIS_X,
    AXIS_Y,

  } Axis_t;

  //}

  
/* template <typename State_t, typename States_t, typename StateId_t, typename Axis_t> */
class Gps : public LateralEstimator<N_STATES, N_AXES> {

using lkf_t      = LKF<N_STATES, N_INPUTS, N_MEASUREMENTS>;
using A_t        = lkf_t::A_t;
using B_t        = lkf_t::B_t;
using H_t        = lkf_t::H_t;
using Q_t        = lkf_t::Q_t;
using x_t        = lkf_t::x_t;
using P_t        = lkf_t::P_t;
using u_t        = lkf_t::u_t;
using z_t        = lkf_t::z_t;
using R_t        = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

  public:
using Base_class = LateralEstimator<N_STATES, N_AXES>;
using state_t = typename Base_class::state_t; 
using states_t = typename Base_class::states_t;

private:
  ros::NodeHandle nh_;

  const std::string _name_ = "LATERAL_GPS";

  double                 dt_;
  A_t                    A_;
  B_t                    B_;
  H_t                    H_;
  Q_t                    Q_;
  R_t                    R_;
  statecov_t             sc_;
  std::unique_ptr<lkf_t> lkf_;
  std::mutex             mutex_lkf_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  /* ros::Subscriber sub_mavros_odom_; */

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  /* int stateIdToIndex(const Axis_t &axis_in, const StateId_t &state_id_in) const; */

  bool isConverged();

public:

public:
  ~Gps(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual std::string getName(void) const override;


  state_t getState(const int &state_id_in) override;
  void    setState(const state_t &state_in, const int &state_id_in) override;

  states_t getStates(void) override;
  void     setStates(const states_t &states_in) override;

  /* virtual void setInput(const Input_t &input_in) const override; */
  /* virtual void setMeasurement(const Measurement_t &measurement_in, const MeasurementId_t &measurement_id_in) const override; */

  /* virtual ProcessNoiseMatrix_t getProcessNoise() const override; */
  /* virtual void                 setProcessNoise(const ProcessNoiseMatrix_t &process_noise_in) const override; */
  /* virtual double               getMeasurementNoise(void) const override; */
  /* virtual void                 setMeasurementNoise(double covariance) const override; */

  void timeoutMavrosOdom(const std::string& topic, const ros::Time& last_msg, const int n_pubs);
  /* void callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg); */
  /* nav_msgs::Odometry odom_mavros_, odom_mavros_previous_; */
  /* bool got_odom_mavros_; */
  /* ros::Time odom_mavros_last_update_; */
};
}  // namespace mrs_odometry

#endif
