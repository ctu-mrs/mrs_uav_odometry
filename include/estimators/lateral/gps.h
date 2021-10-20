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

class Gps : public LateralEstimator {

  /* typedef //{ */

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

  int stateIdToIndex(const Axis_t &axis_in, const StateId &state_id_in);

public:
  static const int _n_axes_         = N_AXES;
  static const int _n_states_       = N_STATES;
  static const int _n_inputs_       = N_INPUTS;
  static const int _n_measurements_ = N_MEASUREMENTS;

  typedef Eigen::Matrix<double, 2, 1>                   State_t;
  typedef Eigen::Matrix<double, 2, _n_states_>          States_t;
  typedef Eigen::Matrix<double, 2, 1>                   Input_t;
  typedef Eigen::Matrix<double, _n_states_, _n_states_> ProcessNoiseMatrix_t;
  typedef LateralMeasurement                            Measurement_t;

public:
  ~Gps(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string &name, const std::string &uav_name);
  virtual bool start(void) const override;
  virtual bool pause(void) const override;
  virtual bool reset(void) const override;

  virtual std::string getName(void) const override;

  bool isConverged();

  virtual State_t getState(const StateId_t &state_id_in) const override;
  virtual void    setState(const State_t &state_in) const override;

  virtual States_t getStates(void) const override;
  virtual void     setStates(const States_t &states_in) const override;

  /* virtual void setInput(const Input_t &input_in) const override; */
  /* virtual void setMeasurement(const Measurement_t &measurement_in, const MeasurementId_t &measurement_id_in) const override; */

  /* virtual ProcessNoiseMatrix_t getProcessNoise() const override; */
  /* virtual void                 setProcessNoise(const ProcessNoiseMatrix_t &process_noise_in) const override; */
  /* virtual double               getMeasurementNoise(void) const override; */
  /* virtual void                 setMeasurementNoise(double covariance) const override; */

  /* void callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg); */
  /* nav_msgs::Odometry odom_mavros_, odom_mavros_previous_; */
  /* bool got_odom_mavros_; */
  /* ros::Time odom_mavros_last_update_; */
};
}  // namespace mrs_odometry

#endif
