#ifndef GPS_H_
#define GPS_H_

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>

#include "lateral_estimator.h"

//}

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

  //}

private:
  ros::NodeHandle nh_;
  std::string     name_;
  std::string     uav_name_;

  bool is_initialized_ = false;;

  double dt_;
  A_t    A_;
  B_t    B_;
  H_t    H_;
  Q_t    Q_;
  R_t    R_;
  statecov_t sc_;
  std::unique_ptr<lkf_t> lkf_;

  ros::Subscriber sub_mavros_odom_;

  ros::Timer timer_predict_;
  int _predict_timer_rate_;
  void       timerPredict(const ros::TimerEvent &event);

public:
  static const int n_states       = N_STATES;
  static const int n_inputs       = N_INPUTS;
  static const int n_measurements = N_MEASUREMENTS;

  typedef Eigen::Matrix<double, 2, 1>               State_t;
  typedef Eigen::Matrix<double, 2, n_states>        States_t;
  typedef Eigen::Matrix<double, 2, 1>               Input_t;
  typedef Eigen::Matrix<double, n_states, n_states> ProcessNoiseMatrix_t;
  typedef LateralMeasurement                        Measurement_t;

public:
  ~Gps(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string& name, const std::string& uav_name);
  virtual bool start(void) const override;
  virtual bool pause(void) const override;
  virtual bool reset(void) const override;

  virtual std::string getName(void) const override;

  virtual State_t getState(const StateId_t &state_id_in) const override;
  virtual void    setState(const State_t &state_in) const override;

  virtual States_t getStates(void) const override;
  virtual void     setStates(const States_t &states_in) const override;

  virtual void setInput(const Input_t &input_in) const override;
  virtual void setMeasurement(const Measurement_t &measurement_in, const MeasurementId_t &measurement_id_in) const override;

  virtual ProcessNoiseMatrix_t getProcessNoise() const override;
  virtual void                 setProcessNoise(const ProcessNoiseMatrix_t &process_noise_in) const override;
  virtual double               getMeasurementNoise(void) const override;
  virtual void                 setMeasurementNoise(double covariance) const override;

  void callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg);
  nav_msgs::Odometry odom_mavros_, odom_mavros_previous_;
  bool got_odom_mavros_;
  ros::Time odom_mavros_last_update_;
};
}  // namespace mrs_odometry

#endif
