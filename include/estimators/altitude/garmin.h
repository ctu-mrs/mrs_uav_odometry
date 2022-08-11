#ifndef GARMIN_H_
#define GARMIN_H_

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include "altitude_estimator.h"

//}

namespace mrs_odometry
{

namespace garmin
{

const int n_states       = 2;
const int n_inputs       = 1;
const int n_measurements = 1;

const std::string name     = "altitude_garmin";
const std::string frame_id = "pixhawk_gps_origin";

}  // namespace garmin

using namespace mrs_lib;

class Garmin : public AltitudeEstimator<garmin::n_states> {

  using lkf_t      = LKF<garmin::n_states, garmin::n_inputs, garmin::n_measurements>;
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

private:
  ros::NodeHandle nh_;

  double                 dt_;
  A_t                    A_;
  B_t                    B_;
  H_t                    H_;
  Q_t                    Q_;
  R_t                    R_;
  statecov_t             sc_;
  std::unique_ptr<lkf_t> lkf_;
  mutable std::mutex     mutex_lkf_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_mavros_odom_;
  double                                        _critical_timeout_mavros_odom_;

  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_garmin_range_;
  double                                        _critical_timeout_garmin_range_;

  ros::Timer timer_update_;
  int        _update_timer_rate_;
  void       timerUpdate(const ros::TimerEvent &event);

  ros::Timer timer_check_health_;
  int        _check_health_timer_rate_;
  void       timerCheckHealth(const ros::TimerEvent &event);

  bool isConverged();

public:
  Garmin() : AltitudeEstimator<garmin::n_states>(garmin::name, garmin::frame_id){};

  ~Garmin(void) {
  }

  virtual void initialize(const ros::NodeHandle &parent_nh) override;
  virtual bool start(void) override;
  virtual bool pause(void) override;
  virtual bool reset(void) override;

  virtual double getState(const int &state_idx_in) const override;
  virtual double getState(const int &state_id_in, const int &axis_in) const override;

  virtual void setState(const double &state_in, const int &state_idx_in) override;
  virtual void setState(const double &state_in, const int &state_id_in, const int &axis_in) override;

  virtual states_t getStates(void) const override;
  virtual void     setStates(const states_t &states_in) override;

  virtual covariance_t getCovariance(void) const override;
  virtual void         setCovariance(const covariance_t &cov_in) override;

  void timeoutMavrosOdom(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
  void timeoutGarminRange(const std::string &topic, const ros::Time &last_msg, const int n_pubs);
};
}  // namespace mrs_odometry

#endif
