#ifndef ALTITUDE_ESTIMATOR_REPREDICTOR_H
#define ALTITUDE_ESTIMATOR_REPREDICTOR_H

#include <ros/ros.h>

#include <mrs_lib/repredictor.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>

#include <string>
#include <vector>
#include <mutex>

#include "types.h"

#include <mrs_msgs/Altitude.h>

#define ALT_DT 0.01
#define ALT_INPUT_COEFF 0.10

namespace mrs_uav_odometry
{


class AltitudeEstimatorRepredictor {

public:
  AltitudeEstimatorRepredictor(const ros::NodeHandle &nh, const std::string &estimator_name, const std::vector<bool> &fusing_measurement,
                               const std::vector<var_alt_H_t> &H_multi, const var_alt_Q_t &Q, const std::vector<var_alt_R_t> &R_multi);

  bool        doPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp);
  bool        doPrediction(const double input);
  bool        doCorrection(const double &measurement, int measurement_type, ros::Time &meas_stamp, ros::Time &pred_stamp);
  bool        getStates(var_alt_x_t &x);
  bool        getState(int state_id, double &state_val);
  std::string getName(void);
  bool        setState(int state_id, const double &state_val);
  bool        setR(double R, int measurement_type);
  bool        getR(double &R, int measurement_type);
  bool        setQ(double cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, const Eigen::Vector2i &idx);
  bool        setInputCoeff(double coeff);
  bool        getCovariance(var_alt_P_t &P);
  bool        setCovariance(const var_alt_P_t &P);
  bool        reset(const var_alt_x_t &states);

private:
  ros::NodeHandle   nh_;
  std::string       m_estimator_name;
  std::vector<bool> m_fusing_measurement;
  int               m_n_states;
  size_t            m_n_measurement_types;

  // repredictor buffer size
  const unsigned m_buf_sz = 100;

  // repredictor
  std::unique_ptr<rep_t> mp_rep;

  // State transition matrix
  var_alt_A_t m_A;

  // Input matrix
  var_alt_B_t m_B;

  // Input coefficient
  double m_b = ALT_INPUT_COEFF;

  // Array with mapping matrices for each fused measurement
  std::vector<var_alt_H_t> m_H_multi;

  // Process covariance matrix
  var_alt_Q_t m_Q;

  // Array with covariances of each fused measurement
  std::vector<var_alt_R_t> m_R_multi;

  // Default dt
  double m_dt    = ALT_DT;
  double m_dt_sq = m_dt * m_dt / 2;

  // Kalman filter - the core of the estimator
  std::vector<std::shared_ptr<var_lkf_alt_t>> mp_lkf_vector;

  // Variable for holding the current state and covariance
  var_alt_statecov_t m_sc;

  std::mutex mutex_lkf;

  bool m_is_initialized = false;

  ros::Publisher pub_state_;
};

}  // namespace mrs_uav_odometry

#endif
