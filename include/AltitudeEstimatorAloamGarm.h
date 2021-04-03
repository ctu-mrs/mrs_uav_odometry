#ifndef ALTITUDE_ESTIMATOR_ALOAMGARM_H
#define ALTITUDE_ESTIMATOR_ALOAMGARM_H

#include <ros/ros.h>

#include <mrs_lib/repredictor.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_lib/median_filter.h>
#include <mrs_lib/param_loader.h>
#include "AltitudeEstimator.h"

#include <string>
#include <vector>
#include <mutex>

#include "types.h"

#define ALT_DT 0.01
#define ALT_INPUT_COEFF 0.10

namespace mrs_uav_odometry
{


class AltitudeEstimatorAloamGarm : public AltitudeEstimator {

public:
  AltitudeEstimatorAloamGarm(const std::string &estimator_name, const std::vector<bool> &fusing_measurement, const std::vector<var_alt_H_t> &H_multi, const var_alt_Q_t &Q,
                    const std::vector<var_alt_R_t> &R_multi, const ros::NodeHandle &nh, const bool use_repredictor = false);

  bool doPrediction(const double input, const double dt, const ros::Time &input_stamp = ros::Time::now(), const ros::Time &predict_stamp = ros::Time::now());
  bool doPrediction(const double input, const ros::Time &input_stamp = ros::Time::now(), const ros::Time &predict_stamp = ros::Time::now());
  bool doCorrection(const double &measurement, int measurement_type, const ros::Time &meas_stamp,
                    const ros::Time &predict_stamp, const std::string &measurement_name);

  bool        getStates(algarm_alt_x_t &x);
  bool        getStates(alt_x_t &x);
  bool        getState(int state_id, double &state_val);
  std::string getName(void);
  bool        setState(int state_id, const double &state_val);
  bool        setR(double R, int measurement_type);
  bool        getR(double &R, int measurement_type);
  bool        setQ(double cov, const Eigen::Vector2i &idx);
  bool        getQ(double &cov, const Eigen::Vector2i &idx);
  bool        setInputCoeff(double coeff);
  bool        getCovariance(alt_P_t &P);
  bool        setCovariance(const alt_P_t &P);
  bool        reset(const alt_x_t &states);

  void        callbackSlamEigenvalues(const mrs_msgs::Float64ArrayStampedConstPtr &msg);

private:
  std::string       m_estimator_name;
  std::vector<bool> m_fusing_measurement;
  int               m_n_states;
  size_t            m_n_measurement_types;

  // repredictor
  std::unique_ptr<algarm_rep_t> mp_rep;

  // State transition matrix
  algarm_alt_A_t m_A;

  // Input matrix
  algarm_alt_B_t m_B;

  // Input coefficient
  double m_b = ALT_INPUT_COEFF;

  // Array with mapping matrices for each fused measurement
  std::vector<algarm_alt_H_t> m_H_multi;
  std::vector<var_alt_H_t> m_H_multi_orig;

  // Process covariance matrix
  algarm_alt_Q_t m_Q;
  var_alt_Q_t m_Q_orig;

  // Array with covariances of each fused measurement
  std::vector<algarm_alt_R_t> m_R_multi;

  ros::NodeHandle m_nh;
  
  // parameter deciding whether to use repredictor or classic lkf
  bool m_use_repredictor = false;

  // subscriber for SLAM eigenvalues
  ros::Subscriber m_eigenvalue_subscriber;

  // debug publishers
  ros::Publisher debug_state_publisher;
  ros::Publisher debug_cov_publisher;
  ros::Publisher debug_Q_publisher;
  ros::Publisher debug_duration_publisher;
  ros::Publisher debug_aloam_ok_publisher;

  // Default dt
  double m_dt    = ALT_DT;
  double m_dt_sq = m_dt * m_dt / 2;

  // Kalman filter - the core of the estimator
  std::unique_ptr<lkf_alt_t> mp_lkf;

  // Kalman filter vector for repredictor
  std::vector<std::shared_ptr<algarm_alt_t>> mp_lkf_vector;

  // Variable for holding the current state and covariance
  algarm_alt_statecov_t m_sc;

  std::mutex mutex_lkf;

  int m_garmin_biased_id;
  int m_aloam_biased_id;
  int m_baro_biased_id;

  bool m_aloam_ok = false;
  float m_aloam_eig = 0;
  bool m_eigenvalue_received = false;

  std::unique_ptr<MedianFilter> m_median_filter;

  bool m_is_initialized = false;

  // Config params
  float _mf_changes_buffer_size_;
  float _mf_changes_max_diff_;
  float _mf_changes_max_diff_close_to_ground_;
  float _mf_close_to_ground_threshold_;
  float _q_factor_range_bias_slam_ok_;
  float _q_factor_range_bias_range_jump_;
  float _q_factor_slam_bias_;
  float _r_factor_range_jump_;
  float _r_factor_slam_bad_;
  algarm_alt_H_t _H_range_biased_;
  algarm_alt_H_t _H_slam_biased_;
  algarm_alt_H_t _H_baro_biased_;
  float _q_biases_;
  int _biased_state_count_;
  float _eigenvalue_hysteresis_upper_;
  float _eigenvalue_hysteresis_lower_;
  int _repredictor_buffer_size_;
  bool _debug_;

};

}  // namespace mrs_uav_odometry

#endif
