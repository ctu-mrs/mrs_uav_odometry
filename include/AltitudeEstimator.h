#ifndef ALTITUDE_ESTIMATOR_H
#define ALTITUDE_ESTIMATOR_H

#include <ros/ros.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>

#include <string>
#include <vector>
#include <mutex>

#include "types.h"

#define ALT_DT 0.01
#define ALT_INPUT_COEFF 0.2

namespace mrs_uav_odometry
{


  class AltitudeEstimator {

  public:
    AltitudeEstimator(const std::string &estimator_name, const std::vector<bool> &fusing_measurement, const std::vector<alt_H_t> &H_multi, const alt_Q_t &Q,
                      const std::vector<alt_R_t> &R_multi);

    bool        doPrediction(const double input, const double dt);
    bool        doPrediction(const double input);
    bool        doCorrection(const double &measurement, int measurement_type);
    bool        getStates(alt_x_t &x);
    bool        getState(int state_id, double &state_val);
    std::string getName(void);
    bool        setState(int state_id, const double &state_val);
    bool        setR(double R, int measurement_type);
    bool        getR(double &R, int measurement_type);
    bool        getCovariance(alt_P_t &P);
    bool        setCovariance(const alt_P_t &P);
    bool        reset(const alt_x_t &states);

  private:
    std::string                  m_estimator_name;
    std::vector<bool>            m_fusing_measurement;
    int    m_n_states;
    size_t m_n_measurement_types;

    // State transition matrix
    alt_A_t              m_A;

    // Input matrix
    alt_B_t              m_B;

    // Input coefficient
    double m_b = ALT_INPUT_COEFF;

    // Array with mapping matrices for each fused measurement
    std::vector<alt_H_t> m_H_multi;

    // Process covariance matrix
    alt_Q_t              m_Q;

    // Array with covariances of each fused measurement
    std::vector<alt_R_t> m_R_multi;


    // Default dt
    double m_dt = ALT_DT;
    double m_dt_sq = m_dt*m_dt/2;

    // Kalman filter - the core of the estimator 
    std::unique_ptr<lkf_alt_t> mp_lkf;

    // Variable for holding the current state and covariance 
    alt_statecov_t m_sc;

    std::mutex mutex_lkf;

    bool m_is_initialized = false;
  };

}  // namespace mrs_uav_odometry

#endif
