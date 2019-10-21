#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <ros/ros.h>

#include <mutex>
#include <string>
#include <vector>

// Legacy LKF implementation
#include <mrs_lib/Lkf.h>

// New optimized LKF implementation
#include <mrs_lib/lkf.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>

#include <types.h>

namespace mrs_odometry
{

using statecov_t = mrs_lib::LKF_MRS_odom::statecov_t;
using u_t = mrs_lib::LKF_MRS_odom::u_t;
using z_t = mrs_lib::LKF_MRS_odom::z_t;
using R_t = mrs_lib::LKF_MRS_odom::R_t;
using H_t = mrs_lib::LKF_MRS_odom::H_t;

  class StateEstimator {

  public:
    StateEstimator(const std::string &estimator_name, const std::vector<bool> &fusing_measurement, const LatMat &A, const LatStateCol1D &B, const LatMat &Q, const std::vector<LatStateCol1D> &H, const std::vector<LatStateCol1D> &R_arr);

    bool        doPrediction(const Vec2 &input, double dt);
    bool        doCorrection(const Vec2 &measurement, int measurement_type);
    bool        getStates(LatState2D &states);
    bool        getState(int state_id, Vec2 &state);
    std::string getName(void);
    bool        setState(int state_id, const Vec2 &state);
    bool        setStates(LatState2D &states);
    bool        setR(double cov, int measurement_type);
    bool        getR(double &cov, int measurement_type);
    bool        setQ(double cov, const Eigen::Vector2i& idx); 
    bool        getQ(double &cov, const Eigen::Vector2i& idx); 
    bool        getQ(double &cov, int diag);
    bool        setQ(double cov, int diag, const std::vector<int>& except);
    bool        reset(const LatState2D &states);

  private:
    std::string                  m_estimator_name;
    std::vector<bool>            m_fusing_measurement;
    LatMat              m_A;
    LatState1D              m_B;
    LatMat              m_Q;
    std::vector<LatStateCol1D>              m_H;
    std::vector<LatStateCol1D> m_R_arr;

    int    m_n_states;
    size_t m_n_measurement_types;

    /* std::unique_ptr<mrs_lib::Lkf> mp_lkf_x; */
    /* std::unique_ptr<mrs_lib::Lkf> mp_lkf_y; */
    std::unique_ptr<mrs_lib::LKF_MRS_odom> mp_lkf_x;
    std::unique_ptr<mrs_lib::LKF_MRS_odom> mp_lkf_y;

    mrs_lib::LKF_MRS_odom::statecov_t sc_x;
    mrs_lib::LKF_MRS_odom::statecov_t sc_y;


    std::mutex mutex_lkf;

    bool m_is_initialized = false;
  };

}  // namespace mrs_odometry

#endif
