#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <ros/ros.h>
#include <mutex>

#include <mrs_lib/Lkf.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>

#include <string>
#include <vector>
#include <mutex>

namespace mrs_odometry
{

  class StateEstimator {

  public:
    StateEstimator(const std::string &estimator_name, const std::vector<bool> &fusing_measurement, const std::vector<Eigen::MatrixXd> &P_arr,
                   const std::vector<Eigen::MatrixXd> &Q_arr, const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &R);

    bool        doPrediction(const Eigen::VectorXd &input, double dt);
    bool        doCorrection(const Eigen::VectorXd &measurement, int measurement_type);
    bool        getStates(Eigen::MatrixXd &states);
    bool        getState(int state_id, Eigen::VectorXd &state);
    std::string getName(void);
    bool        setState(int state_id, const Eigen::VectorXd &state);
    bool        setStates(Eigen::MatrixXd &states);
    bool        setQ(double cov, int measurement_type);
    bool        getQ(double &cov, int measurement_type);
    bool        setR(double cov, const Eigen::Vector2i& idx); 
    bool        getR(double &cov, const Eigen::Vector2i& idx); 
    bool        getR(double &cov, int diag);
    bool        setR(double cov, int diag, const std::vector<int>& except);
    bool        reset(const Eigen::MatrixXd &states);

  private:
    std::string                  m_estimator_name;
    std::vector<bool>            m_fusing_measurement;
    std::vector<Eigen::MatrixXd> m_P_arr;
    std::vector<Eigen::MatrixXd> m_Q_arr;
    Eigen::MatrixXd              m_A;
    Eigen::MatrixXd              m_B;
    Eigen::MatrixXd              m_R;

    int    m_n_states;
    size_t m_n_measurement_types;

    mrs_lib::Lkf *mp_lkf_x;
    mrs_lib::Lkf *mp_lkf_y;

    std::mutex mutex_lkf;

    bool m_is_initialized = false;
  };

}  // namespace mrs_odometry

#endif
