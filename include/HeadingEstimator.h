#ifndef HEADING_ESTIMATOR_H
#define HEADING_ESTIMATOR_H

#include <ros/ros.h>

#include <mrs_lib/lkf_legacy.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>

#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/MavrosState.h>

#include <string>
#include <vector>
#include <mutex>

namespace mrs_odometry
{

  class HeadingEstimator {

  public:
    HeadingEstimator(const std::string &estimator_name, const std::vector<bool> &fusing_measurement, const std::vector<Eigen::MatrixXd> &P_arr,
                     const std::vector<Eigen::MatrixXd> &Q_arr, const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &R);

    bool        doPrediction(const Eigen::VectorXd &input, double dt);
    bool        doCorrection(const Eigen::VectorXd &measurement, int measurement_type);
    bool        getStates(Eigen::MatrixXd &states);
    bool        getState(int state_id, Eigen::VectorXd &state);
    std::string getName(void);
    bool        setState(int state_id, const Eigen::VectorXd &state);
    bool        setR(double cov, int measurement_type);
    bool        getR(double &cov, int measurement_type);
    bool        getCovariance(Eigen::MatrixXd &cov);
    bool        setCovariance(const Eigen::MatrixXd &cov);
    bool        getInnovation(const Eigen::VectorXd &measurement, int measurement_type, Eigen::VectorXd &innovation);
    bool        getInnovationCovariance(int measurement_type, Eigen::MatrixXd &innovation_cov);
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
    int    m_n_inputs;
    int    m_n_measurements;
    size_t m_n_measurement_types;

    mrs_lib::Lkf *mp_lkf_x;

    std::mutex mutex_lkf;

    bool m_is_initialized = false;
  };

}  // namespace mrs_odometry

#endif
