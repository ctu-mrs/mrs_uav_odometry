#include "mrs_odometry/StateEstimator.h"

namespace mrs_odometry
{

/*  //{ StateEstimator() */

StateEstimator::StateEstimator(const std::string estimator_name, const std::vector<bool> fusing_measurement, std::vector<Eigen::MatrixXd> P_arr,
                 std::vector<Eigen::MatrixXd> m_Q_arr, const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &R)
    : m_estimator_name(estimator_name), m_fusing_measurement(fusing_measurement), m_P_arr(P_arr), m_Q_arr(Q_arr), m_A(A), m_B(B), m_R(R) {

  Eigen::MatrixXd Q_zero = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd P_zero = Eigen::MatrixXd::Zero(1, m_A.rows());

  mp_lkf_x = new mrs_lib::Lkf(m_A.rows(), 1, 1, m_A, m_B, m_R, Q_zero, P_zero);
  mp_lkf_y = new mrs_lib::Lkf(m_A.rows(), 1, 1, m_A, m_B, m_R, Q_zero, P_zero);
}

//}

/*  //{ doPrediction() */

void StateEstimator::doPrediction(const Eigen::VectorXd &input, double dt) {

  Eigen::VectorXd input_vec_x = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd input_vec_y = Eigen::VectorXd::Zero(1);

  input_vec_x << input(0);
  input_vec_y << input(1);

  Eigen::MatrixXd newA = m_A;
  newA(0, 1)           = dt;
  newA(1, 2)           = dt;

  mutex_lkf.lock();
  {
    mp_lkf_x->setA(newA);
    mp_lkf_x->setInput(input_vec_x);
    mp_lkf_x->iterateWithoutCorrection();
    mp_lkf_y->setA(newA);
    mp_lkf_y->setInput(input_vec_y);
    mp_lkf_y->iterateWithoutCorrection();
  }
  mutex_lkf.unlock();
}

//}

/*  //{ doCorrection() */

void StateEstimator::doCorrection(const Eigen::VectorXd &measurement, int measurement_type) {

  if (!m_fusing_measurement[measurement_type]) {
    return;
  }

  Eigen::VectorXd mes_vec_x = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd mes_vec_y = Eigen::VectorXd::Zero(1);

  mes_vec_x << measurement(0);
  mes_vec_y << measurement(1);

  mutex_lkf.lock();
  {
    mp_lkf_x->setP(m_P_arr[mesurement_type]);
    mp_lkf_x->setMeasurement(mes_vec_x, m_Q_arr[measurement_type]);
    mp_lkf_x->doCorrection();
    mp_lkf_y->setP(m_P_arr[mesurement_type]);
    mp_lkf_y->setMeasurement(mes_vec_y, m_Q_arr[measurement_type]);
    mp_lkf_y->doCorrection();
  }
  mutex_lkf.unlock();
}

//}

/*  //{ getStates() */

Eigen::MatrixXd StateEstimator::getStates(void) {

  Eigen::MatrixXd states = Eigen::MatrixXd::Zero(m_A.rows(), m_A.rows());

  mutex_lkf.lock();
  {
    states.col(0) = mp_lkf_x.getStates();
    states.col(1) = mp_lkf_y.getStates();
  }
  mutex_lkf.unlock();

  return states;
}

//}

/*  //{ getState() */

double StateEstimator::getState(int row, int col) {

  if (col == 0) {
    mutex_lkf.lock();
    { double state = mp_lkf_x.getState(col); }
    mutex_lkf.unlock();
    return state;
  } else if (col == 1) {
    mutex_lkf.lock();
    { double state = mp_lkf_y.getState(col); }
    mutex_lkf.unlock();
    return state;
  } else {
    std::cerr << "[StateEstimator]: Requested invalid state." << std::endl;
  }
}

//}

}  // namespace mrs_odometry
