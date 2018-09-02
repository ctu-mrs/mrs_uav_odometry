#include "StateEstimator.h"

namespace mrs_odometry
{

/*  //{ StateEstimator() */

// clang-format off
StateEstimator::StateEstimator(
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const std::vector<Eigen::MatrixXd> &P_arr,
    const std::vector<Eigen::MatrixXd> &Q_arr,
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &R)
    :
    m_estimator_name(estimator_name),
    m_fusing_measurement(fusing_measurement),
    m_P_arr(P_arr),
    m_Q_arr(Q_arr),
    m_A(A),
    m_B(B),
    m_R(R)
  {

  // clang-format on

  // Number of states
  m_n_states = m_A.rows();

  // Number of measurement types
  m_n_measurement_types = m_fusing_measurement.size();


  /*  //{ sanity checks */


  // Check size of m_A
  if (m_A.cols() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"A\". Should be: " << m_n_states << " is:" << m_A.cols() << std::endl;
    return;
  }

  // Check size of m_B
  if (m_B.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"B\". Should be: " << m_n_states << " is:" << m_B.cols() << std::endl;
    return;
  }

  // Check size of m_R
  if (m_R.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"R.rows()\". Should be: " << m_n_states << " is:" << m_R.rows() << std::endl;
    return;
  }

  if (m_R.cols() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_R.cols() << std::endl;
    return;
  }

  // Check size of m_P_arr
  if (m_P_arr.size() != m_n_measurement_types) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"m_P_arr\". Should be: " << m_n_measurement_types << " is:" << m_P_arr.size() << std::endl;
    return;
  }

  // Check size of m_P_arr elements
  for (size_t i = 0; i < m_P_arr.size(); i++) {
    if (m_P_arr[i].rows() != 1 || m_P_arr[i].cols() != m_n_states) {
      std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
                << "): wrong size of \"m_P_arr[" << i << "]\". Should be: (1, " << m_n_states << ") is: (" << m_P_arr[i].rows() << ", " << m_P_arr[i].cols()
                << ")" << std::endl;
      return;
    }
  }

  // Check size of m_Q_arr
  if (m_Q_arr.size() != m_n_measurement_types) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"m_Q_arr\". Should be: " << m_n_measurement_types << " is:" << m_Q_arr.size() << std::endl;
    return;
  }

  // Check size of m_Q_arr elements
  for (size_t i = 0; i < m_Q_arr.size(); i++) {
    if (m_Q_arr[i].rows() != 1 || m_Q_arr[i].cols() != 1) {
      std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
                << "): wrong size of \"m_Q_arr[" << i << "]\". Should be: (1, 1) is: (" << m_Q_arr[i].rows() << ", " << m_Q_arr[i].cols() << ")" << std::endl;
      return;
    }
  }


  //}
  Eigen::MatrixXd Q_zero = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd P_zero = Eigen::MatrixXd::Zero(1, m_n_states);

  mp_lkf_x = new mrs_lib::Lkf(m_n_states, 1, 1, m_A, m_B, m_R, Q_zero, P_zero);
  mp_lkf_y = new mrs_lib::Lkf(m_n_states, 1, 1, m_A, m_B, m_R, Q_zero, P_zero);

  std::cout << "[StateEstimator]: New StateEstimator initialized " << std::endl;
  std::cout << "name: " << m_estimator_name << std::endl;
  std::cout << " fusing measurements: " << std::endl;
  for (size_t i = 0; i < m_fusing_measurement.size(); i++) {
    std::cout << m_fusing_measurement[i] << " ";
  }
  std::cout << std::endl << " P_arr: " << std::endl;
  for (size_t i = 0; i < m_P_arr.size(); i++) {
    std::cout << m_P_arr[i] << std::endl;
  }
  std::cout << std::endl << " Q_arr: " << std::endl;
  for (size_t i = 0; i < m_Q_arr.size(); i++) {
    std::cout << m_Q_arr[i] << std::endl;
  }
  std::cout << std::endl << " A: " << std::endl << m_A << std::endl << " B: " << std::endl << m_B << std::endl << " R: " << std::endl << m_R << std::endl;

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool StateEstimator::doPrediction(const Eigen::VectorXd &input, double dt) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of input
  if (input.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): wrong size of \"input\". Should be: " << 2 << " is:" << input.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(input(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(input(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input(1)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  /* std::cout << "[StateEstimator]: " << m_estimator_name << " fusing input: " << input << " with time step: " << dt << std::endl; */

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

  return true;
}

//}

/*  //{ doCorrection() */

bool StateEstimator::doCorrection(const Eigen::VectorXd &measurement, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (measurement.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): wrong size of \"input\". Should be: " << 2 << " is:" << measurement.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(measurement(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  //}

  /* std::cout << "[StateEstimator]: " << m_estimator_name << " fusing correction: " << measurement << " of type: " << measurement_type << std::endl; */

  Eigen::VectorXd mes_vec_x = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd mes_vec_y = Eigen::VectorXd::Zero(1);

  mes_vec_x << measurement(0);
  mes_vec_y << measurement(1);

  mutex_lkf.lock();
  {
    mp_lkf_x->setP(m_P_arr[measurement_type]);
    mp_lkf_x->setMeasurement(mes_vec_x, m_Q_arr[measurement_type]);
    mp_lkf_x->doCorrection();
    mp_lkf_y->setP(m_P_arr[measurement_type]);
    mp_lkf_y->setMeasurement(mes_vec_y, m_Q_arr[measurement_type]);
    mp_lkf_y->doCorrection();
  }
  mutex_lkf.unlock();

  return true;
}

//}

/*  //{ getStates() */

bool StateEstimator::getStates(Eigen::MatrixXd &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  mutex_lkf.lock();
  {
    states.col(0) = mp_lkf_x->getStates();
    states.col(1) = mp_lkf_y->getStates();
  }
  mutex_lkf.unlock();

  return true;
}

//}

/*  //{ getState() */

bool StateEstimator::getState(int state_id, Eigen::VectorXd &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  mutex_lkf.lock();
  {
    state(0) = mp_lkf_x->getState(state_id);
    state(1) = mp_lkf_y->getState(state_id);
  }
  mutex_lkf.unlock();

  return true;
}

//}

/*  //{ setState() */

bool StateEstimator::setState(int state_id, const Eigen::VectorXd &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check the size of state
  if (state.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): wrong size of \"state.size()\". Should be: " << 2 << " is:" << state.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(state(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state(1)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  mutex_lkf.lock();
  {
    mp_lkf_x->setState(state_id, state(0));
    mp_lkf_y->setState(state_id, state(1));
  }
  mutex_lkf.unlock();

  return true;
}

//}

/*  //{ setCovariance() */

bool StateEstimator::setCovariance(double cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  mutex_lkf.lock();
  { m_Q_arr[measurement_type](0, 0) = cov; }
  mutex_lkf.unlock();

  return true;
}

//}

/*  //{ reset() */

bool StateEstimator::reset(const Eigen::MatrixXd &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;
  // Check size of states
  if ((int)states.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.rows()\". Should be: " << m_n_states << " is:" << states.rows() << std::endl;
    return false;
  }

  if (states.cols() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.cols()\". Should be: " << 2 << " is:" << states.cols() << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < states.rows(); i++) {
    for (int j = 0; j < states.cols(); j++) {
      if (!std::isfinite(states(i, j))) {
        std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
                  << "): NaN detected in variable \"states(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  mutex_lkf.lock();
  {
    mp_lkf_x->reset(states.col(0));
    mp_lkf_y->reset(states.col(1));
  }
  mutex_lkf.unlock();

  return true;
}

//}
}  // namespace mrs_odometry
