#include "AltitudeEstimator.h"

namespace mrs_odometry
{

/*  //{ AltitudeEstimator() */

// clang-format off
AltitudeEstimator::AltitudeEstimator(
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
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"A\". Should be: " << m_n_states << " is:" << m_A.cols() << std::endl;
    return;
  }

  // Check size of m_B
  if (m_B.rows() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"B\". Should be: " << m_n_states << " is:" << m_B.cols() << std::endl;
    return;
  }

  // Check size of m_R
  if (m_R.rows() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"R.rows()\". Should be: " << m_n_states << " is:" << m_R.rows() << std::endl;
    return;
  }

  if (m_R.cols() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_R.cols() << std::endl;
    return;
  }

  // Check size of m_P_arr
  if (m_P_arr.size() != m_n_measurement_types) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"m_P_arr\". Should be: " << m_n_measurement_types << " is:" << m_P_arr.size() << std::endl;
    return;
  }

  // Check size of m_P_arr elements
  for (size_t i = 0; i < m_P_arr.size(); i++) {
    if (m_P_arr[i].rows() != 1 || m_P_arr[i].cols() != m_n_states) {
      std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
                << "): wrong size of \"m_P_arr[" << i << "]\". Should be: (1, " << m_n_states << ") is: (" << m_P_arr[i].rows() << ", " << m_P_arr[i].cols()
                << ")" << std::endl;
      return;
    }
  }

  // Check size of m_Q_arr
  if (m_Q_arr.size() != m_n_measurement_types) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"m_Q_arr\". Should be: " << m_n_measurement_types << " is:" << m_Q_arr.size() << std::endl;
    return;
  }

  // Check size of m_Q_arr elements
  for (size_t i = 0; i < m_Q_arr.size(); i++) {
    if (m_Q_arr[i].rows() != 1 || m_Q_arr[i].cols() != 1) {
      std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
                << "): wrong size of \"m_Q_arr[" << i << "]\". Should be: (1, 1) is: (" << m_Q_arr[i].rows() << ", " << m_Q_arr[i].cols() << ")" << std::endl;
      return;
    }
  }


  //}

  Eigen::MatrixXd Q_zero = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd P_zero = Eigen::MatrixXd::Zero(1, m_n_states);

  mp_lkf_x = new mrs_lib::Lkf(m_n_states, 1, 1, m_A, m_B, m_R, Q_zero, P_zero);

  // Initialize all states to 0
  for (int i = 0; i < m_n_states; i++) {
    mp_lkf_x->setState(i, 0.0);
  }

  std::cout << "[AltitudeEstimator]: New AltitudeEstimator initialized " << std::endl;
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

bool AltitudeEstimator::doPrediction(const Eigen::VectorXd &input, double dt) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of input
  if (input.size() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): wrong size of \"input\". Should be: " << 1 << " is:" << input.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(input(0))) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " fusing input: " << input << " with time step: " << dt << std::endl; */

  Eigen::VectorXd input_vec_x = Eigen::VectorXd::Zero(1);

  input_vec_x << input(0);

  Eigen::MatrixXd newA = m_A;
  Eigen::MatrixXd newB = m_B;
  newB(0, 0)           = dt;
  newB(2, 0)           = dt;

  /* newA(0, 1)           = dt; */
  /* newA(2, 3)           = dt; */

  /* newA(0, 2)           = std::pow(dt, 2)/2; */
  /* newA(1, 2)           = dt; */

  /* newA(3, 1)           = dt; */
  /* newA(3, 2)           = std::pow(dt, 2)/2; */

  /* std::cout << newA << std::endl; */
  /* std::cout << newB << std::endl; */

  {
    std::scoped_lock lock(mutex_lkf);

    /* mp_lkf_x->setA(newA); */
    mp_lkf_x->setB(newB);
    mp_lkf_x->setInput(input_vec_x);
    mp_lkf_x->iterateWithoutCorrection();
  }

  return true;
}

//}

/*  //{ doPrediction() */

bool AltitudeEstimator::doPrediction(const Eigen::VectorXd &input) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of input
  if (input.size() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input 
              << "): wrong size of \"input\". Should be: " << 1 << " is:" << input.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(input(0))) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input
              << "): NaN detected in variable \"input(0)\"." << std::endl;
    return false;
  }

  //}

  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " fusing input: " << input << " with time step: " << dt << std::endl; */

  Eigen::VectorXd input_vec_x = Eigen::VectorXd::Zero(1);

  input_vec_x << input(0);

  {
    std::scoped_lock lock(mutex_lkf);

    mp_lkf_x->setInput(input_vec_x);
    mp_lkf_x->iterateWithoutCorrection();
  }

  return true;
}

//}

/*  //{ doCorrection() */

bool AltitudeEstimator::doCorrection(const Eigen::VectorXd &measurement, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (measurement.size() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): wrong size of \"input\". Should be: " << 2 << " is:" << measurement.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(measurement(0))) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  // Fuse the measurement
  Eigen::VectorXd mes_vec_x = Eigen::VectorXd::Zero(1);

  mes_vec_x << measurement(0);

  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " fusing correction: " << measurement << " of type: " << measurement_type << " with mapping:
   * "
   * <<  m_P_arr[measurement_type] << " and covariance" <<  m_Q_arr[measurement_type] << std::endl; */
  {
    std::scoped_lock lock(mutex_lkf);

    Eigen::VectorXd states = Eigen::VectorXd::Zero(m_n_states);
    states                 = mp_lkf_x->getStates();
    mp_lkf_x->setP(m_P_arr[measurement_type]);
    mp_lkf_x->setMeasurement(mes_vec_x, m_Q_arr[measurement_type]);
    mp_lkf_x->doCorrection();
  }

  return true;
}

//}

/*  //{ getInnovation() */

bool AltitudeEstimator::getInnovation(const Eigen::VectorXd &measurement, int measurement_type, Eigen::VectorXd &innovation) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (measurement.size() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovation(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): wrong size of \"input\". Should be: " << 2 << " is:" << measurement.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(measurement(0))) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovation(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovation(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovation(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  //}

  // Fuse the measurement if this estimator allows it
  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " fusing correction: " << measurement << " of type: " << measurement_type << " with mapping:
   * "
   * <<  m_P_arr[measurement_type] << " and covariance" <<  m_Q_arr[measurement_type] << std::endl; */
  {
    std::scoped_lock lock(mutex_lkf);

    Eigen::VectorXd states = Eigen::VectorXd::Zero(m_n_states);
    states                 = mp_lkf_x->getStates();
    innovation             = measurement - (m_P_arr[measurement_type] * states);
  }

  return true;
}

//}

/*  //{ getInnovationCovariance() */

bool AltitudeEstimator::getInnovationCovariance(int measurement_type, Eigen::MatrixXd &innovation_cov) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovationCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getInnovationCovariance(int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  //}

  // Fuse the measurement if this estimator allows it
  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " fusing correction: " << measurement << " of type: " << measurement_type << " with mapping:
   * "
   * <<  m_P_arr[measurement_type] << " and covariance" <<  m_Q_arr[measurement_type] << std::endl; */
  {
    std::scoped_lock lock(mutex_lkf);

    innovation_cov = m_Q_arr[measurement_type] + (m_P_arr[measurement_type] * mp_lkf_x->getCovariance() * m_P_arr[measurement_type].transpose());
  }

  return true;
}

//}

/*  //{ getStates() */

bool AltitudeEstimator::getStates(Eigen::MatrixXd &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  states = mp_lkf_x->getStates();

  return true;
}

//}

/*  //{ getN() */

bool AltitudeEstimator::getN(int& n) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  n = m_n_states; 

  return true;
}

//}

/*  //{ getState() */

bool AltitudeEstimator::getState(int state_id, Eigen::VectorXd &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " getting value: " << mp_lkf_x->getState(state_id) << " of state: " << state_id <<
     * std::endl;
     */
    state(0) = mp_lkf_x->getState(state_id);
  }

  return true;
}

//}

/*  //{ getName() */

std::string AltitudeEstimator::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool AltitudeEstimator::setState(int state_id, const Eigen::VectorXd &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check the size of state
  if (state.size() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): wrong size of \"state.size()\". Should be: " << 2 << " is:" << state.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(state(0))) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    mp_lkf_x->setState(state_id, state(0));
  }

  return true;
}

//}

/*  //{ setR() */

bool AltitudeEstimator::setR(double cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  double old_cov = m_Q_arr[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q_arr[measurement_type](0, 0) = cov;
  }

  std::cout << "[AltitudeEstimator]: " << m_estimator_name << ".setQ(double cov=" << cov << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl;

  return true;
}

//}

/*  //{ getR() */

bool AltitudeEstimator::getR(double &cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_Q_arr[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getCovariance() */

bool AltitudeEstimator::getCovariance(Eigen::MatrixXd &cov) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = mp_lkf_x->getCovariance();
  }

  return true;
}

//}

/*  //{ setCovariance() */

bool AltitudeEstimator::setCovariance(const Eigen::MatrixXd &cov) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (cov.rows() != m_n_states || cov.cols() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &cov=" << cov << "): wrong size of \"cov\". Should be: ("
              << m_n_states << "," << m_n_states << ") is: (" << cov.rows() << "," << cov.cols() << ")" << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < m_n_states; i++) {
    if (!std::isfinite(cov(i, i))) {
      std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &cov=" << cov << "): NaN detected in variable \"cov("
                << i << "," << i << ")\"." << std::endl;
      return false;
    }
  }

  //}

  // Set the covariance
  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << " setting covariance: " << cov << std::endl; */
  {
    std::scoped_lock lock(mutex_lkf);

    mp_lkf_x->setCovariance(cov);
  }

  return true;
}
//}

/*  //{ reset() */

bool AltitudeEstimator::reset(const Eigen::MatrixXd &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;
  // Check size of states
  if ((int)states.rows() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.rows()\". Should be: " << m_n_states << " is:" << states.rows() << std::endl;
    return false;
  }

  if (states.cols() != 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.cols()\". Should be: " << 1 << " is:" << states.cols() << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < states.rows(); i++) {
    for (int j = 0; j < states.cols(); j++) {
      if (!std::isfinite(states(i, j))) {
        std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
                  << "): NaN detected in variable \"states(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    mp_lkf_x->reset(states.col(0));
  }

  return true;
}

//}
}  // namespace mrs_odometry
