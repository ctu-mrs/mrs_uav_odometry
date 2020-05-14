#include "AltitudeEstimator.h"

namespace mrs_uav_odometry
{

/*  //{ AltitudeEstimator() */

// clang-format off
AltitudeEstimator::AltitudeEstimator(
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const std::vector<alt_H_t> &H_multi,
    const alt_Q_t &Q,
    const std::vector<alt_R_t> &R_multi)
    :
    m_estimator_name(estimator_name),
    m_fusing_measurement(fusing_measurement),
    m_H_multi(H_multi),
    m_Q(Q),
    m_R_multi(R_multi)
  {

  // clang-format on

  // Number of states
  m_n_states = m_A.rows();

  // Number of measurement types
  m_n_measurement_types = m_fusing_measurement.size();

  /*  //{ sanity checks */

  // Check size of m_Q
  if (m_Q.rows() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"Q.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_R_multi
  if (m_R_multi.size() != m_n_measurement_types) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
              << "): wrong size of \"m_R_multi\". Should be: " << m_n_measurement_types << " is:" << m_R_multi.size() << std::endl;
    return;
  }

  // Check size of m_R_multi elements
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    if (m_R_multi[i].rows() != ALT_N_MEASUREMENTS || m_R_multi[i].cols() != ALT_N_MEASUREMENTS) {
      std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (3, 3) is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")" << std::endl;
      return;
    }
  }


  //}

  m_A << 1, m_dt, m_dt_sq,
      0, 1, m_dt,
      0, 0, 1;

  m_B << 0, 0, m_dt;

  // set measurement mapping matrix H to zero, it will be set later during each correction step
  alt_H_t m_H_zero = m_H_zero.Zero();

  mp_lkf = std::make_unique<lkf_alt_t>(m_A, m_B, m_H_zero);

  // Initialize all states to 0
  const alt_x_t x0 = alt_x_t::Zero();
  alt_P_t P_tmp = alt_P_t::Identity();
  const alt_P_t P0 = 1000.0*P_tmp*P_tmp.transpose();
  const alt_statecov_t sc0({x0, P0});
  m_sc = sc0;

  std::cout << "[AltitudeEstimator]: New AltitudeEstimator initialized " << std::endl;
  std::cout << "name: " << m_estimator_name << std::endl;
  std::cout << " fusing measurements: " << std::endl;
  for (size_t i = 0; i < m_fusing_measurement.size(); i++) {
    std::cout << m_fusing_measurement[i] << " ";
  }

  std::cout << std::endl << " A: " << std::endl << m_A << std::endl << " B: " << std::endl << m_B << std::endl << " Q: " << std::endl << m_Q << std::endl;

  std::cout << std::endl << " R_multi: " << std::endl;
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    std::cout << m_R_multi[i] << std::endl;
  }

  std::cout << std::endl << " H_multi: " << std::endl;
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    std::cout << m_H_multi[i] << std::endl;
  }

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool AltitudeEstimator::doPrediction(const double input, const double dt) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  alt_u_t u = u.Zero();
  //TODO u(2)??? spis u(0), kdyz to ma jen jeden input ne?
  u(2) = input;

  double dtsq = pow(dt, 2);
  alt_A_t A = m_A;
  alt_B_t B = m_B;
  B(2, 0)           = dt;

  A(0, 1)           = dt;
  A(1, 2)           = dt;

  A(0, 2)           = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
    mp_lkf->A = A;
    mp_lkf->B = B;
    m_sc = mp_lkf->predict(m_sc, u, m_Q, dt);
  }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimator]: LKF prediction step failed: %s", e.what());
    }
  }
  return true;
}


//}

/*  //{ doPrediction() */

bool AltitudeEstimator::doPrediction(const double input) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const double input=" << input
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  //}

  alt_u_t u = u.Zero();
  u(2) = input;

  double dt = m_dt;
  double dtsq = pow(dt, 2);

  alt_B_t B = m_B;
  B(2, 0)           = dt;

  alt_A_t A = m_A;
  A(0, 1)           = dt;
  A(1, 2)           = dt;

  A(0, 2)           = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);
    try {
      // Apply the prediction step
    mp_lkf->A = A;
    mp_lkf->B = B;
    m_sc = mp_lkf->predict(m_sc, u, m_Q, dt);
  }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimator]: LKF prediction step failed: %s", e.what());
    }
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

  // Prepare the measurement vector
  alt_z_t z;
  z << measurement(0);

  alt_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
    std::scoped_lock lock(mutex_lkf);
    {

    try {
      mp_lkf->H = m_H_multi[measurement_type];
      m_sc = mp_lkf->correct(m_sc, z, m_R_multi[measurement_type]);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimator]: LKF correction step failed: %s", e.what());
    }
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

  states = m_sc.x;

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

    state(0) = m_sc.x(state_id);
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

    m_sc.x(state_id) = state(0);
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

  double old_cov = m_R_multi[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = cov;
  }

  std::cout << "[AltitudeEstimator]: " << m_estimator_name << ".setQ(double cov=" << cov << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_cov << " to: " << m_R_multi[measurement_type](0, 0) << std::endl;

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

    cov = m_R_multi[measurement_type](0, 0);
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

    cov = m_sc.P;
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
  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.P = cov;
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

    m_sc.x = (states.col(0));
  }

  return true;
}

//}
}  // namespace mrs_uav_odometry
