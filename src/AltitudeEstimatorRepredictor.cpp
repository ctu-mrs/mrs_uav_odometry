#include "AltitudeEstimatorRepredictor.h"

namespace mrs_uav_odometry
{

/*  //{ AltitudeEstimatorRepredictor() */

var_alt_A_t generateA(const double dt) {
  var_alt_A_t A;
  A << 1, dt, dt * dt, 0, 1, dt, 0, 0, 1.0 - ALT_INPUT_COEFF;
  return A;
}

var_alt_B_t generateB([[maybe_unused]] const double dt) {
  var_alt_B_t B;
  B << 0, 0, ALT_INPUT_COEFF;
  return B;
}

// clang-format off
AltitudeEstimatorRepredictor::AltitudeEstimatorRepredictor(
    const ros::NodeHandle &nh,
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const std::vector<var_alt_H_t> &H_multi,
    const var_alt_Q_t &Q,
    const std::vector<var_alt_R_t> &R_multi)
    :
    nh_(nh),
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
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".AltitudeEstimatorRepredictor()"
              << "): wrong size of \"Q.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".AltitudeEstimatorRepredictor()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_R_multi
  if (m_R_multi.size() != m_n_measurement_types) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".AltitudeEstimatorRepredictor()"
              << "): wrong size of \"m_R_multi\". Should be: " << m_n_measurement_types << " is:" << m_R_multi.size() << std::endl;
    return;
  }

  // Check size of m_R_multi elements
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    if (m_R_multi[i].rows() != ALT_P_MEASUREMENTS || m_R_multi[i].cols() != ALT_P_MEASUREMENTS) {
      std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".AltitudeEstimatorRepredictor()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (3, 3) is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")"
                << std::endl;
      return;
    }
  }

  //}

  // clang-format off
  /* m_A << 1, m_dt, m_dt_sq, */
  /*     0, 1, m_dt, */
  /*     0, 0, 1.0-m_b; */

  /* m_B << 0, 0, m_b; */

  // clang-format on

  // set measurement mapping matrix H to zero, it will be set later during each correction step
  var_alt_H_t m_H_zero = m_H_zero.Zero();

  /* mp_lkf = std::make_unique<var_lkf_alt_t>(m_A, m_B, m_H_zero); */
  /* mp_lkf = std::make_unique<var_lkf_alt_t>(generateA, generateB, m_H_zero); */
  mp_lkf = std::make_shared<var_lkf_alt_t>(generateA, generateB, m_H_zero);

  // Initialize all states to 0
  const var_alt_x_t        x0    = var_alt_x_t::Zero();
  var_alt_P_t              P_tmp = var_alt_P_t::Identity();
  const var_alt_P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
  const var_alt_statecov_t sc0({x0, P0});
  m_sc                 = sc0;
  const var_alt_u_t u0 = var_alt_u_t::Zero();
  const ros::Time   t0 = ros::Time(0);

  // Initialize repredictor
  mp_rep = std::make_unique<rep_t>(x0, P0, u0, Q, t0, mp_lkf, m_buf_sz);

  std::cout << "[AltitudeEstimatorRepredictor]: New AltitudeEstimatorRepredictor initialized " << std::endl;
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

  pub_state_ = nh_.advertise<mrs_msgs::Altitude>("altitude_state", 1);

  m_is_initialized = true;
}

//}


/*  //{ doPrediction() */

bool AltitudeEstimatorRepredictor::doPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  var_alt_u_t u = u.Zero();
  u(0)          = input;

  double      dtsq = pow(dt, 2) / 2;
  var_alt_A_t A    = m_A;

  A(0, 1) = dt;
  A(1, 2) = dt;

  A(0, 2) = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
      /* mp_lkf->A = A; */
      /* m_sc      = mp_lkf->predict(m_sc, u, m_Q, dt); */
      
      // Add input and predict
      // TODO use correct timestamps (cooresponding to time of the input and required time of prediction)
      // TODO do something about the dt parameter???
      mp_rep->addInput(u, m_Q, input_stamp);
      m_sc = mp_rep->predictTo(predict_stamp);

      /* mrs_msgs::Altitude msg; */
      /* msg.header.stamp = predict_stamp; */
      /* msg.height = m_sc.x(0); */
      /* msg.velocity = m_sc.x(1); */
      /* msg.acceleration = m_sc.x(2); */
      /* pub_state_.publish(msg); */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorRepredictor]: LKF prediction step failed: %s", e.what());
    }
  }
  return true;
}


//}

/*  //{ doPrediction() */

bool AltitudeEstimatorRepredictor::doPrediction(const double input) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doPrediction(const double input=" << input
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  //}

  var_alt_u_t u = u.Zero();
  u(0)          = input;

  double dt   = m_dt;
  double dtsq = m_dt_sq;

  var_alt_A_t A = m_A;
  A(0, 1)       = dt;
  A(1, 2)       = dt;

  A(0, 2) = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);
    try {
      // Apply the prediction step
      mp_lkf->A = A;
      /* m_sc      = mp_lkf->predict(m_sc, u, m_Q, dt); */
      // Add input and predict
      // TODO use correct timestamps (cooresponding to time of the input and required time of prediction)
      // TODO check what's happening with the dt and the A matrix
      mp_rep->addInput(u, m_Q, ros::Time::now());
      m_sc = mp_rep->predictTo(ros::Time::now());

    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorRepredictor]: LKF prediction step failed: %s", e.what());
    }
  }

  return true;
}

//}

/*  //{ doCorrection() */

bool AltitudeEstimatorRepredictor::doCorrection(const double &measurement, int measurement_type, ros::Time &meas_stamp, ros::Time &pred_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  // Prepare the measurement vector
  var_alt_z_t z;
  z << measurement;

  var_alt_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
  std::scoped_lock lock(mutex_lkf);
  {

    try {
      mp_lkf->H = m_H_multi[measurement_type];
      /* m_sc      = mp_lkf->correct(m_sc, z, m_R_multi[measurement_type]); */
      mp_rep->addMeasurement(z, m_R_multi[measurement_type], meas_stamp); // TODO use correct timestamp
      m_sc = mp_rep->predictTo(pred_stamp); // TODO modify odometry so that correction only adds measurement and prediction follows by getting new statecov???
      // TODO test that it works as before without repredictor when appropriate timestamps are used
      mrs_msgs::Altitude msg;
      msg.header.stamp = pred_stamp;
      msg.height = m_sc.x(0);
      msg.velocity = m_sc.x(1);
      msg.acceleration = m_sc.x(2);
      pub_state_.publish(msg);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorRepredictor]: LKF correction step failed: %s", e.what());
    }
  }

  return true;
}

//}

/*  //{ getStates() */

bool AltitudeEstimatorRepredictor::getStates(var_alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  x = m_sc.x;

  return true;
}

//}

/*  //{ getState() */

bool AltitudeEstimatorRepredictor::getState(int state_id, double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state_val=" << state_val
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    state_val = m_sc.x(state_id);
  }

  return true;
}

//}

/*  //{ getName() */

std::string AltitudeEstimatorRepredictor::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool AltitudeEstimatorRepredictor::setState(int state_id, const double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_val)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x(state_id) = state_val;
  }

  return true;
}

//}

/*  //{ setR() */

bool AltitudeEstimatorRepredictor::setR(double R, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(R)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"R\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (R <= 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): \"R\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  double old_R = m_R_multi[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = R;
  }

  std::cout << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setR(double R=" << R << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_R << " to: " << m_R_multi[measurement_type](0, 0) << std::endl;

  return true;
}

//}

/*  //{ getR() */

bool AltitudeEstimatorRepredictor::getR(double &R, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    R = m_R_multi[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getQ() */

bool AltitudeEstimatorRepredictor::getQ(double &cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_Q(idx(0), idx(1));
  }

  return true;
}

//}

/*  //{ setQ() */

bool AltitudeEstimatorRepredictor::setQ(double cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q(idx(0), idx(1)) = cov;
  }

  /* std::cout << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
   * << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ setInputCoeff() */

bool AltitudeEstimatorRepredictor::setInputCoeff(double coeff) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(coeff)) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): NaN detected in variable \"coeff\"."
              << std::endl;
    return false;
  }

  // Check for non-positive coefficient
  if (coeff <= 0) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be > 0."
              << std::endl;
    return false;
  }

  // Check for larger than 1 coefficient
  if (coeff >= 1) {
    std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be < 1."
              << std::endl;
    return false;
  }

  //}

  double old_coeff;

  {
    std::scoped_lock lock(mutex_lkf);
    old_coeff = m_B(2, 0);
    m_A(2, 2) = 1.0 - coeff;
    mp_lkf->A = m_A;
    m_B(2, 0) = coeff;
    mp_lkf->B = m_B;
  }

  std::cout << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff
            << " Changed input coefficient from: " << old_coeff << " to: " << coeff << std::endl;

  return true;
}

//}

/*  //{ getCovariance() */

bool AltitudeEstimatorRepredictor::getCovariance(var_alt_P_t &P) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    P = m_sc.P;
  }

  return true;
}

//}

/*  //{ setCovariance() */

bool AltitudeEstimatorRepredictor::setCovariance(const var_alt_P_t &P) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < m_n_states; i++) {
    if (!std::isfinite(P(i, i))) {
      ROS_ERROR_STREAM("[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &P=" << P
                                                          << "): NaN detected in variable \"P(" << i << "," << i << ")\".");
      return false;
    }
  }

  //}

  // Set the covariance
  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.P = P;
  }

  return true;
}
//}

/*  //{ reset() */

bool AltitudeEstimatorRepredictor::reset(const var_alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (!std::isfinite(x(i, j))) {
        std::cerr << "[AltitudeEstimatorRepredictor]: " << m_estimator_name << ".reset(const var_alt_x_t &x="  // << x
                  << "): NaN detected in variable \"x(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x = (x);
  }

  return true;
}

//}
}  // namespace mrs_uav_odometry
