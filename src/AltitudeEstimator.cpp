#include "AltitudeEstimator.h"

namespace mrs_uav_odometry
{

/*  //{ AltitudeEstimator() */

// clang-format off
AltitudeEstimator::AltitudeEstimator()
{
  // empty contructor for child classes
}

//}

/*  //{ AltitudeEstimator() */

// clang-format off
AltitudeEstimator::AltitudeEstimator(
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const std::vector<alt_H_t> &H_multi,
    const alt_Q_t &Q,
    const std::vector<alt_R_t> &R_multi,
    const bool use_repredictor)
    :
    m_estimator_name(estimator_name),
    m_fusing_measurement(fusing_measurement),
    m_H_multi(H_multi),
    m_Q(Q),
    m_R_multi(R_multi),
    m_use_repredictor(use_repredictor)
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
    if (m_R_multi[i].rows() != ALT_P_MEASUREMENTS || m_R_multi[i].cols() != ALT_P_MEASUREMENTS) {
      std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".AltitudeEstimator()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (3, 3) is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")"
                << std::endl;
      return;
    }
  }

  //}

  // clang-format off
  m_A << 1, m_dt, m_dt_sq,
      0, 1, m_dt,
      0, 0, 1.0-m_b;

  m_B << 0, 0, m_b;

  // clang-format on

  // set measurement mapping matrix H to zero, it will be set later during each correction step
  alt_H_t m_H_zero = m_H_zero.Zero();

  // Initialize all states to 0
  const alt_x_t        x0    = alt_x_t::Zero();
  alt_P_t              P_tmp = alt_P_t::Identity();
  const alt_P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
  const alt_statecov_t sc0({x0, P0});
  m_sc                 = sc0;
  const var_alt_u_t u0 = alt_u_t::Zero();
  const ros::Time   t0 = ros::Time(0);

  if (m_use_repredictor) {
    std::cout << "[AltitudeEstimator]: Using repredictor in " << m_estimator_name.c_str() << " estimator." << std::endl;
    // Lambda functions generating A and B matrices based on dt
    auto generateA = [](const double dt) {
      var_alt_A_t A;
      A << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1.0 - ALT_INPUT_COEFF;
      return A;
    };
    auto generateB = []([[maybe_unused]] const double dt) {
      var_alt_B_t B;
      B << 0, 0, ALT_INPUT_COEFF;
      return B;
    };
    // Initialize separate LKF models for each H matrix
    for (size_t i = 0; i < m_H_multi.size(); i++) {
      mp_lkf_vector.push_back(std::make_shared<var_lkf_alt_t>(generateA, generateB, m_H_multi[i]));
    }
    // Initialize repredictor
    mp_rep = std::make_unique<rep_t>(x0, P0, u0, Q, t0, mp_lkf_vector.at(0), m_buf_sz);
  } else {
    // Initialize a single LKF
    mp_lkf = std::make_unique<lkf_alt_t>(m_A, m_B, m_H_zero);
  }

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

bool AltitudeEstimator::doPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

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
  u(0)      = input;

  double  dtsq = pow(dt, 2) / 2;
  alt_A_t A    = m_A;

  A(0, 1) = dt;
  A(1, 2) = dt;

  A(0, 2) = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
      if (m_use_repredictor) {
        mp_rep->addInputChangeWithNoise(u, m_Q, input_stamp, mp_lkf_vector[0]);
        m_sc = mp_rep->predictTo(predict_stamp);
      } else {
        mp_lkf->A = A;
        m_sc      = mp_lkf->predict(m_sc, u, m_Q, dt);
      }
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

bool AltitudeEstimator::doPrediction(const double input, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doPrediction(const double input=" << input << "): NaN detected in variable \"input\"."
              << std::endl;
    return false;
  }

  //}

  alt_u_t u = u.Zero();
  u(0)      = input;

  double dt   = m_dt;
  double dtsq = m_dt_sq;

  alt_A_t A = m_A;
  A(0, 1)   = dt;
  A(1, 2)   = dt;

  A(0, 2) = dtsq;

  {
    std::scoped_lock lock(mutex_lkf);
    try {
      // Apply the prediction step
      if (m_use_repredictor) {
        // add new input to repredictor (pass pointer to a lkf model so that it uses the correct model even after altitude coeff change)
        mp_rep->addInputChangeWithNoise(u, m_Q, input_stamp, mp_lkf_vector[0]);
        m_sc = mp_rep->predictTo(predict_stamp);
      } else {
        mp_lkf->A = A;
        m_sc      = mp_lkf->predict(m_sc, u, m_Q, dt);
      }
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

bool AltitudeEstimator::doCorrection(const double &measurement, int measurement_type, const ros::Time &meas_stamp, const ros::Time &predict_stamp,
                                     const std::string &measurement_name) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  // skip non-height measurements before the first height measurement if using repredictor
  if (m_use_repredictor && !m_got_height_measurement) {
    if (m_H_multi[measurement_type](0, 0) == 0) {
      ROS_WARN_THROTTLE(1.0, "[AltitudeEstimator]: Skipping non-height measurements before receiving the first height measurement.");
      return false;
    } else {
      m_got_height_measurement = true;
    }
  }

  // Prepare the measurement vector
  alt_z_t z;
  z << measurement;

  alt_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
  std::scoped_lock lock(mutex_lkf);
  {

    try {
      if (m_use_repredictor) {

        mp_rep->addMeasurement(z, m_R_multi[measurement_type], meas_stamp, mp_lkf_vector[measurement_type]);
        m_sc = mp_rep->predictTo(predict_stamp);
      } else {
        mp_lkf->H = m_H_multi[measurement_type];
        m_sc      = mp_lkf->correct(m_sc, z, m_R_multi[measurement_type]);
      }
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

bool AltitudeEstimator::getStates(alt_x_t &x) {

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

bool AltitudeEstimator::getState(int state_id, double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state_val=" << state_val
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

std::string AltitudeEstimator::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool AltitudeEstimator::setState(int state_id, const double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_val)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x(state_id) = state_val;
    // reset repredictor
    if (m_use_repredictor) {
      const var_alt_u_t u0 = alt_u_t::Zero();
      const ros::Time   t0 = ros::Time(0);
      mp_rep               = std::make_unique<rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
    }
  }

  return true;
}

//}

/*  //{ setR() */

bool AltitudeEstimator::setR(double R, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(R)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"R\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (R <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): \"R\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  double old_R = m_R_multi[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = R;
  }

  std::cout << "[AltitudeEstimator]: " << m_estimator_name << ".setR(double R=" << R << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_R << " to: " << m_R_multi[measurement_type](0, 0) << std::endl;

  return true;
}

//}

/*  //{ getR() */

bool AltitudeEstimator::getR(double &R, int measurement_type) {

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

    R = m_R_multi[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getQ() */

bool AltitudeEstimator::getQ(double &cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
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

bool AltitudeEstimator::setQ(double cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q(idx(0), idx(1)) = cov;
  }

  /* std::cout << "[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ setInputCoeff() */

bool AltitudeEstimator::setInputCoeff(double coeff) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(coeff)) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): NaN detected in variable \"coeff\"."
              << std::endl;
    return false;
  }

  // Check for non-positive coefficient
  if (coeff <= 0) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be > 0." << std::endl;
    return false;
  }

  // Check for larger than 1 coefficient
  if (coeff >= 1) {
    std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be < 1." << std::endl;
    return false;
  }

  //}

  double old_coeff;

  {
    std::scoped_lock lock(mutex_lkf);
    old_coeff = m_B(2, 0);
    if (m_use_repredictor) {
      // Lambda functions generating A and B matrices based on dt
      auto generateA = [=](const double dt) {
        var_alt_A_t A;
        A << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1.0 - coeff;
        return A;
      };
      auto generateB = [=]([[maybe_unused]] const double dt) {
        var_alt_B_t B;
        B << 0, 0, coeff;
        return B;
      };
      // Clear lkf vector
      mp_lkf_vector.clear();
      // Reinitialize separate LKF models for each H matrix
      for (size_t i = 0; i < m_H_multi.size(); i++) {
        mp_lkf_vector.push_back(std::make_shared<var_lkf_alt_t>(generateA, generateB, m_H_multi[i]));
      }
    } else {
      m_A(2, 2) = 1.0 - coeff;
      mp_lkf->A = m_A;
      m_B(2, 0) = coeff;
      mp_lkf->B = m_B;
    }
  }

  std::cout << "[AltitudeEstimator]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << " Changed input coefficient from: " << old_coeff
            << " to: " << coeff << std::endl;

  return true;
}

//}

/*  //{ getCovariance() */

bool AltitudeEstimator::getCovariance(alt_P_t &P) {

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

bool AltitudeEstimator::setCovariance(const alt_P_t &P) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < m_n_states; i++) {
    if (!std::isfinite(P(i, i))) {
      ROS_ERROR_STREAM("[AltitudeEstimator]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &P=" << P << "): NaN detected in variable \"P(" << i
                                               << "," << i << ")\".");
      return false;
    }
  }

  //}

  // Set the covariance
  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.P = P;
    // reset repredictor
    if (m_use_repredictor) {
      const var_alt_u_t u0 = alt_u_t::Zero();
      const ros::Time   t0 = ros::Time(0);
      mp_rep               = std::make_unique<rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
    }
  }

  return true;
}
//}

/*  //{ reset() */

bool AltitudeEstimator::reset(const alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (!std::isfinite(x(i, j))) {
        std::cerr << "[AltitudeEstimator]: " << m_estimator_name << ".reset(const alt_x_t &x="  // << x
                  << "): NaN detected in variable \"x(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x = (x);
    if (m_use_repredictor) {
      const var_alt_u_t u0 = alt_u_t::Zero();
      const ros::Time   t0 = ros::Time(0);
      mp_rep               = std::make_unique<rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
    }
  }

  return true;
}

//}
}  // namespace mrs_uav_odometry
