#include "AltitudeEstimatorAloamGarm.h"

namespace mrs_uav_odometry
{

/*  //{ AltitudeEstimatorAloamGarm() */

// clang-format off
AltitudeEstimatorAloamGarm::AltitudeEstimatorAloamGarm(
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const std::vector<var_alt_H_t> &H_multi,
    const var_alt_Q_t &Q,
    const std::vector<var_alt_R_t> &R_multi,
    const ros::NodeHandle &nh,
    const bool use_repredictor)
    :
    AltitudeEstimator(),
    m_estimator_name(estimator_name),
    m_fusing_measurement(fusing_measurement),
    m_H_multi_orig(H_multi),
    m_Q_orig(Q),
    m_R_multi(R_multi),
    m_nh(nh),
    m_use_repredictor(use_repredictor)
  {

  // clang-format on

  // TODO do not hardcode this?
  // add columns for measurement biases
  for (auto H : m_H_multi_orig) {
    algarm_alt_H_t H_new;
    H_new.block(0, 0, 1, 3) = H;
    m_H_multi.push_back(H_new);
  }
  algarm_alt_H_t H_garmin_biased;
  H_garmin_biased << 1, 0, 0, -1, 0, 0;
  m_H_multi.push_back(H_garmin_biased);
  m_garmin_biased_id = m_H_multi.size() - 1;

  algarm_alt_H_t H_aloam_biased;
  H_aloam_biased << 1, 0, 0, 0, -1, 0;
  m_H_multi.push_back(H_aloam_biased);
  m_aloam_biased_id = m_H_multi.size() - 1;

  /* algarm_alt_H_t H_garmin_bias_only; */
  /* H_garmin_bias_only << 0, 0, 0, 1, 0, 0; */
  /* m_H_multi.push_back(H_garmin_bias_only); */
  /* m_garmin_bias_only_id = m_H_multi.size() - 1; */

  algarm_alt_H_t H_baro_biased;
  H_baro_biased << 0, 1, 0, 0, 0, 1;
  m_H_multi.push_back(H_baro_biased);
  m_baro_biased_id = m_H_multi.size() - 1;

  int orig_rows                     = m_Q_orig.rows();
  int orig_cols                     = m_Q_orig.cols();
  m_Q.block(0, 0, 3, 3)             = m_Q_orig.block(0, 0, 3, 3);
  m_Q(orig_rows, orig_cols)         = 1;
  m_Q(orig_rows + 1, orig_cols + 1) = 1;
  m_Q(orig_rows + 2, orig_cols + 2) = 1;

  // Number of states
  m_n_states = m_A.rows();

  // Number of measurement types
  m_n_measurement_types = m_fusing_measurement.size();

  /*  //{ sanity checks */

  // Check size of m_Q
  if (m_Q.rows() != m_n_states) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".AltitudeEstimatorAloamGarm()"
              << "): wrong size of \"Q.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".AltitudeEstimatorAloamGarm()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_R_multi
  if (m_R_multi.size() != m_n_measurement_types) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".AltitudeEstimatorAloamGarm()"
              << "): wrong size of \"m_R_multi\". Should be: " << m_n_measurement_types << " is:" << m_R_multi.size() << std::endl;
    return;
  }

  // Check size of m_R_multi elements
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    if (m_R_multi[i].rows() != ALT_P_MEASUREMENTS || m_R_multi[i].cols() != ALT_P_MEASUREMENTS) {
      std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".AltitudeEstimatorAloamGarm()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (3, 3) is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")"
                << std::endl;
      return;
    }
  }

  //}


  // set measurement mapping matrix H to zero, it will be set later during each correction step
  algarm_alt_H_t m_H_zero = m_H_zero.Zero();

  // Initialize all states to 0
  const algarm_alt_x_t        x0    = algarm_alt_x_t::Zero();
  algarm_alt_P_t              P_tmp = algarm_alt_P_t::Identity();
  const algarm_alt_P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
  const algarm_alt_statecov_t sc0({x0, P0});
  m_sc                    = sc0;
  const algarm_alt_u_t u0 = algarm_alt_u_t::Zero();
  const ros::Time      t0 = ros::Time(0);

  std::cout << "[AltitudeEstimatorAloamGarm]: Using repredictor in " << m_estimator_name.c_str() << " estimator." << std::endl;
  // Lambda functions generating A and B matrices based on dt
  auto generateA = [](const double dt) {
    algarm_alt_A_t A;
    A << 1, dt, dt * dt / 2, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1.0 - ALT_INPUT_COEFF, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    return A;
  };
  auto generateB = []([[maybe_unused]] const double dt) {
    algarm_alt_B_t B;
    B << 0, 0, ALT_INPUT_COEFF, 0, 0, 0;
    return B;
  };
  // Initialize separate LKF models for each H matrix
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    mp_lkf_vector.push_back(std::make_shared<algarm_alt_t>(generateA, generateB, m_H_multi[i]));
  }
  // Initialize repredictor
  mp_rep = std::make_unique<algarm_rep_t>(x0, P0, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);

  /* std::cout << "aloamgarm - t0: " << t0 << "P0: " << P0 << "x0: " << x0 << "m_Q: " << m_Q << std::endl; */

  std::cout << "[AltitudeEstimatorAloamGarm]: New AltitudeEstimatorAloamGarm initialized " << std::endl;
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

  int mf_buffer_size = 10;
  /* float mf_max_diff     = 1.0; */
  float mf_max_diff        = 0.1;
  m_median_filter          = std::make_unique<MedianFilter>(mf_buffer_size, 50, 0, mf_max_diff);
  debug_state_publisher    = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_state", 1);
  debug_cov_publisher      = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_cov", 1);
  debug_Q_publisher        = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_Q", 1);
  debug_duration_publisher = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_duration", 1);
  debug_aloam_ok_publisher = m_nh.advertise<mrs_msgs::BoolStamped>("debug_aloamgarm_aloam_ok", 1);

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool AltitudeEstimatorAloamGarm::doPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doPrediction(const double &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  ros::WallTime time_beginning = ros::WallTime::now();

  algarm_alt_u_t u = u.Zero();
  u(0)             = input;

  algarm_alt_Q_t Q = m_Q;
  if (m_aloam_ok) {
    Q(3, 3) = m_Q(3, 3) * 10e3;
  }

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
      mrs_msgs::Float64ArrayStamped msg;
      msg.header.stamp = input_stamp;
      for (int i = 0; i < Q.size(); i++) {
        msg.values.push_back(Q(i));
      }
      debug_Q_publisher.publish(msg);
      mp_rep->addInputChangeWithNoise(u, Q, input_stamp, mp_lkf_vector[0]);
      m_sc = mp_rep->predictTo(predict_stamp);
      /* std::cout << "u: " << u << std::endl; */
      /* std::cout << "Q: " << Q << std::endl; */
      /* std::cout << "m_Q: " << m_Q << std::endl; */
      /* ROS_WARN("prediction, m_sc.P(0): %.3f, input stamp: %.6f, predict stamp: %.6f", m_sc.P(0), input_stamp.toSec(), predict_stamp.toSec()); */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorAloamGarm]: LKF prediction step failed: %s", e.what());
    }
  }

  ros::WallTime                 time_end = ros::WallTime::now();
  double                        diff     = (time_end.toSec() - time_beginning.toSec()) * 1000;
  if(diff > 10){
    ROS_WARN("[AltitudeEstimatorAloamGarm] Correction took %.2f ms (longer than 10 ms).", diff);
  }
  mrs_msgs::Float64ArrayStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.values.push_back(diff);
  debug_duration_publisher.publish(msg);
  /* ROS_INFO("reprediction took %.9f ms", diff); */

  return true;
}


//}

/*  //{ doPrediction() */

bool AltitudeEstimatorAloamGarm::doPrediction(const double input, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(input)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doPrediction(const double input=" << input
              << "): NaN detected in variable \"input\"." << std::endl;
    return false;
  }

  //}

  algarm_alt_u_t u = u.Zero();
  u(0)             = input;

  algarm_alt_Q_t Q = m_Q;
  if (m_aloam_ok) {
    Q(3, 3) = m_Q(3, 3) * 10e3;
  }

  {
    std::scoped_lock lock(mutex_lkf);
    try {
      // Apply the prediction step
      // add new input to repredictor (pass pointer to a lkf model so that it uses the correct model even after altitude coeff change)
      mrs_msgs::Float64ArrayStamped msg;
      msg.header.stamp = input_stamp;
      for (int i = 0; i < Q.size(); i++) {
        msg.values.push_back(Q(i));
      }
      debug_Q_publisher.publish(msg);
      mp_rep->addInputChangeWithNoise(u, Q, input_stamp, mp_lkf_vector[0]);
      m_sc = mp_rep->predictTo(predict_stamp);
      /* ROS_WARN("prediction, m_sc.P(0): %.3f", m_sc.P(0)); */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorAloamGarm]: LKF prediction step failed: %s", e.what());
    }
  }

  return true;
}

//}

/*  //{ doCorrection() */

bool AltitudeEstimatorAloamGarm::doCorrection(const double &measurement, int measurement_type, const ros::Time &meas_stamp, const ros::Time &predict_stamp,
                                              const std::string &measurement_name, const double &aloam_eigenvalue) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  ros::WallTime time_beginning = ros::WallTime::now();

  // Prepare the measurement vector
  algarm_alt_z_t z;
  z << measurement;

  int lkf_id = measurement_type;
  if (measurement_name == "height_range") {
    lkf_id = m_garmin_biased_id;
  } else if (measurement_name == "height_aloam") {
    lkf_id = m_aloam_biased_id;
  } else if (measurement_name == "vel_baro") {
    lkf_id = m_baro_biased_id;
  }

  algarm_alt_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
  std::scoped_lock lock(mutex_lkf);
  {

    // TODO some better solution? (bigger difference from median allowed when close to the ground)
    if (measurement_name == "height_range" && !m_median_filter->isValid(z(0)) && m_median_filter->isFilled() &&
        (z(0) > 0.6 || fabs(m_median_filter->getMedian() - z(0)) > 0.5)) {
      // set H matrix so that only garmin bias is updated
      /* lkf_id = m_garmin_bias_only_id; */
      // calculate new bias measurement
      /* z(0) = m_sc.x(0) - z(0); */
      ROS_WARN("Garmin jump detected, old bias: %.2f, new calculated bias: %.2f.", m_sc.x(3), z(0));
      std::cout << "H: " << mp_lkf_vector[lkf_id]->H << std::endl;
      std::cout << "x: " << m_sc.x << std::endl;
      std::cout << "P: " << m_sc.P << std::endl;
      std::cout << "orig R: " << R << std::endl;
      /* R(0)             = 1000000; */
      R(0) = R(0) * 10e4;
      std::cout << "new R: " << R << std::endl;
      algarm_alt_u_t u = u.Zero();
      algarm_alt_Q_t Q = m_Q;
      std::cout << "orig Q: " << Q << std::endl;
      /* Q(3, 3)          = 1000000000; */
      /* Q(3, 3)          = m_Q(3,3) * 1000000000; */
      Q(3, 3) = m_Q(3, 3) * 10e9;
      std::cout << "new Q: " << Q << std::endl;

      mrs_msgs::Float64ArrayStamped msg;
      msg.header.stamp = meas_stamp;
      for (int i = 0; i < Q.size(); i++) {
        msg.values.push_back(Q(i));
      }
      debug_Q_publisher.publish(msg);
      mp_rep->addInputChangeWithNoise(u, Q, meas_stamp, mp_lkf_vector[0]);
    }

    // hysteresis to prevent rapid switching between the 2 states
    if (measurement_name == "height_aloam" && m_aloam_ok && aloam_eigenvalue < 100) {
      m_aloam_ok = false;
    } else if (measurement_name == "height_aloam" && !m_aloam_ok && aloam_eigenvalue > 120) {
      m_aloam_ok = true;
    }

    if (measurement_name == "height_aloam") {
      m_aloam_eig = aloam_eigenvalue;
    }

    if (measurement_name == "height_aloam" && !m_aloam_ok) {
      // set H matrix so that only garmin bias is updated
      /* lkf_id = m_aloam_bias_only_id; */
      // calculate new bias measurement
      // TODO zkusit korekci primo s puvodni matici H
      /* z(0) = m_sc.x(0) - z(0);  // TODO from wrong time */
      /* ROS_WARN("Shitty aloam detected, old bias: %.2f, new calculated bias: %.2f.", m_sc.x(3), z(0)); */
      /* std::cout << "H: " << mp_lkf_vector[lkf_id]->H << std::endl; */
      /* std::cout << "x: " << m_sc.x << std::endl; */
      /* std::cout << "P: " << m_sc.P << std::endl; */
      /* R(0)             = 1000; */
      R(0)             = R(0) * 10e2;
      algarm_alt_u_t u = u.Zero();
      algarm_alt_Q_t Q = m_Q;
      /* Q(4, 4)          = 1000000; */
      /* Q(4, 4)          = m_Q(4,4)*1000000; */
      Q(4, 4) = m_Q(4, 4) * 10e6;

      mrs_msgs::Float64ArrayStamped msg;
      msg.header.stamp = meas_stamp;
      for (int i = 0; i < Q.size(); i++) {
        msg.values.push_back(Q(i));
      }
      debug_Q_publisher.publish(msg);
      mp_rep->addInputChangeWithNoise(u, Q, meas_stamp, mp_lkf_vector[0]);

      mrs_msgs::BoolStamped msg_aloam_ok;
      msg_aloam_ok.stamp = meas_stamp;  // TODO why not header.stamp???
      msg_aloam_ok.data  = false;
      debug_aloam_ok_publisher.publish(msg_aloam_ok);
    } else if (measurement_name == "height_aloam") {
      mrs_msgs::BoolStamped msg_aloam_ok;
      msg_aloam_ok.stamp = meas_stamp;  // TODO why not header.stamp???
      msg_aloam_ok.data  = true;
      debug_aloam_ok_publisher.publish(msg_aloam_ok);
    }

    try {
      mp_rep->addMeasurement(z, R, meas_stamp, mp_lkf_vector[lkf_id]);
      m_sc = mp_rep->predictTo(predict_stamp);
      /* std::cout << "z: " << z << std::endl; */
      /* std::cout << "R: " << R << std::endl; */
      /* ROS_WARN("correction, m_sc.P(0): %.3f, meas stamp: %.6f, predict stamp: %.6f", m_sc.P(0), meas_stamp.toSec(), predict_stamp.toSec()); */

      mrs_msgs::Float64ArrayStamped msg;
      msg.header.stamp = predict_stamp;
      for (int i = 0; i < m_sc.x.size(); i++) {
        msg.values.push_back(m_sc.x(i));
      }
      debug_state_publisher.publish(msg);

      mrs_msgs::Float64ArrayStamped msg_cov;
      msg_cov.header.stamp = predict_stamp;
      for (int i = 0; i < m_sc.P.size(); i++) {
        msg_cov.values.push_back(m_sc.P(i));
      }
      debug_cov_publisher.publish(msg_cov);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorAloamGarm]: LKF correction step failed: %s", e.what());
    }
  }

  ros::WallTime                 time_end = ros::WallTime::now();
  double                        diff     = (time_end.toSec() - time_beginning.toSec()) * 1000;
  if(diff > 10){
    ROS_WARN("[AltitudeEstimatorAloamGarm] Correction took %.2f ms (longer than 10 ms).", diff);
  }
  mrs_msgs::Float64ArrayStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.values.push_back(diff);
  // Differentiate duration when correction from ALOAM is used
  if (measurement_name == "height_aloam") {
    msg.values.push_back(diff);
  }
  debug_duration_publisher.publish(msg);

  return true;
}

//}

/*  //{ getStates() */

bool AltitudeEstimatorAloamGarm::getStates(algarm_alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  x = m_sc.x;

  return true;
}

//}

/*  //{ getStates() */

bool AltitudeEstimatorAloamGarm::getStates(alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  // TODO return only position, velocity and acc - backward compatibility
  x = m_sc.x.block(0, 0, 3, 1);

  return true;
}

//}

/*  //{ getState() */

bool AltitudeEstimatorAloamGarm::getState(int state_id, double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", double &state_val=" << state_val
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

std::string AltitudeEstimatorAloamGarm::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool AltitudeEstimatorAloamGarm::setState(int state_id, const double &state_val) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_val)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const double &state=" << state_val
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x(state_id) = state_val;
    // reset repredictor
    const algarm_alt_u_t u0 = algarm_alt_u_t::Zero();
    const ros::Time      t0 = ros::Time(0);
    mp_rep                  = std::make_unique<algarm_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
  }

  return true;
}

//}

/*  //{ setR() */

bool AltitudeEstimatorAloamGarm::setR(double R, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(R)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"R\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (R <= 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): \"R\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setcovariance(double R=" << R << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  double old_R = m_R_multi[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = R;
  }

  std::cout << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setR(double R=" << R << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_R << " to: " << m_R_multi[measurement_type](0, 0) << std::endl;

  return true;
}

//}

/*  //{ getR() */

bool AltitudeEstimatorAloamGarm::getR(double &R, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
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

bool AltitudeEstimatorAloamGarm::getQ(double &cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
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

bool AltitudeEstimatorAloamGarm::setQ(double cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q(idx(0), idx(1)) = cov;
  }

  /* std::cout << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
   * << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ setInputCoeff() */

bool AltitudeEstimatorAloamGarm::setInputCoeff(double coeff) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(coeff)) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): NaN detected in variable \"coeff\"."
              << std::endl;
    return false;
  }

  // Check for non-positive coefficient
  if (coeff <= 0) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be > 0." << std::endl;
    return false;
  }

  // Check for larger than 1 coefficient
  if (coeff >= 1) {
    std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff << "): \"coeff\" should be < 1." << std::endl;
    return false;
  }

  //}

  double old_coeff;

  {
    std::scoped_lock lock(mutex_lkf);
    old_coeff = m_B(2, 0);
    // Lambda functions generating A and B matrices based on dt
    auto generateA = [=](const double dt) {
      algarm_alt_A_t A;
      A << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1.0 - coeff;
      return A;
    };
    auto generateB = [=]([[maybe_unused]] const double dt) {
      algarm_alt_B_t B;
      B << 0, 0, coeff;
      return B;
    };
    // Clear lkf vector
    mp_lkf_vector.clear();
    // Reinitialize separate LKF models for each H matrix
    for (size_t i = 0; i < m_H_multi.size(); i++) {
      mp_lkf_vector.push_back(std::make_shared<algarm_alt_t>(generateA, generateB, m_H_multi[i]));
    }
  }

  std::cout << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff
            << " Changed input coefficient from: " << old_coeff << " to: " << coeff << std::endl;

  return true;
}

//}

/*  //{ getCovariance() */

bool AltitudeEstimatorAloamGarm::getCovariance(algarm_alt_P_t &P) {

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

bool AltitudeEstimatorAloamGarm::setCovariance(const algarm_alt_P_t &P) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < m_n_states; i++) {
    if (!std::isfinite(P(i, i))) {
      ROS_ERROR_STREAM("[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &P=" << P
                                                        << "): NaN detected in variable \"P(" << i << "," << i << ")\".");
      return false;
    }
  }

  //}

  // Set the covariance
  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.P = P;
    // reset repredictor
    const algarm_alt_u_t u0 = algarm_alt_u_t::Zero();
    const ros::Time      t0 = ros::Time(0);
    mp_rep                  = std::make_unique<algarm_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
  }

  return true;
}
//}

/*  //{ reset() */

bool AltitudeEstimatorAloamGarm::reset(const algarm_alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (!std::isfinite(x(i, j))) {
        std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".reset(const algarm_alt_x_t &x="  // << x
                  << "): NaN detected in variable \"x(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x                  = (x);
    const algarm_alt_u_t u0 = algarm_alt_u_t::Zero();
    const ros::Time      t0 = ros::Time(0);
    mp_rep                  = std::make_unique<algarm_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
  }

  return true;
}

//}
}  // namespace mrs_uav_odometry
