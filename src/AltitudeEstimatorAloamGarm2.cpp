#include "AltitudeEstimatorAloamGarm2.h"
#include "mrs_msgs/Float64ArrayStamped.h"

namespace mrs_uav_odometry
{

/*  //{ AltitudeEstimatorAloamGarm2() */

// clang-format off
AltitudeEstimatorAloamGarm2::AltitudeEstimatorAloamGarm2(
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
  // load parameters
  mrs_lib::ParamLoader param_loader(m_nh, "Odometry");
  param_loader.loadParam("aloamgarm/mf_changes_buffer_size", _mf_changes_buffer_size_);
  param_loader.loadParam("aloamgarm/mf_changes_max_diff", _mf_changes_max_diff_);
  param_loader.loadParam("aloamgarm/mf_changes_max_diff_close_to_ground", _mf_changes_max_diff_close_to_ground_);
  param_loader.loadParam("aloamgarm/mf_close_to_ground_threshold", _mf_close_to_ground_threshold_);
  param_loader.loadParam("aloamgarm/q_factor_range_bias_slam_ok", _q_factor_range_bias_slam_ok_);
  param_loader.loadParam("aloamgarm/q_factor_range_bias_range_jump", _q_factor_range_bias_range_jump_);
  param_loader.loadParam("aloamgarm/q_factor_slam_bias", _q_factor_slam_bias_);
  param_loader.loadParam("aloamgarm/r_factor_range_jump", _r_factor_range_jump_);
  param_loader.loadParam("aloamgarm/r_factor_slam_bad", _r_factor_slam_bad_);
  param_loader.loadMatrixStatic("aloamgarm/H_range_biased", _H_range_biased_);
  param_loader.loadMatrixStatic("aloamgarm/H_slam_biased", _H_slam_biased_);
  param_loader.loadMatrixStatic("aloamgarm/H_baro_biased", _H_baro_biased_);
  param_loader.loadParam("aloamgarm/q_biases", _q_biases_);
  param_loader.loadParam("aloamgarm/biased_state_count", _biased_state_count_);
  param_loader.loadParam("aloamgarm/eigenvalue_hysteresis_upper", _eigenvalue_hysteresis_upper_);
  param_loader.loadParam("aloamgarm/eigenvalue_hysteresis_lower", _eigenvalue_hysteresis_lower_);
  param_loader.loadParam("aloamgarm/repredictor_buffer_size", _repredictor_buffer_size_);
  param_loader.loadParam("aloamgarm/debug", _debug_);
  param_loader.loadParam("aloamgarm/kernel_sigma", _kernel_sigma_);
  param_loader.loadParam("aloamgarm/nis_buffer_size", _nis_buffer_size_);
  param_loader.loadParam("aloamgarm/nis_threshold", _nis_threshold_);
  param_loader.loadParam("aloamgarm/nis_avg_threshold", _nis_avg_threshold_);
  param_loader.loadMatrixStatic("aloamgarm/initial_state", _initial_state_);
  param_loader.loadMatrixStatic("aloamgarm/initial_cov", _initial_cov_);
  param_loader.loadParam("aloamgarm/initial_time", _initial_time_);
  _initial_time_stamp_ = ros::Time(_initial_time_);
  param_loader.loadParam("aloamgarm/use_initial_conditions", _use_initial_conditions_);

  // add columns for measurement biases
  for (auto H : m_H_multi_orig) {
    algarm2_alt_H_t H_new   = H_new.Zero();
    H_new.block(0, 0, 1, 3) = H;
    m_H_multi.push_back(H_new);
  }

  // add H matrices for biased states
  m_H_multi.push_back(_H_range_biased_);
  m_garmin_biased_id = m_H_multi.size() - 1;

  m_H_multi.push_back(_H_slam_biased_);
  m_aloam_biased_id = m_H_multi.size() - 1;

  m_H_multi.push_back(_H_baro_biased_);
  m_baro_biased_id = m_H_multi.size() - 1;

  // add bias process noise to Q matrix
  m_Q                   = m_Q.Zero();
  int orig_rows         = m_Q_orig.rows();
  int orig_cols         = m_Q_orig.cols();
  m_Q.block(0, 0, 3, 3) = m_Q_orig.block(0, 0, 3, 3);
  for (int i = 0; i < _biased_state_count_; i++) {
    m_Q(orig_rows + i, orig_cols + i) = _q_biases_;
  }
  m_Q(5, 5) = 1.0;
  std::cout << "m_Q: " << m_Q << std::endl;

  // Lambda functions generating A and B matrices based on dt
  auto generateA = [](const double dt) {
    algarm2_alt_A_t A;
    A << 1, dt, dt * dt / 2, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1.0 - ALT_INPUT_COEFF, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    return A;
  };
  m_A            = generateA(0.01);
  auto generateB = []([[maybe_unused]] const double dt) {
    algarm2_alt_B_t B;
    B << 0, 0, ALT_INPUT_COEFF, 0, 0, 0;
    return B;
  };
  m_B = generateB(0.01);

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
  algarm2_alt_H_t m_H_zero = m_H_zero.Zero();

  // Initialize all states to 0
  const algarm2_alt_x_t        x0    = algarm2_alt_x_t::Zero();
  algarm2_alt_P_t              P_tmp = algarm2_alt_P_t::Identity();
  const algarm2_alt_P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
  const algarm2_alt_statecov_t sc0({x0, P0});
  m_sc                     = sc0;
  const algarm2_alt_u_t u0 = algarm2_alt_u_t::Zero();
  const ros::Time       t0 = ros::Time(0);

  std::cout << "[AltitudeEstimatorAloamGarm]: Using repredictor in " << m_estimator_name.c_str() << " estimator." << std::endl;
  // Initialize separate LKF models for each H matrix
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    mp_lkf_vector.push_back(std::make_shared<algarm2_alt_t>(generateA, generateB, m_H_multi[i], _kernel_sigma_, m_nh, _nis_threshold_, _nis_avg_threshold_));
  }

  std::shared_ptr<boost::circular_buffer<double>> nis_buffer = std::make_shared<boost::circular_buffer<double>>(_nis_buffer_size_);
  // Initialize repredictor
  if (!_use_initial_conditions_) {
    mp_rep = std::make_unique<algarm2_rep_t>(x0, P0, u0, m_Q, t0, mp_lkf_vector.at(0), _repredictor_buffer_size_, nis_buffer);
  } else {
    mp_rep = std::make_unique<algarm2_rep_t>(_initial_state_, _initial_cov_, u0, m_Q, _initial_time_stamp_, mp_lkf_vector.at(0), _repredictor_buffer_size_,
                                             nis_buffer);
  }

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

  m_median_filter = std::make_unique<MedianFilter>(_mf_changes_buffer_size_, 100, 0, _mf_changes_max_diff_);

  if (_debug_) {
    debug_state_publisher = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_state", 1);
    debug_cov_publisher   = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_cov", 1);
    debug_Q_publisher     = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_Q", 1);
    /* debug_duration_publisher = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_duration", 1); */
    debug_duration_publisher = m_nh.advertise<mrs_msgs::AloamgarmDebug>("debug_aloamgarm_duration", 1);
    debug_aloam_ok_publisher = m_nh.advertise<mrs_msgs::BoolStamped>("debug_aloamgarm_aloam_ok", 1);
    debug_median_publisher = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_aloamgarm_median", 1);
  }

  m_eigenvalue_subscriber =
      m_nh.subscribe("slam_eigenvalues", 1, &AltitudeEstimatorAloamGarm2::callbackSlamEigenvalues, this, ros::TransportHints().tcpNoDelay());

  mrs_msgs::Float64ArrayStamped msg_eigenvalue;

  ros::Subscriber eigenvalue_subscriber;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[AltitudeEstimatorAloamGarm]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool AltitudeEstimatorAloamGarm2::doPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

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

  if (_use_initial_conditions_ && _initial_time_stamp_ > ros::Time::now()) {
    ROS_WARN_THROTTLE(0.5, "[Aloamgarm2] Using initial conditions, time is too low, skipping.");
    return true;
  }

  ros::WallTime time_beginning = ros::WallTime::now();

  algarm2_alt_u_t u = u.Zero();
  u(0)              = input;

  algarm2_alt_Q_t Q = m_Q;
  if (m_aloam_ok) {
    Q(3, 3) = m_Q(3, 3) * _q_factor_range_bias_slam_ok_;
  }

  {
    std::scoped_lock lock(mutex_lkf);

    if (!m_altitude_correction_received) {
      return true;
    }

    try {
      // Apply the prediction step
      mp_rep->addInputChangeWithNoise(u, Q, input_stamp, mp_lkf_vector[0]);
      m_sc = mp_rep->predictTo(predict_stamp);

      // debug publisher//{
      if (_debug_) {
        mrs_msgs::Float64ArrayStamped msg;
        msg.header.stamp = input_stamp;
        for (int i = 0; i < Q.size(); i++) {
          msg.values.push_back(Q(i));
        }
        debug_Q_publisher.publish(msg);

        mrs_msgs::Float64ArrayStamped msg_cov;
        msg_cov.header.stamp = predict_stamp;
        for (int i = 0; i < m_sc.P.size(); i++) {
          msg_cov.values.push_back(m_sc.P(i));
        }
        debug_cov_publisher.publish(msg_cov);
      }
      /*//}*/
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorAloamGarm]: LKF prediction step failed: %s", e.what());
    }
  }

  ros::WallTime time_end = ros::WallTime::now();
  double        diff     = (time_end.toSec() - time_beginning.toSec()) * 1000;
  if (diff > 10) {
    ROS_WARN_THROTTLE(1.0, "[AltitudeEstimatorAloamGarm] Correction took %.2f ms (longer than 10 ms).", diff);
  }
  // debug publisher//{
  if (_debug_) {
    mrs_msgs::AloamgarmDebug msg;
    msg.header.stamp         = ros::Time::now();
    msg.duration_all         = diff;
    msg.duration_input       = diff;
    float timestamp_diff     = ((predict_stamp.toSec() - input_stamp.toSec()) * 1000);
    msg.timestamp_diff_all   = timestamp_diff;
    msg.timestamp_diff_input = timestamp_diff;
    debug_duration_publisher.publish(msg);
  }
  /*//}*/
  return true;
}


//}

/*  //{ doPrediction() */

bool AltitudeEstimatorAloamGarm2::doPrediction(const double input, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

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

  if (_use_initial_conditions_ && _initial_time_stamp_ > ros::Time::now()) {
    ROS_WARN_THROTTLE(0.5, "[Aloamgarm2] Using initial conditions, time is too low, skipping.");
    return true;
  }

  algarm2_alt_u_t u = u.Zero();
  u(0)              = input;

  algarm2_alt_Q_t Q = m_Q;
  if (m_aloam_ok) {
    Q(3, 3) = m_Q(3, 3) * _q_factor_range_bias_slam_ok_;
  }

  {
    std::scoped_lock lock(mutex_lkf);
    try {
      // Apply the prediction step
      // add new input to repredictor (pass pointer to a lkf model so that it uses the correct model even after altitude coeff change)
      mp_rep->addInputChangeWithNoise(u, Q, input_stamp, mp_lkf_vector[0]);
      m_sc = mp_rep->predictTo(predict_stamp);

      // debug publisher//{
      if (_debug_) {
        mrs_msgs::Float64ArrayStamped msg;
        msg.header.stamp = input_stamp;
        for (int i = 0; i < Q.size(); i++) {
          msg.values.push_back(Q(i));
        }
        debug_Q_publisher.publish(msg);

        mrs_msgs::Float64ArrayStamped msg_cov;
        msg_cov.header.stamp = predict_stamp;
        for (int i = 0; i < m_sc.P.size(); i++) {
          msg_cov.values.push_back(m_sc.P(i));
        }
        debug_cov_publisher.publish(msg_cov);
      }
      /*//}*/
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

bool AltitudeEstimatorAloamGarm2::doCorrection(const double &measurement, int measurement_type, const ros::Time &meas_stamp, const ros::Time &predict_stamp,
                                               const std::string &measurement_name) {

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

  if (_use_initial_conditions_ && _initial_time_stamp_ > ros::Time::now()) {
    ROS_WARN_THROTTLE(0.5, "[Aloamgarm2] Using initial conditions, time is too low, skipping.");
    return true;
  }

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  ros::WallTime time_beginning = ros::WallTime::now();

  // Prepare the measurement vector
  algarm2_alt_z_t z;
  z << measurement;

  int lkf_id = measurement_type;
  if (measurement_name == "height_range") {
    lkf_id = m_garmin_biased_id;
  } else if (measurement_name == "height_aloam") {
    lkf_id = m_aloam_biased_id;
  } else if (measurement_name == "vel_baro") {
    lkf_id = m_baro_biased_id;
  }

  algarm2_alt_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
  std::scoped_lock lock(mutex_lkf);
  {

    algarm2_alt_Q_t Q = m_Q;

    if (measurement_name == "height_range" || measurement_name == "height_aloam") {
      m_altitude_correction_received = true;
    }

    if (measurement_name == "vel_baro" && !m_altitude_correction_received) {
      return true;
    }

    bool measurement_jumped = false;

    // RANGEFINDER//{
    if (measurement_name == "height_range") {


      if (!m_median_filter->isValid(z(0)) && m_median_filter->isFilled() &&
          (z(0) > _mf_close_to_ground_threshold_ || fabs(m_median_filter->getMedian() - z(0)) > _mf_changes_max_diff_close_to_ground_)) {
        measurement_jumped = true;
      }

      mrs_msgs::Float64ArrayStamped median_msg;
      median_msg.header.stamp = meas_stamp;
      median_msg.values.push_back(m_median_filter->getMedian());
      debug_median_publisher.publish(median_msg);

      /* if (z(0) < _mf_close_to_ground_threshold_) { */
      if (m_median_filter->getMedian() < _mf_close_to_ground_threshold_) {
        /* if (z(0) < _mf_close_to_ground_threshold_) { */
          m_close_to_ground = true;
          R(0)              = R(0) * 100;
        /* } */
        // else keep the original R
      }
      if (z(0) < 0.05) {
        return true;
      }
      if (z(0) > 4.0) {
        /* if (m_median_filter->isFilled() && m_median_filter->getMedian() > 4.0) { */
        R(0) = R(0) * 100;
      }
    }

    /* if (measurement_name == "height_range" && !m_median_filter->isValid(z(0)) && m_median_filter->isFilled() && */
    /*     (z(0) > _mf_close_to_ground_threshold_ || fabs(m_median_filter->getMedian() - z(0)) > _mf_changes_max_diff_close_to_ground_)) { */
    /*   // set H matrix so that only garmin bias is updated */
    /*   // calculate new bias measurement */
    /*   ROS_WARN_THROTTLE(0.5, "[AltitudeEstimatorAloamGarm] Garmin jump detected, altitude: %.2f, old bias: %.2f, measurement: %.2f.", m_sc.x(0), m_sc.x(3),
     */
    /*                     z(0)); */
    /*   R(0)    = R(0) * _r_factor_range_jump_; */
    /*   Q(3, 3) = m_Q(3, 3) * _q_factor_range_bias_range_jump_; */

    /*   mp_rep->addProcessNoiseChange(Q, meas_stamp, mp_lkf_vector[0]); */
    /* } */
    /*//}*/

    // LIDAR SLAM//{
    if (measurement_name == "height_aloam") {
      /* return true; // TODO remove */
      if (!m_eigenvalue_received) {
        ROS_WARN("[AltitudeEstimatorAloamGarm] SLAM altitude received, but no eigenvalue received yet.");
      } else {
        ROS_INFO_ONCE("[AltitudeEstimatorAloamGarm] SLAM altitude and eigenvalues successfully received.");
      }
      if (!m_aloam_ok) {
        // set matrices so that only bias is updated
        R(0)    = R(0) * _r_factor_slam_bad_;
        Q(4, 4) = m_Q(4, 4) * _q_factor_slam_bias_;

        mp_rep->addProcessNoiseChange(Q, meas_stamp, mp_lkf_vector[0]);
      }
    }
    /*//}*/

    // add the measurement to repredictor buffer and calculate new state vector
    try {
      int meas_id = -1;
      if (measurement_name == "height_range") {
        meas_id = lkf_id;
      }
      mp_rep->addMeasurement(z, R, meas_stamp, mp_lkf_vector[lkf_id], meas_id);
      m_sc = mp_rep->predictTo(predict_stamp);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[AltitudeEstimatorAloamGarm]: LKF correction step failed: %s", e.what());
    }

    // Debug publishers//{
    if (_debug_) {
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

      mrs_msgs::Float64ArrayStamped msg_q;
      msg_q.header.stamp = meas_stamp;
      for (int i = 0; i < Q.size(); i++) {
        msg_q.values.push_back(Q(i));
      }
      debug_Q_publisher.publish(msg_q);
    }
    /*//}*/
  }

  ros::WallTime time_end = ros::WallTime::now();
  double        diff     = (time_end.toSec() - time_beginning.toSec()) * 1000;
  if (diff > 10) {
    ROS_WARN_THROTTLE(1.0, "[AltitudeEstimatorAloamGarm] Correction took %.2f ms (longer than 10 ms).", diff);
  }

  // Debug - publish correction duration//{
  if (_debug_) {
    /* mrs_msgs::Float64ArrayStamped msg; */
    /* msg.header.stamp = ros::Time::now(); */
    /* msg.values.push_back(diff); // processing duration */
    /* // Differentiate duration when correction from ALOAM is used */
    /* msg.values.push_back((predict_stamp.toSec() - meas_stamp.toSec())*1000); // timestamp difference */
    /* if (measurement_name == "height_aloam") { */
    /*   msg.values.push_back(diff); // processing duration in case of aloam */
    /* } */
    /* debug_duration_publisher.publish(msg); */

    mrs_msgs::AloamgarmDebug msg;
    msg.header.stamp       = ros::Time::now();
    msg.duration_all       = diff;
    float timestamp_diff   = ((predict_stamp.toSec() - meas_stamp.toSec()) * 1000);
    msg.timestamp_diff_all = timestamp_diff;
    if (measurement_name == "height_aloam") {
      msg.duration_aloam       = diff;
      msg.timestamp_diff_aloam = timestamp_diff;
    } else if (measurement_name == "height_range") {
      msg.duration_garmin       = diff;
      msg.timestamp_diff_garmin = timestamp_diff;
    } else if (measurement_name == "vel_baro") {
      msg.duration_baro       = diff;
      msg.timestamp_diff_baro = timestamp_diff;
    } else {
      msg.duration_rest       = diff;
      msg.timestamp_diff_rest = timestamp_diff;
    }
    debug_duration_publisher.publish(msg);
  }
  /*//}*/
  return true;
}

//}

/*  //{ getStates() */

bool AltitudeEstimatorAloamGarm2::getStates(algarm2_alt_x_t &x) {

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

bool AltitudeEstimatorAloamGarm2::getStates(alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  x = m_sc.x.block(0, 0, 3, 1);

  return true;
}

//}

/*  //{ getState() */

bool AltitudeEstimatorAloamGarm2::getState(int state_id, double &state_val) {

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

std::string AltitudeEstimatorAloamGarm2::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool AltitudeEstimatorAloamGarm2::setState(int state_id, const double &state_val) {

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
    const algarm2_alt_u_t u0 = algarm2_alt_u_t::Zero();
    const ros::Time       t0 = ros::Time(0);
    mp_rep                   = std::make_unique<algarm2_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), _repredictor_buffer_size_);
  }

  return true;
}

//}

/*  //{ setR() */

bool AltitudeEstimatorAloamGarm2::setR(double R, int measurement_type) {

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

bool AltitudeEstimatorAloamGarm2::getR(double &R, int measurement_type) {

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

bool AltitudeEstimatorAloamGarm2::getQ(double &cov, const Eigen::Vector2i &idx) {

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

bool AltitudeEstimatorAloamGarm2::setQ(double cov, const Eigen::Vector2i &idx) {

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

bool AltitudeEstimatorAloamGarm2::setInputCoeff(double coeff) {

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
    // Don't forget to update these functions when changing system model!
    auto generateA = [=](const double dt) {
      algarm2_alt_A_t A;
      A << 1, dt, dt * dt / 2, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 1.0 - coeff, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
      return A;
    };
    m_A            = generateA(0.01);
    auto generateB = [=]([[maybe_unused]] const double dt) {
      algarm2_alt_B_t B;
      B << 0, 0, coeff, 0, 0, 0;
      return B;
    };
    m_B = generateB(0.01);
    // Clear lkf vector
    mp_lkf_vector.clear();
    // Reinitialize separate LKF models for each H matrix
    for (size_t i = 0; i < m_H_multi.size(); i++) {
      mp_lkf_vector.push_back(std::make_shared<algarm2_alt_t>(generateA, generateB, m_H_multi[i], _kernel_sigma_, m_nh, _nis_threshold_, _nis_avg_threshold_));
    }
  }

  std::cout << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".setInputCoeff(double coeff=" << coeff
            << " Changed input coefficient from: " << old_coeff << " to: " << coeff << std::endl;

  return true;
}

//}

/*  //{ getCovariance() */

bool AltitudeEstimatorAloamGarm2::getCovariance(alt_P_t &P) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    P = m_sc.P.block(0, 0, 3, 3);
  }

  return true;
}

//}

/*  //{ setCovariance() */

bool AltitudeEstimatorAloamGarm2::setCovariance(const alt_P_t &P) {

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

    algarm2_alt_P_t P_tmp    = algarm2_alt_P_t::Identity();
    m_sc.P                   = 1000.0 * P_tmp * P_tmp.transpose();
    m_sc.P.block(0, 0, 3, 3) = P;
    // reset repredictor
    const algarm2_alt_u_t u0 = algarm2_alt_u_t::Zero();
    const ros::Time       t0 = ros::Time(0);
    mp_rep                   = std::make_unique<algarm2_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), _repredictor_buffer_size_);
  }

  return true;
}
//}

/*  //{ reset() */

bool AltitudeEstimatorAloamGarm2::reset(const alt_x_t &x) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      if (!std::isfinite(x(i, j))) {
        std::cerr << "[AltitudeEstimatorAloamGarm]: " << m_estimator_name << ".reset(const algarm2_alt_x_t &x="  // << x
                  << "): NaN detected in variable \"x(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x                   = algarm2_alt_x_t::Zero();
    m_sc.x.block(0, 0, 3, 1) = (x);
    const algarm2_alt_u_t u0 = algarm2_alt_u_t::Zero();
    const ros::Time       t0 = ros::Time(0);
    mp_rep                   = std::make_unique<algarm2_rep_t>(m_sc.x, m_sc.P, u0, m_Q, t0, mp_lkf_vector.at(0), _repredictor_buffer_size_);
  }

  return true;
}

//}

/* callbackSlamEigenvalues() */ /*//{*/
void AltitudeEstimatorAloamGarm2::callbackSlamEigenvalues(const mrs_msgs::Float64ArrayStampedConstPtr &msg) {
  if (!m_is_initialized) {
    return;
  }
  // note: It could happen that the SLAM altitude will be received earlier than the eigenvalue but it shouldnt pose any problems

  // eigenvalue order in the array should be theta_x, theta_y, theta_z, x, y, z
  double aloam_eigenvalue = msg->values[5];

  std::scoped_lock lock(mutex_lkf);
  {
    // hysteresis to prevent rapid switching between the 2 states
    if (m_aloam_ok && aloam_eigenvalue < _eigenvalue_hysteresis_lower_) {
      m_aloam_ok = false;
    } else if (!m_aloam_ok && aloam_eigenvalue > _eigenvalue_hysteresis_upper_) {
      m_aloam_ok = true;
    }
    // save eigenvalue
    m_aloam_eig           = aloam_eigenvalue;
    m_eigenvalue_received = true;
    // Debug publisher//{
    if (_debug_) {
      mrs_msgs::BoolStamped msg_aloam_ok;
      msg_aloam_ok.stamp = msg->header.stamp;
      msg_aloam_ok.data  = m_aloam_ok;
      debug_aloam_ok_publisher.publish(msg_aloam_ok);
    }
    /*//}*/
  }
}
/*//}*/
}  // namespace mrs_uav_odometry
