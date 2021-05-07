#define VERSION "1.0.0.0"

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/EspOdometry.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/RtkFixType.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/ChangeEstimator.h>
#include <mrs_msgs/ChangeHdgEstimator.h>
#include <mrs_msgs/ChangeAltEstimator.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/LkfStates.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/OffsetOdom.h>
#include <mrs_msgs/Altitude.h>
#include <mrs_msgs/AltitudeStateNames.h>
#include <mrs_msgs/AltitudeType.h>
#include <mrs_msgs/Heading.h>
#include <mrs_msgs/HeadingStateNames.h>
#include <mrs_msgs/HeadingType.h>
#include <mrs_msgs/EstimatedState.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/AttitudeCommand.h>
#include <mrs_msgs/ReferenceStampedSrv.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/median_filter.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/transform_broadcaster.h>

#include <types.h>
#include <support.h>
#include <StateEstimator.h>
#include <AltitudeEstimator.h>
#include <AltitudeEstimatorAloamGarm.h>
#include <HeadingEstimator.h>
#include <mrs_uav_odometry/odometry_dynparamConfig.h>

#include <string>
#include <locale>
#include <Eigen/Eigen>
#include <math.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <queue>

//}

/* defines //{ */

#define btoa(x) ((x) ? "true" : "false")
#define NAME_OF(v) #v

//}

/* using //{ */

using namespace Eigen;

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

namespace mrs_uav_odometry
{

/* //{ class Odometry */

class Odometry : public nodelet::Nodelet {

public:
  std::string  _version_;
  virtual void onInit();

public:
  int  _main_rate_;
  bool is_initialized_      = false;
  bool is_ready_to_takeoff_ = false;

private:
  std::string _uav_name_;
  bool        _simulation_ = false;

  bool   _debug_publish_corrections_;
  bool   _debug_publish_servoing_;
  bool   _publish_fused_odom_;
  bool   _publish_pixhawk_velocity_;
  bool   _dynamic_optflow_cov_       = false;
  double _dynamic_optflow_cov_scale_ = 0;
  double twist_q_x_prev_             = 0;
  double twist_q_y_prev_             = 0;

  double _max_optflow_altitude_;
  double _max_default_altitude_;

  ros::NodeHandle nh_;

private:
  ros::Publisher pub_uav_state_;
  ros::Publisher pub_odom_main_;
  ros::Publisher pub_odom_main_inno_;
  ros::Publisher pub_odom_local_;
  ros::Publisher pub_odom_stable_;
  ros::Publisher pub_slow_odom_;
  ros::Publisher pub_esp_odom_;
  ros::Publisher pub_rtk_local_;
  ros::Publisher pub_rtk_local_odom_;
  ros::Publisher pub_gps_local_odom_;
  ros::Publisher pub_orientation_gt_;
  ros::Publisher pub_orientation_mavros_;
  ros::Publisher pub_des_attitude_global_;
  ros::Publisher pub_odometry_diag_;
  ros::Publisher pub_altitude_;
  ros::Publisher pub_height_;
  ros::Publisher pub_orientation_;
  ros::Publisher pub_max_altitude_;
  ros::Publisher pub_lkf_states_x_;
  ros::Publisher pub_lkf_states_y_;
  ros::Publisher pub_heading_states_;
  ros::Publisher pub_altitude_state_;
  ros::Publisher pub_alt_cov_;
  ros::Publisher pub_hector_reset_;
  ros::Publisher pub_imu_untilted_;
  ros::Publisher pub_brick_diag_;

  ros::Publisher pub_compass_hdg_corr_;
  ros::Publisher pub_hector_hdg_corr_;
  ros::Publisher pub_aloam_hdg_corr_;
  ros::Publisher pub_liosam_hdg_corr_;
  ros::Publisher pub_brick_hdg_corr_;
  ros::Publisher pub_vio_hdg_corr_;
  ros::Publisher pub_vslam_hdg_corr_;
  ros::Publisher pub_vel_baro_corr_;
  ros::Publisher pub_vel_liosam_twist_z_corr_;


  ros::Publisher pub_cmd_hdg_input_;
  ros::Publisher pub_cmd_hdg_rate_input_;

  ros::Publisher pub_debug_optflow_filter_;
  ros::Publisher pub_debug_icp_twist_filter_;

  ros::Publisher pub_debug_aloam_delay_;

private:
  ros::Subscriber sub_global_position_;
  ros::Subscriber sub_control_manager_diag_;

  ros::Subscriber sub_pixhawk_;
  ros::Subscriber sub_pixhawk_imu_;
  ros::Subscriber sub_pixhawk_compass_;
  ros::Subscriber sub_optflow_;
  ros::Subscriber sub_optflow_low_;
  ros::Subscriber sub_vio_;
  ros::Subscriber sub_vslam_;
  ros::Subscriber sub_control_accel_;
  ros::Subscriber sub_t265_odom_;
  ros::Subscriber sub_brick_;
  ros::Subscriber rtk_gps_sub_;
  ros::Subscriber sub_icp_twist_;
  ros::Subscriber sub_hector_pose_;
  ros::Subscriber sub_aloam_odom_;
  ros::Subscriber sub_liosam_odom_;
  ros::Subscriber sub_brick_pose_;
  ros::Subscriber sub_attitude_command_;
  ros::Subscriber sub_ground_truth_;
  ros::Subscriber sub_vio_state_;
  ros::Subscriber sub_uav_mass_estimate_;
  ros::Subscriber sub_gps_covariance_;

private:
  ros::ServiceServer ser_reset_lateral_kalman_;
  ros::ServiceServer ser_reset_hector_;
  ros::ServiceServer ser_reliable_hector_;
  ros::ServiceServer ser_offset_odom_;
  ros::ServiceServer ser_garmin_;
  ros::ServiceServer ser_toggle_rtk_altitude_;
  ros::ServiceServer ser_change_odometry_source_;
  ros::ServiceServer ser_change_estimator_type_;
  ros::ServiceServer ser_change_estimator_type_string_;
  ros::ServiceServer ser_change_hdg_estimator_type_;
  ros::ServiceServer ser_change_hdg_estimator_type_string_;
  ros::ServiceServer ser_change_alt_estimator_type_;
  ros::ServiceServer ser_change_alt_estimator_type_string_;
  ros::ServiceServer ser_toggle_callbacks_;

  ros::ServiceClient ser_client_failsafe_;
  ros::ServiceClient ser_client_hover_;
  ros::ServiceClient ser_client_reference_;
  ros::ServiceClient ser_client_ehover_;
  ros::ServiceClient ser_client_tracker_;
  ros::ServiceClient ser_client_controller_;
  ros::ServiceClient ser_client_enable_callbacks_;

private:
  mrs_lib::TransformBroadcaster *             broadcaster_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  mrs_lib::Transformer                        transformer_;

  double         init_magnetic_heading_ = 0.0;
  double         init_brick_hdg_        = 0.0;
  double         hdg_diff_              = 0.0;
  ros::Publisher pub_odom_mavros_;

  nav_msgs::Odometry odom_main_inno_;
  std::mutex         mutex_odom_main_inno_;
  std::mutex         mutex_uav_state_;


  // PIXHAWK
  nav_msgs::Odometry odom_pixhawk_;
  nav_msgs::Odometry odom_pixhawk_previous_;
  std::mutex         mutex_odom_pixhawk_;
  nav_msgs::Odometry odom_pixhawk_shifted_;
  nav_msgs::Odometry odom_pixhawk_previous_shifted_;
  std::mutex         mutex_odom_pixhawk_shifted_;
  ros::Time          odom_pixhawk_last_update_;
  std::mutex         mutex_gps_local_odom_;
  std::mutex         mutex_gps_covariance_;
  double             gps_covariance_                       = 0.0;
  double             _gps_fallback_covariance_limit_       = 0.0;
  double             _gps_fallback_covariance_ok_          = 0.0;
  std::string        _gps_fallback_estimator_              = "OPTFLOW";
  bool               _gps_fallback_allowed_                = false;
  bool               _gps_return_after_fallback_           = false;
  bool               gps_in_fallback_                      = false;
  int                c_gps_cov_over_lim_                   = 0;
  int                c_gps_cov_ok_                         = 0;
  int                _gps_fallback_bad_samples_            = 0;
  int                _gps_fallback_good_samples_           = 0;
  double             _gps_fallback_altitude_               = 4.0;
  double             _gps_fallback_wait_for_altitude_time_ = 5.0;

  double des_hdg_rate_ = 0.0;
  double des_hdg_      = 0.0;

  std::vector<nav_msgs::Odometry> vec_odom_aux_;

  // Mavros odometry and local odometry cache for aloam mapping tf
  std::vector<nav_msgs::Odometry> vec_odom_mavros_;
  std::mutex                      mutex_vec_odom_mavros_;
  std::vector<nav_msgs::Odometry> vec_odom_local_;
  std::mutex                      mutex_vec_odom_local_;

  // OPTFLOW
  geometry_msgs::TwistWithCovarianceStamped optflow_twist_;
  std::mutex                                mutex_optflow_;
  geometry_msgs::TwistWithCovarianceStamped optflow_twist_previous_;
  ros::Time                                 optflow_twist_last_update_;
  bool                                      _use_lat_mf_optflow_twist_ = false;
  std::unique_ptr<MedianFilter>             lat_mf_optflow_twist_x_;
  std::unique_ptr<MedianFilter>             lat_mf_optflow_twist_y_;
  double                                    _optflow_max_valid_twist_;
  bool                                      fusing_optflow_low_ = true;
  bool                                      _use_optflow_low_   = false;

  // VIO
  nav_msgs::Odometry odom_vio_;
  std::mutex         mutex_odom_vio_;
  nav_msgs::Odometry odom_vio_previous_;
  std::mutex         mutex_odom_vio_previous_;
  ros::Time          odom_vio_last_update_;

  // VSLAM
  geometry_msgs::PoseWithCovarianceStamped pose_vslam_;
  std::mutex                               mutex_pose_vslam_;
  geometry_msgs::PoseWithCovarianceStamped pose_vslam_previous_;
  std::mutex                               mutex_pose_vslam_previous_;
  ros::Time                                pose_vslam_last_update_;

  // Realsense t265
  nav_msgs::Odometry odom_t265_;
  std::mutex         mutex_odom_t265_;
  nav_msgs::Odometry odom_t265_previous_;
  ros::Time          odom_t265_last_update_;

  // brick msgs
  nav_msgs::Odometry odom_brick_;
  std::mutex         mutex_odom_brick_;
  nav_msgs::Odometry odom_brick_previous_;
  ros::Time          odom_brick_last_update_;
  int                counter_odom_brick_;
  int                c_failed_brick_x_       = 0;
  int                c_failed_brick_y_       = 0;
  int                c_failed_brick_timeout_ = 0;
  int                c_failed_brick_hdg_     = 0;

  // IMU msgs
  sensor_msgs::Imu pixhawk_imu_;
  sensor_msgs::Imu pixhawk_imu_previous_;
  std::mutex       mutex_pixhawk_imu_;
  ros::Time        pixhawk_imu_last_update_;

  // Control acceleration msgs
  sensor_msgs::Imu       control_accel_;
  sensor_msgs::Imu       control_accel_previous_;
  std::mutex             mutex_control_accel_;
  ros::Time              control_accel_last_update_;
  geometry_msgs::Vector3 acc_global_prev_;

  // Compass msgs
  std_msgs::Float64 compass_hdg_;
  std_msgs::Float64 compass_hdg_previous_;
  double            hdg_previous_;
  std::mutex        mutex_compass_hdg_;
  ros::Time         compass_hdg_last_update_;

  // Hector heading msgs
  double     hector_hdg_previous_;
  std::mutex mutex_hector_hdg_;
  ros::Time  hector_hdg_last_update_;
  double     hdg_hector_corr_;

  // ALOAM heading msgs
  double aloam_hdg_previous_;
  int    _aloam_hdg_filter_buffer_size_;
  double _aloam_hdg_filter_max_diff_;

  // Aloam heading messages
  std::mutex         mutex_aloam_;
  double             pos_aloam_x_, pos_aloam_y_;
  nav_msgs::Odometry aloam_odom_;
  nav_msgs::Odometry aloam_odom_previous_;
  ros::Time          aloam_odom_last_update_;
  Vec2               aloam_offset_;
  Vec2               aloam_vel_state_;
  double             aloam_offset_hdg_;
  bool               aloam_corr_ready_ = false;

  ros::Time aloam_timestamp_;
  bool      aloam_updated_mapping_tf_ = false;

  // LIOSAM heading msgs
  double liosam_hdg_previous_;
  int    _liosam_hdg_filter_buffer_size_;
  double _liosam_hdg_filter_max_diff_;

  // LIOSAM heading messages
  std::mutex         mutex_liosam_;
  double             pos_liosam_x_, pos_liosam_y_;
  nav_msgs::Odometry liosam_odom_;
  nav_msgs::Odometry liosam_odom_previous_;
  ros::Time          liosam_odom_last_update_;
  Vec2               liosam_offset_;
  Vec2               liosam_vel_state_;
  double             liosam_offset_hdg_;
  bool               liosam_corr_ready_ = false;

  ros::Time liosam_timestamp_;
  bool      liosam_updated_mapping_tf_ = false;

  // brick heading msgs
  double     brick_hdg_previous_;
  std::mutex mutex_brick_hdg_;
  ros::Time  brick_hdg_last_update_;
  double     accum_hdg_brick_;
  double     _accum_hdg_brick_alpha_;

  // VIO heading msgs
  double     vio_hdg_previous_;
  std::mutex mutex_vio_hdg_;
  std::mutex mutex_vio_hdg_previous_;

  // VSLAM heading msgs
  double     vslam_hdg_previous_;
  std::mutex mutex_vslam_hdg_;

  geometry_msgs::Vector3Stamped orientation_mavros_;
  geometry_msgs::Vector3Stamped orientation_gt_;

  double     _uav_mass_estimate_;
  std::mutex mutex_uav_mass_estimate_;

  // Target attitude msgs
  mrs_msgs::AttitudeCommand attitude_command_;
  mrs_msgs::AttitudeCommand attitude_command_prev_;
  ros::Time                 attitude_command_last_update_;
  std::mutex                mutex_attitude_command_;

  // RTK
  std::mutex       mutex_rtk_;
  mrs_msgs::RtkGps rtk_odom_previous_;
  mrs_msgs::RtkGps rtk_odom_;
  ros::Time        rtk_last_update_;
  bool             _rtk_fuse_sps_;
  bool             _use_rtk_altitude_;

  // Hector messages
  std::mutex                 mutex_hector_;
  double                     pos_hector_x_, pos_hector_y_;
  geometry_msgs::PoseStamped hector_pose_;
  geometry_msgs::PoseStamped hector_pose_previous_;
  ros::Time                  hector_pose_last_update_;
  bool                       hector_reset_called_         = false;
  bool                       _reset_hector_after_takeoff_ = false;
  Vec2                       hector_offset_;
  Vec2                       hector_vel_state_;
  double                     hector_offset_hdg_;
  int                        c_hector_msg_;
  int                        c_hector_init_msgs_;
  bool                       hector_corr_ready_ = false;

  // ICP messages
  std::mutex                                mutex_icp_twist_;
  geometry_msgs::TwistWithCovarianceStamped icp_twist_;
  geometry_msgs::TwistWithCovarianceStamped icp_twist_previous_;
  ros::Time                                 icp_twist_last_update_;
  bool                                      _use_lat_mf_icp_twist_ = false;
  std::unique_ptr<MedianFilter>             lat_mf_icp_twist_x_;
  std::unique_ptr<MedianFilter>             lat_mf_icp_twist_y_;
  double                                    _icp_max_valid_twist_;

  std::unique_ptr<MedianFilter> hdg_mf_icp_rate_;
  bool                          _use_hdg_mf_icp_rate_;
  double                        _icp_max_valid_hdg_rate_;
  double                        icp_hdg_rate_inconsistent_samples_;

  // brick messages
  std::mutex                 mutex_brick_;
  geometry_msgs::PoseStamped brick_pose_;
  geometry_msgs::PoseStamped brick_pose_previous_;
  ros::Time                  brick_pose_last_update_;
  double                     _brick_timeout_;

  std::mutex         mutex_ground_truth_;
  nav_msgs::Odometry ground_truth_;

  mrs_msgs::RtkGps rtk_local_previous_;
  mrs_msgs::RtkGps rtk_local_;

  mrs_msgs::EstimatorType  estimator_type_;
  mrs_msgs::EstimatorType  fallback_brick_estimator_type_;
  mrs_msgs::HeadingType    fallback_brick_hdg_estimator_type_;
  mrs_msgs::EstimatorType  _estimator_type_takeoff_;
  std::vector<std::string> _estimator_type_names_;
  std::vector<std::string> _altitude_type_names_;
  std::string              _altitude_estimator_name_;
  std::mutex               mutex_estimator_type_;
  std::mutex               mutex_alt_estimator_type_;
  int                      estimator_iteration_;
  std::mutex               mutex_alt_input_;
  double                   alt_input_ = 0;
  ros::Time                alt_input_stamp_;

  std::mutex  mutex_odom_local_;
  std::string fcu_frame_id_;
  std::string fcu_untilted_frame_id_;
  std::string local_origin_frame_id_;
  std::string stable_origin_frame_id_;
  std::string aloam_mapping_origin_frame_id_;
  std::string liosam_mapping_origin_frame_id_;
  std::string last_stable_name_;
  std::string last_local_name_;
  std::string first_frame_;

  bool   got_fcu_untilted_ = false;
  bool   got_init_heading_ = false;
  double init_heading_;

  bool is_updating_state_     = false;
  bool finished_state_update_ = false;

  double     _hiccup_thr_ = 0.03;
  int        c_hiccup_    = 0;
  std::mutex mutex_c_hiccup_;

  // | -------------------- message callbacks ------------------- |
  void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackVslamPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void callbackT265Odometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackOptflowTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);
  void callbackOptflowTwistLow(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);
  void callbackPixhawkUtm(const sensor_msgs::NavSatFixConstPtr &msg);
  void callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg);
  void callbackHectorPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackAloamOdom(const nav_msgs::OdometryConstPtr &msg);
  void callbackLioSamOdom(const nav_msgs::OdometryConstPtr &msg);
  void callbackICPTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);
  void callbackBrickPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg);
  void callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg);
  void callbackReconfigure(mrs_uav_odometry::odometry_dynparamConfig &config, uint32_t level);
  void callbackPixhawkImu(const sensor_msgs::ImuConstPtr &msg);
  void callbackPixhawkCompassHdg(const std_msgs::Float64ConstPtr &msg);
  void callbackUavMassEstimate(const std_msgs::Float64ConstPtr &msg);
  void callbackGPSCovariance(const nav_msgs::OdometryConstPtr &msg);

  // | ------------------- service callbacks ------------------- |
  bool callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackChangeOdometrySource(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackChangeEstimator(mrs_msgs::ChangeEstimator::Request &req, mrs_msgs::ChangeEstimator::Response &res);
  bool callbackChangeEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackResetEstimator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackResetHector([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackReliableHector([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackChangeHdgEstimator(mrs_msgs::ChangeHdgEstimator::Request &req, mrs_msgs::ChangeHdgEstimator::Response &res);
  bool callbackChangeHdgEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackChangeAltEstimator(mrs_msgs::ChangeAltEstimator::Request &req, mrs_msgs::ChangeAltEstimator::Response &res);
  bool callbackChangeAltEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackToggleCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // | --------------------- helper methods --------------------- |
  bool isReadyToTakeoff();
  void stateEstimatorsPrediction(const geometry_msgs::Vector3 &acc_in, double dt, const ros::Time &input_stamp = ros::Time::now(),
                                 const ros::Time &predict_stamp = ros::Time::now());
  void stateEstimatorsCorrection(double x, double y, const std::string &measurement_name, const ros::Time &meas_stamp = ros::Time::now(),
                                 const ros::Time &predict_stamp = ros::Time::now());
  bool changeCurrentEstimator(const mrs_msgs::EstimatorType &desired_estimator);

  void altitudeEstimatorsPrediction(const double input, const double dt, const ros::Time &input_stamp = ros::Time::now(),
                                    const ros::Time &predict_stamp = ros::Time::now());
  void altitudeEstimatorCorrection(double value, const std::string &measurement_name, const ros::Time &meas_stamp = ros::Time::now(),
                                   const ros::Time &predict_stamp = ros::Time::now());
  void altitudeEstimatorCorrection(double value, const std::string &measurement_name, const std::shared_ptr<mrs_uav_odometry::AltitudeEstimator> &estimator,
                                   const ros::Time &meas_stamp = ros::Time::now(), const ros::Time &predict_stamp = ros::Time::now());

  bool changeCurrentAltitudeEstimator(const mrs_msgs::AltitudeType &desired_estimator);

  void headingEstimatorsPrediction(const double hdg, const double hdg_rate, const double dt, const ros::Time &input_stamp = ros::Time::now(),
                                   const ros::Time &predict_stamp = ros::Time::now());
  void headingEstimatorsCorrection(const double value, const std::string &measurement_name, const ros::Time &input_stamp = ros::Time::now(),
                                   const ros::Time &predict_stamp = ros::Time::now());
  bool changeCurrentHeadingEstimator(const mrs_msgs::HeadingType &desired_estimator);

  void               getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz);
  double             getGlobalZAcceleration(const geometry_msgs::Quaternion &q_msg, const double &acc_z_in);
  void               getRotatedVector(const geometry_msgs::Vector3 &acc_in, double hdg_in, geometry_msgs::Vector3 &acc_out);
  void               rotateLateralStates(const double hdg_new, const double hdg_old);
  double             getCurrentHeading();
  bool               isValidType(const mrs_msgs::EstimatorType &type);
  bool               isValidType(const mrs_msgs::HeadingType &type);
  bool               isValidType(const mrs_msgs::AltitudeType &type);
  bool               isTimestampOK(const double curr_sec, const double prev_sec);
  std::string        printOdometryDiag();
  bool               stringInVector(const std::string &value, const std::vector<std::string> &vector);
  nav_msgs::Odometry applyOdomOffset(const nav_msgs::Odometry &msg, const tf2::Vector3 &pos_offset, const tf2::Quaternion &rot_offset);
  bool               isValidGate(const double &value, const double &min_value, const double &max_value, const std::string &value_name = "");
  double             saturateCorrection(const double &correction, const double max_correction, const std::string &correction_name);

  // | ------------------ call service routines ----------------- |
  bool callEnableControlCallbacks();
  bool callDisableControlCallbacks();
  bool callHover();
  bool callMpcController();

  nav_msgs::Odometry shared_odom_;
  std::mutex         mutex_shared_odometry_;

  nav_msgs::Odometry rtk_local_odom_;
  std::mutex         mutex_rtk_local_odom_;

  nav_msgs::Odometry odom_local_;
  tf2::Vector3       pos_odom_offset_;
  tf2::Quaternion    rot_odom_offset_;

  nav_msgs::Odometry odom_stable_;
  tf2::Vector3       odom_stable_pos_offset_;
  tf2::Quaternion    odom_stable_rot_offset_;

  // Garmin altitude subscriber and callback
  ros::Subscriber               sub_garmin_;
  sensor_msgs::Range            range_garmin_;
  sensor_msgs::Range            range_garmin_previous_;
  std::mutex                    mutex_range_garmin_;
  void                          callbackGarmin(const sensor_msgs::RangeConstPtr &msg);
  std::unique_ptr<MedianFilter> alt_mf_garmin_;
  ros::Time                     garmin_last_update_;
  bool                          saturate_garmin_corrections_ = false;
  double                        _garmin_inno_gate_value_sq_;
  bool                          _use_garmin_inno_gate_;
  double                        _garmin_min_valid_alt_;
  double                        _garmin_max_valid_alt_;


  bool   callbacks_enabled_ = false;
  bool   baro_corrected_    = false;
  double baro_offset_       = 0.0;

  // sonar altitude subscriber and callback
  ros::Subscriber               sub_sonar_;
  sensor_msgs::Range            range_sonar_;
  sensor_msgs::Range            range_sonar_previous_;
  std::mutex                    mutex_range_sonar_;
  void                          callbackSonar(const sensor_msgs::RangeConstPtr &msg);
  std::unique_ptr<MedianFilter> alt_mf_sonar_;
  double                        _sonar_min_valid_alt_, _sonar_max_valid_alt_;
  ros::Time                     sonar_last_update_;

  // Plane altitude subscriber and callback
  ros::Subscriber               sub_plane_;
  sensor_msgs::Range            range_plane_;
  sensor_msgs::Range            range_plane_previous_;
  std::mutex                    mutex_range_plane_;
  void                          callbackPlane(const sensor_msgs::RangeConstPtr &msg);
  std::unique_ptr<MedianFilter> alt_mf_plane_;
  double                        _plane_min_valid_alt_, _plane_max_valid_alt_;
  ros::Time                     plane_last_update_;

  // Brick altitude median filter
  std::unique_ptr<MedianFilter> alt_mf_brick_;
  double                        _brick_min_valid_alt_, _brick_max_valid_alt_;

  // VIO altitude median filter
  std::unique_ptr<MedianFilter> alt_mf_vio_;
  double                        _vio_min_valid_alt_, _vio_max_valid_alt_;

  // ALOAM height median filter
  std::unique_ptr<MedianFilter> alt_mf_aloam_;
  double                        _aloam_min_valid_alt_, _aloam_max_valid_alt_;

  // LIOSAM height median filter
  std::unique_ptr<MedianFilter> alt_mf_liosam_;
  double                        _liosam_min_valid_alt_, _liosam_max_valid_alt_;

  // Flag set when received first message
  bool got_odom_pixhawk_     = false;
  bool got_odom_t265_        = false;
  bool got_optflow_          = false;
  bool got_range_            = false;
  bool got_plane_            = false;
  bool got_pixhawk_utm_      = false;
  bool got_rtk_              = false;
  bool got_hector_pose_      = false;
  bool got_aloam_odom_       = false;
  bool got_liosam_odom_      = false;
  bool got_brick_pose_       = false;
  bool got_attitude_command_ = false;
  bool got_vio_              = false;
  bool got_vslam_            = false;
  bool got_altitude_sensors_ = false;
  bool got_lateral_sensors_  = false;
  bool got_rtk_fix_          = false;
  bool got_pixhawk_imu_      = false;
  bool got_compass_hdg_      = false;
  bool got_control_accel_    = false;
  bool got_icp_twist_        = false;

  bool failsafe_called_ = false;

  int  got_rtk_counter_;
  bool got_rtk_local_origin_z_;

  bool rtk_odom_initialized_ = false;

  geometry_msgs::Vector3Stamped des_attitude_global_;

  // for setting home position
  double _utm_origin_x_, _utm_origin_y_;
  int    _utm_origin_units_ = 0;
  double rtk_local_origin_z_ = 0.0;
  bool   _init_gps_origin_local_;
  double _init_gps_offset_x_, _init_gps_offset_y_;
  double land_position_x_, land_position_y_;
  bool   land_position_set_ = false;

  // current position in UTM as measure by pixhawk
  double     pixhawk_utm_position_x_, pixhawk_utm_position_y_;
  std::mutex mutex_pixhawk_utm_position_;

  // subscribing to tracker status
  mrs_msgs::ControlManagerDiagnostics control_manager_diag_;
  std::mutex                          mutex_control_manager_diag_;
  void                                callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);
  bool                                got_control_manager_diag_ = false;
  bool                                isUavFlying();
  bool                                isUavLandoff();
  bool                                uav_in_the_air_ = false;

  const std::string null_tracker_    = NULL_TRACKER;
  const std::string landoff_tracker_ = LANDOFF_TRACKER;

  // offset to adjust the local origin
  double pixhawk_odom_offset_x_, pixhawk_odom_offset_y_;
  bool   got_pixhawk_odom_offset_  = false;
  bool   got_pixhawk_odom_shifted_ = false;

  geometry_msgs::Vector3 mavros_glitch_;  // TODO is this still needed in new mavros versions?

  // initial position
  double init_pose_x_   = 0.0;
  double init_pose_y_   = 0.0;
  double init_pose_hdg_ = 0.0;

  // heading estimation
  hdg_Q_t                                                  _Q_hdg_;
  std::mutex                                               mutex_heading_estimator_;
  std::mutex                                               mutex_hdg_estimator_type_;
  std::vector<std::string>                                 _heading_estimators_names_;
  std::vector<std::string>                                 _active_heading_estimators_names_;
  std::vector<std::string>                                 _hdg_model_state_names_;
  std::vector<std::string>                                 _hdg_measurement_names_;
  std::map<std::string, std::vector<std::string>>          map_hdg_estimator_measurement_;
  std::map<std::string, hdg_R_t>                           map_hdg_measurement_covariance_;
  std::map<std::string, std::string>                       map_hdg_measurement_state_;
  std::map<std::string, int>                               map_hdg_measurement_name_id_;
  std::map<std::string, hdg_H_t>                           map_hdg_states_;
  std::map<std::string, mrs_msgs::Float64ArrayStamped>     map_hdg_estimator_msg_;
  std::map<std::string, ros::Publisher>                    map_hdg_estimator_pub_;
  std::map<std::string, std::shared_ptr<HeadingEstimator>> heading_estimators_;
  std::shared_ptr<HeadingEstimator>                        current_hdg_estimator_;
  std::string                                              current_hdg_estimator_name_;
  mrs_msgs::HeadingType                                    hdg_estimator_type_;
  mrs_msgs::HeadingType                                    _hdg_estimator_type_takeoff_;
  std::vector<std::string>                                 _heading_type_names_;  // TODO unify with _heading_estimators_names_
  std::string                                              _heading_estimator_name_;
  std::mutex                                               mutex_current_hdg_estimator_;
  bool                                                     init_hdg_avg_done_;
  int                                                      init_hdg_avg_samples_;
  double                                                   init_hdg_avg_;

  int    _compass_hdg_filter_buffer_size_;
  double _compass_hdg_filter_max_diff_;

  std::unique_ptr<MedianFilter> hdg_mf_optflow_rate_;
  bool                          _use_hdg_mf_optflow_rate_;
  double                        _optflow_max_valid_hdg_rate_;
  int                           compass_inconsistent_samples_;
  int                           optflow_inconsistent_samples_;
  bool                          is_heading_estimator_initialized_ = false;
  bool                          _gyro_fallback_;

  // altitude estimation
  alt_Q_t                                                   _Q_alt_;
  std::mutex                                                mutex_altitude_estimator_;
  std::vector<std::string>                                  _altitude_estimators_names_;
  std::vector<std::string>                                  _alt_model_state_names_;
  std::vector<std::string>                                  _alt_measurement_names_;
  std::map<std::string, std::vector<std::string>>           map_alt_estimator_measurement_;
  std::map<std::string, int>                                map_alt_n_states_;
  std::map<std::string, alt_R_t>                            map_alt_measurement_covariance_;
  std::map<std::string, std::string>                        map_alt_measurement_state_;
  std::map<std::string, int>                                map_alt_measurement_name_id_;
  std::map<std::string, alt_H_t>                            map_alt_states_;
  std::map<std::string, mrs_msgs::Float64Stamped>           map_alt_estimator_msg_;
  std::map<std::string, ros::Publisher>                     map_alt_estimator_pub_;
  std::map<std::string, std::shared_ptr<AltitudeEstimator>> _altitude_estimators_;
  std::shared_ptr<AltitudeEstimator>                        current_alt_estimator_;
  std::string                                               current_alt_estimator_name_;
  mrs_msgs::AltitudeType                                    alt_estimator_type_;
  mrs_msgs::AltitudeType                                    _alt_estimator_type_takeoff;
  std::mutex                                                mutex_current_alt_estimator_;
  bool                                                      is_lateral_estimator_initialized_ = false;
  bool                                                      lat_gps_est_pos_init_             = false;
  int                                                       counter_altitude_                 = 0;
  double                                                    _excessive_tilt_sq_;

  // State estimation
  std::vector<std::string>                               _state_estimators_names_;
  std::vector<std::string>                               _active_state_estimators_names_;
  std::vector<std::string>                               _model_state_names_;
  std::vector<std::string>                               _measurement_names_;
  std::map<std::string, std::vector<std::string>>        map_estimator_measurement_;
  std::map<std::string, bool>                            map_estimator_publish_tf_;
  std::map<std::string, Mat1>                            map_measurement_covariance_;
  std::map<std::string, std::string>                     map_measurement_state_;
  std::map<std::string, int>                             map_measurement_name_id_;
  std::map<std::string, LatStateCol1D>                   map_states_;
  std::map<std::string, nav_msgs::Odometry>              map_estimator_odom_;
  std::map<std::string, ros::Publisher>                  map_estimator_pub_;
  std::map<std::string, std::shared_ptr<StateEstimator>> _lateral_estimators_;
  std::shared_ptr<StateEstimator>                        current_lat_estimator_;
  std::mutex                                             mutex_current_lat_estimator_;
  std::string                                            current_lat_estimator_name_;

  bool   _saturate_mavros_position_;
  double _max_mavros_pos_correction_;
  double _max_vio_pos_correction_;
  double _max_vslam_pos_correction_;
  double _max_brick_pos_correction_;
  double _max_brick_hdg_correction_;
  double _max_rtk_pos_correction_;
  double _max_t265_vel_;
  double max_safe_brick_jump_sq_;
  double max_safe_brick_hdg_jump_sq_;

  LatMat        _A_lat_, _Q_lat_;
  LatStateCol1D _B_lat_;

  // RTK LKF
  using lkf_rtk_t = mrs_lib::LKF<2, 2, 2>;
  lkf_rtk_t::statecov_t sc_lat_rtk_;
  lkf_rtk_t::A_t        _A_lat_rtk_;
  lkf_rtk_t::B_t        _B_lat_rtk_;
  lkf_rtk_t::H_t        _H_lat_rtk_;
  lkf_rtk_t::R_t        _R_lat_rtk_;
  lkf_rtk_t::Q_t        _Q_lat_rtk_;
  lkf_rtk_t::P_t        _P_lat_rtk_;

  std::unique_ptr<lkf_rtk_t> estimator_rtk_;
  std::mutex                 mutex_rtk_est_;

  bool got_home_position_fix_ = false;
  bool calculatePixhawkOdomOffset(void);

  bool odometry_published_;

  double     max_altitude_ = 10;
  std::mutex mutex_max_altitude_;
  bool       gps_reliable_     = false;
  bool       hector_reliable_  = false;
  bool       aloam_reliable_   = false;
  bool       liosam_reliable_  = false;
  bool       vio_reliable_     = false;
  bool       vslam_reliable_   = false;
  bool       optflow_reliable_ = false;
  bool       rtk_reliable_     = false;
  bool       t265_reliable_    = false;
  bool       brick_reliable_   = false;
  bool       icp_reliable_     = false;
  bool       plane_reliable_   = false;

  // Active lateral estimators
  bool gps_active_        = false;
  bool rtk_active_        = false;
  bool optflow_active_    = false;
  bool optflowgps_active_ = false;
  bool hector_active_     = false;
  bool vio_active_        = false;
  bool vslam_active_      = false;
  bool t265_active_       = false;
  bool aloam_active_      = false;
  bool liosam_active_     = false;
  bool aloamgarm_active_  = false;
  bool aloamrep_active_   = false;
  bool brick_active_      = false;
  bool icp_active_        = false;

  // Active altitude estimators
  bool height_active_ = false;
  // TODO so far all are active

  // Active heading estimators
  bool compass_active_ = false;

  bool      brick_semi_reliable_ = false;
  ros::Time brick_semi_reliable_started_;

  bool   _pass_rtk_as_odom_ = false;
  double _max_altitude_correction_;

  // disabling rangefinders on the flight
  bool garmin_enabled_;
  bool sonar_enabled_;

  ros::Timer slow_odom_timer_;
  ros::Timer diag_timer_;
  ros::Timer lat_states_timer_;
  ros::Timer max_altitude_timer_;
  ros::Timer topic_watcher_timer_;
  ros::Timer hector_reset_routine_timer_;
  bool       hector_reset_routine_running_;
  bool       _perform_hector_reset_routine_;
  int        _slow_odom_rate_;
  int        _diag_rate_;
  int        _lkf_states_rate_;
  int        _max_altitude_rate_;
  int        topic_watcher_rate_ = 10;
  void       slowOdomTimer(const ros::TimerEvent &event);
  void       diagTimer(const ros::TimerEvent &event);
  void       lkfStatesTimer(const ros::TimerEvent &event);
  void       rtkRateTimer(const ros::TimerEvent &event);
  void       maxAltitudeTimer(const ros::TimerEvent &event);
  void       topicWatcherTimer(const ros::TimerEvent &event);
  void       callbackTimerHectorResetRoutine(const ros::TimerEvent &event);

  using lkf_height_t = mrs_lib::LKF<1, 1, 1>;
  std::unique_ptr<lkf_height_t> estimator_height_;
  lkf_height_t::R_t             _R_height_;
  lkf_height_t::Q_t             _Q_height_;
  lkf_height_t::statecov_t      sc_height_;
  std::mutex                    mutex_estimator_height_;
  ros::Time                     time_main_timer_prev_;
  bool                          first_main_timer_tick_ = true;

  // for fusing rtk altitude
  double _garmin_z_offset_;
  double _sonar_z_offset_;
  double _fcu_height_;

private:
  // for fusing rtk altitude
  bool       rtk_altitude_enabled_;
  double     rtk_altitude_integral_;
  ros::Timer rtk_rate_timer_;
  double     rtkQ_;
  double     rtk_max_down_difference_;
  double     rtk_max_abs_difference_;

private:
  ros::Timer main_timer_;
  void       mainTimer(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                            config_mutex_;
  typedef mrs_uav_odometry::odometry_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>       ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>              reconfigure_server_;
  mrs_uav_odometry::odometry_dynparamConfig         last_drs_config_;
};

//}

/* //{ onInit() */

void Odometry::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Odometry]: Initializing Odometry node");

  /* initialize variables //{ */

  odometry_published_       = false;
  got_odom_pixhawk_         = false;
  got_optflow_              = false;
  got_vio_                  = false;
  got_vslam_                = false;
  got_brick_pose_           = false;
  got_rtk_                  = false;
  got_odom_t265_            = false;
  got_rtk_fix_              = false;
  got_pixhawk_utm_          = false;
  got_control_manager_diag_ = false;
  got_home_position_fix_    = false;
  got_altitude_sensors_     = false;
  got_lateral_sensors_      = false;
  got_pixhawk_imu_          = false;
  got_compass_hdg_          = false;
  got_icp_twist_            = false;

  failsafe_called_               = false;
  hector_reset_called_           = false;
  _reset_hector_after_takeoff_   = false;
  _perform_hector_reset_routine_ = false;
  hector_reset_routine_running_  = false;
  hector_offset_ << 0, 0;
  hector_offset_hdg_   = 0;
  c_hector_msg_        = 0;
  c_hector_init_msgs_  = 0;
  estimator_iteration_ = 0;

  aloam_offset_ << 0, 0;
  aloam_offset_hdg_ = 0;

  liosam_offset_ << 0, 0;
  liosam_offset_hdg_ = 0;

  acc_global_prev_.x = 0.0;
  acc_global_prev_.y = 0.0;
  acc_global_prev_.z = 0.0;

  got_rtk_counter_ = 0;

  is_heading_estimator_initialized_ = false;
  init_hdg_avg_samples_             = 0;
  init_hdg_avg_                     = 0.0;
  init_hdg_avg_done_                = false;

  is_updating_state_     = false;
  finished_state_update_ = false;
  // got_brick_altitude = false;

  pixhawk_utm_position_x_ = 0;
  pixhawk_utm_position_y_ = 0;

  rtk_local_origin_z_     = 0;
  got_rtk_local_origin_z_ = false;

  pixhawk_odom_offset_x_ = 0;
  pixhawk_odom_offset_y_ = 0;

  counter_odom_brick_ = 0;

  accum_hdg_brick_                   = 0.0;
  compass_inconsistent_samples_      = 0;
  optflow_inconsistent_samples_      = 0;
  icp_hdg_rate_inconsistent_samples_ = 0;

  odom_pixhawk_last_update_ = ros::Time::now();

  garmin_enabled_       = true;
  sonar_enabled_        = true;
  rtk_altitude_enabled_ = false;

  odom_main_inno_.pose.pose.orientation.x = 0.0;
  odom_main_inno_.pose.pose.orientation.y = 0.0;
  odom_main_inno_.pose.pose.orientation.z = 0.0;
  odom_main_inno_.pose.pose.orientation.w = 1.0;

  //}

  // ------------------------------------------------------------------------
  // |                        odometry estimator type                       |
  // ------------------------------------------------------------------------

  /* check estimator type to name conversion //{ */

  // prepare the array of names
  // IMPORTANT, update this with each update of the EstimatorType message
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOW));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::GPS));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOWGPS));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::RTK));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::VIO));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::BRICK));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::T265));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::HECTOR));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::BRICKFLOW));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::VSLAM));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::ICP));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::ALOAM));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::ALOAMGARM));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::ALOAMREP));
  _estimator_type_names_.push_back(NAME_OF(mrs_msgs::EstimatorType::LIOSAM));

  ROS_WARN("[Odometry]: SAFETY Checking the EstimatorType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::EstimatorType::TYPE_COUNT; i++) {
    std::size_t found         = _estimator_type_names_[i].find_last_of(":");
    _estimator_type_names_[i] = _estimator_type_names_[i].substr(found + 1);
    ROS_INFO("[Odometry]: estimator_type_[%d]=%s", i, _estimator_type_names_[i].c_str());
  }

  // prepare the array of names
  // IMPORTANT, update this with each update of the AltitudeType message
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::HEIGHT));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::PLANE));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::BRICK));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::VIO));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::ALOAM));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::ALOAMGARM));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::ALOAMREP));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::BARO));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::RTK));
  _altitude_type_names_.push_back(NAME_OF(mrs_msgs::AltitudeType::LIOSAM));

  ROS_WARN("[Odometry]: SAFETY Checking the AltitudeType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::AltitudeType::TYPE_COUNT; i++) {
    std::size_t found        = _altitude_type_names_[i].find_last_of(":");
    _altitude_type_names_[i] = _altitude_type_names_[i].substr(found + 1);
    ROS_INFO("[Odometry]: _altitude_type[%d]=%s", i, _altitude_type_names_[i].c_str());
  }

  // prepare the array of names
  // IMPORTANT, update this with each update of the HeadingType message
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::PIXHAWK));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::GYRO));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::COMPASS));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::OPTFLOW));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::HECTOR));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::BRICK));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::VIO));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::VSLAM));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::ICP));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::BRICKFLOW));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::ALOAM));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::ALOAMREP));
  _heading_type_names_.push_back(NAME_OF(mrs_msgs::HeadingType::LIOSAM));

  ROS_WARN("[Odometry]: SAFETY Checking the HeadingType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::HeadingType::TYPE_COUNT; i++) {
    std::size_t found       = _heading_type_names_[i].find_last_of(":");
    _heading_type_names_[i] = _heading_type_names_[i].substr(found + 1);
    ROS_INFO("[Odometry]: _heading_type[%d]=%s", i, _heading_type_names_[i].c_str());
  }

  //}

  // | ------------------- parameters loading ------------------- |

  ROS_INFO("[Odometry]: Loading parameters");

  mrs_lib::ParamLoader param_loader(nh_, "Odometry");

  /* check the version //{ */

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Odometry]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  //}

  /* system parameters //{ */

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("hiccup_time_threshold", _hiccup_thr_);
  param_loader.loadParam("uav_mass", _uav_mass_estimate_);

  //}

  /* debug parameters //{ */

  param_loader.loadParam("debug/publish_corrections", _debug_publish_corrections_);
  param_loader.loadParam("debug/publish_fused_odom", _publish_fused_odom_);
  param_loader.loadParam("debug/publish_pixhawk_velocity", _publish_pixhawk_velocity_);
  param_loader.loadParam("debug/publish_servoing", _debug_publish_servoing_);
  param_loader.loadParam("debug/pass_rtk_as_odom", _pass_rtk_as_odom_);

  //}

  /* frame ids //{ */

  fcu_frame_id_                   = _uav_name_ + "/fcu";
  fcu_untilted_frame_id_          = _uav_name_ + "/fcu_untilted";
  local_origin_frame_id_          = _uav_name_ + "/local_origin";
  stable_origin_frame_id_         = _uav_name_ + "/stable_origin";
  aloam_mapping_origin_frame_id_  = _uav_name_ + "/aloam_mapping_origin";
  liosam_mapping_origin_frame_id_ = _uav_name_ + "/liosam_mapping_origin";
  last_local_name_                = _uav_name_ + "/null_origin";
  last_stable_name_               = _uav_name_ + "/null_origin";

  //}

  /* publish rates //{ */

  param_loader.loadParam("publish_rate/main", _main_rate_);
  param_loader.loadParam("publish_rate/slow", _slow_odom_rate_);
  param_loader.loadParam("publish_rate/diag", _diag_rate_);
  param_loader.loadParam("publish_rate/max_altitude", _max_altitude_rate_);
  param_loader.loadParam("publish_rate/est_states", _lkf_states_rate_);

  //}

  /* coordinate frames origins //{ */

  bool is_origin_param_ok = true;
  param_loader.loadParam("utm_origin_units", _utm_origin_units_);
  if (_utm_origin_units_ == 0) {
    ROS_INFO("[Odometry]: Loading UTM origin in UTM units.");
    is_origin_param_ok &= param_loader.loadParam("utm_origin_x", _utm_origin_x_);
    is_origin_param_ok &= param_loader.loadParam("utm_origin_y", _utm_origin_y_);
  } else {
    double lat, lon;
    ROS_INFO("[Odometry]: Loading UTM origin in LatLon units.");
    is_origin_param_ok &= param_loader.loadParam("utm_origin_lat", lat);
    is_origin_param_ok &= param_loader.loadParam("utm_origin_lon", lon);
    ROS_INFO("[Odometry]: Converted to UTM x: %f, y: %f.", _utm_origin_x_, _utm_origin_y_);
    mrs_lib::UTM(lat, lon, &_utm_origin_x_, &_utm_origin_y_);
  }

  is_origin_param_ok &= param_loader.loadParam("init_gps_origin_local", _init_gps_origin_local_);
  is_origin_param_ok &= param_loader.loadParam("init_gps_offset_x", _init_gps_offset_x_);
  is_origin_param_ok &= param_loader.loadParam("init_gps_offset_y", _init_gps_offset_y_);

  if (!is_origin_param_ok) {
    ROS_ERROR("[Odometry]: Could not load all mandatory parameters from world file. Please check your world file.");
    ros::shutdown();
  }

  //}

  /* sensor z offsets //{ */

  param_loader.loadParam("offset/garmin", _garmin_z_offset_);
  param_loader.loadParam("offset/sonar", _sonar_z_offset_);
  param_loader.loadParam("offset/fcu_height", _fcu_height_);

  //}

  /* altitude estimation parameters //{ */

  ROS_INFO("[Odometry]: Loading altitude estimation parameters");

  /* altitude estimators string names //{ */

  param_loader.loadParam("altitude_estimators/model_states", _alt_model_state_names_);
  param_loader.loadParam("altitude_estimators/measurements", _alt_measurement_names_);
  param_loader.loadParam("altitude_estimators/altitude_estimators", _altitude_estimators_names_);

  //}

  /* altitude median filters //{ */

  double buffer_size, max_valid, min_valid, max_diff;

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Garmin
  param_loader.loadParam("altitude/median_filter/garmin/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/garmin/max_diff", max_diff);
  alt_mf_garmin_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/sonar/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/sonar/max_diff", max_diff);
  alt_mf_sonar_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/plane/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/plane/max_diff", max_diff);
  alt_mf_plane_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/brick/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/brick/max_diff", max_diff);
  alt_mf_brick_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/vio/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/vio/max_diff", max_diff);
  alt_mf_vio_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/aloam/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/aloam/max_diff", max_diff);
  alt_mf_aloam_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  param_loader.loadParam("altitude/median_filter/liosam/buffer_size", buffer_size);
  param_loader.loadParam("altitude/median_filter/liosam/max_diff", max_diff);
  alt_mf_liosam_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  //}

  /* altitude measurement min and max value gates //{ */

  param_loader.loadParam("altitude/gate/garmin/min", _garmin_min_valid_alt_);
  param_loader.loadParam("altitude/gate/garmin/max", _garmin_max_valid_alt_);

  param_loader.loadParam("altitude/gate/sonar/min", _sonar_min_valid_alt_);
  param_loader.loadParam("altitude/gate/sonar/max", _sonar_max_valid_alt_);

  param_loader.loadParam("altitude/gate/plane/min", _plane_min_valid_alt_);
  param_loader.loadParam("altitude/gate/plane/max", _plane_max_valid_alt_);

  param_loader.loadParam("altitude/gate/brick/min", _brick_min_valid_alt_);
  param_loader.loadParam("altitude/gate/brick/max", _brick_max_valid_alt_);

  param_loader.loadParam("altitude/gate/vio/min", _vio_min_valid_alt_);
  param_loader.loadParam("altitude/gate/vio/max", _vio_max_valid_alt_);

  param_loader.loadParam("altitude/gate/aloam/min", _aloam_min_valid_alt_);
  param_loader.loadParam("altitude/gate/aloam/max", _aloam_max_valid_alt_);

  param_loader.loadParam("altitude/gate/liosam/min", _liosam_min_valid_alt_);
  param_loader.loadParam("altitude/gate/liosam/max", _liosam_max_valid_alt_);

  //}

  /* innovation gates //{ */

  double garmin_inno_gate_value_tmp;
  param_loader.loadParam("altitude/gate/garmin/use_inno_gate", _use_garmin_inno_gate_);
  param_loader.loadParam("altitude/gate/garmin/inno_gate_value", garmin_inno_gate_value_tmp);
  _garmin_inno_gate_value_sq_ = std::pow(garmin_inno_gate_value_tmp, 2);

  //}

  /* maximum altitude //{ */

  param_loader.loadParam("altitude/max_optflow", _max_optflow_altitude_);
  double max_default_altitude;
  param_loader.loadParam("altitude/max_default", max_default_altitude);
  param_loader.loadParam("safety_area/max_height", _max_default_altitude_, max_default_altitude);

  //}

  /* altitude correction saturation //{ */

  param_loader.loadParam("altitude/max_saturated_correction", _max_altitude_correction_);

  //}

  /* excessive tilt //{ */

  double excessive_tilt_tmp;
  param_loader.loadParam("altitude/excessive_tilt", excessive_tilt_tmp);
  _excessive_tilt_sq_ = std::pow(excessive_tilt_tmp, 2);

  //}

  /* altitude takeoff estimator type //{ */

  param_loader.loadParam("altitude_estimator", _altitude_estimator_name_);
  size_t pos_alt = std::distance(_altitude_type_names_.begin(), find(_altitude_type_names_.begin(), _altitude_type_names_.end(), _altitude_estimator_name_));

  _alt_estimator_type_takeoff.name = _altitude_estimator_name_;
  _alt_estimator_type_takeoff.type = (int)pos_alt;

  //}

  /* altitude measurements to estimators mapping //{ */

  for (std::vector<std::string>::iterator it = _altitude_estimators_names_.begin(); it != _altitude_estimators_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam("altitude_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _alt_measurement_names_)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_alt_estimator_measurement_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  //}

  /* altitude measurement to model state mapping //{ */

  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it != _alt_measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("altitude_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _alt_model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_alt_measurement_state_.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  //}

  /* altitude model state mapping (name to id) //{ */

  for (std::vector<std::string>::iterator it = _alt_model_state_names_.begin(); it != _alt_model_state_names_.end(); ++it) {

    alt_H_t temp_matrix;
    param_loader.loadMatrixStatic("altitude_estimators/state_mapping/" + *it, temp_matrix);

    map_alt_states_.insert(std::pair<std::string, alt_H_t>(*it, temp_matrix));
  }

  //}

  /* altitude measurement covariances (R matrices) //{ */

  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it != _alt_measurement_names_.end(); ++it) {

    alt_R_t temp_matrix;
    param_loader.loadMatrixStatic("altitude/R/" + *it, temp_matrix);

    alt_R_t R_alt = R_alt.Identity() * temp_matrix(0);
    map_alt_measurement_covariance_.insert(std::pair<std::string, alt_R_t>(*it, R_alt));
  }


  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it < _alt_measurement_names_.end(); it++) {
    map_alt_measurement_name_id_.insert(std::pair<std::string, int>(*it, (int)std::distance(_alt_measurement_names_.begin(), it)));
  }

  //}

  /* altitude process covariance (Q matrix) //{ */

  param_loader.loadMatrixStatic("altitude/Q", _Q_alt_);

  //}

  /* garmin enabled //{ */

  param_loader.loadParam("altitude/garmin_enabled", garmin_enabled_);

  //}

  param_loader.loadParam("altitude/use_rtk_altitude", _use_rtk_altitude_);

  //}

  /* create altitude estimator //{ */

  ROS_INFO("[Odometry]: Creating altitude estimators");

  // Loop through all estimators
  for (std::vector<std::string>::iterator it = _altitude_estimators_names_.begin(); it != _altitude_estimators_names_.end(); ++it) {

    std::vector<bool>    alt_fusing_measurement;
    std::vector<alt_H_t> H_multi_alt;
    std::vector<alt_R_t> R_multi_alt;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_alt_estimator_measurement_.find(*it);

    // Loop through all measurements
    for (std::vector<std::string>::iterator it2 = _alt_measurement_names_.begin(); it2 != _alt_measurement_names_.end(); ++it2) {

      // Check whether measurement is fused by the estimator
      if (stringInVector(*it2, temp_vec->second)) {
        alt_fusing_measurement.push_back(true);
      } else {
        alt_fusing_measurement.push_back(false);
      }
      ROS_WARN("[Odometry]: estimator: %s measurement: %s fusing: %s", it->c_str(), it2->c_str(), btoa(stringInVector(*it2, temp_vec->second)));

      // Find state name
      std::map<std::string, std::string>::iterator pair_measurement_state = map_alt_measurement_state_.find(*it2);

      // Find measurement to state mapping
      std::map<std::string, alt_H_t>::iterator pair_state_matrix = map_alt_states_.find(pair_measurement_state->second);
      H_multi_alt.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, alt_R_t>::iterator pair_measurement_covariance = map_alt_measurement_covariance_.find(*it2);
      R_multi_alt.push_back(pair_measurement_covariance->second);
    }

    // Add pointer to altitude estimator to array
    if (*it == "ALOAMREP") {
      _altitude_estimators_.insert(std::pair<std::string, std::shared_ptr<AltitudeEstimator>>(
          *it, std::make_shared<AltitudeEstimator>(*it, alt_fusing_measurement, H_multi_alt, _Q_alt_, R_multi_alt, true)));
    } else if (*it == "ALOAMGARM") {
      _altitude_estimators_.insert(std::pair<std::string, std::shared_ptr<AltitudeEstimatorAloamGarm>>(
          *it, std::make_shared<AltitudeEstimatorAloamGarm>(*it, alt_fusing_measurement, H_multi_alt, _Q_alt_, R_multi_alt, nh_, true)));
    } else {
      _altitude_estimators_.insert(std::pair<std::string, std::shared_ptr<AltitudeEstimator>>(
          *it, std::make_shared<AltitudeEstimator>(*it, alt_fusing_measurement, H_multi_alt, _Q_alt_, R_multi_alt)));
    }

    // Map odometry to estimator name
    mrs_msgs::Float64Stamped alt_msg;
    std::string              alt_estimator_name = *it;
    std::transform(alt_estimator_name.begin(), alt_estimator_name.end(), alt_estimator_name.begin(), ::tolower);
    map_alt_estimator_msg_.insert(std::pair<std::string, mrs_msgs::Float64Stamped>(*it, alt_msg));

    // Map publisher to estimator name
    ros::Publisher pub = nh_.advertise<mrs_msgs::Float64Stamped>(alt_estimator_name + "_out", 1);
    map_alt_estimator_pub_.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }

  // Height Garmin filter
  lkf_height_t::A_t A_height;
  A_height << 1;
  lkf_height_t::B_t B_height;
  B_height << 0;
  lkf_height_t::H_t H_height;
  H_height << 1;
  estimator_height_ = std::make_unique<lkf_height_t>(A_height, B_height, H_height);

  param_loader.loadMatrixStatic("height/R", _R_height_);
  param_loader.loadMatrixStatic("height/Q", _Q_height_);

  ROS_INFO("[Odometry]: Altitude estimator prepared");

  //}

  /*  //{ lateral estimation parameters*/

  ROS_INFO("[Odometry]: Loading lateral estimation parameters");

  /* lateral estimators string names //{ */

  param_loader.loadParam("state_estimators/state_estimators", _state_estimators_names_);
  param_loader.loadParam("state_estimators/active", _active_state_estimators_names_);
  param_loader.loadParam("state_estimators/model_states", _model_state_names_);
  param_loader.loadParam("state_estimators/measurements", _measurement_names_);


  //}

  /* gps lateral parameters //{ */

  param_loader.loadParam("lateral/gps_fallback/allowed", _gps_fallback_allowed_);
  param_loader.loadParam("lateral/gps_fallback/fallback_estimator", _gps_fallback_estimator_);
  param_loader.loadParam("lateral/gps_fallback/cov_limit", _gps_fallback_covariance_limit_);
  param_loader.loadParam("lateral/gps_fallback/cov_ok", _gps_fallback_covariance_ok_);
  param_loader.loadParam("lateral/gps_fallback/return_after_ok", _gps_return_after_fallback_);
  param_loader.loadParam("lateral/gps_fallback/bad_samples", _gps_fallback_bad_samples_);
  param_loader.loadParam("lateral/gps_fallback/good_samples", _gps_fallback_good_samples_);
  param_loader.loadParam("lateral/gps_fallback/altitude", _gps_fallback_altitude_);
  param_loader.loadParam("lateral/gps_fallback/altitude_wait_time", _gps_fallback_wait_for_altitude_time_);

  //}

  /* optic flow lateral parameters//{ */

  param_loader.loadParam("lateral/optflow/optimized_low", _use_optflow_low_);
  max_altitude_ = _max_default_altitude_;
  param_loader.loadParam("lateral/optflow/dynamic_cov", _dynamic_optflow_cov_);
  param_loader.loadParam("lateral/optflow/dynamic_cov_scale", _dynamic_optflow_cov_scale_);

  //}

  /* hector slam lateral parameters //{ */

  param_loader.loadParam("lateral/hector/reset_after_takeoff", _reset_hector_after_takeoff_);
  param_loader.loadParam("lateral/hector/reset_routine", _perform_hector_reset_routine_);

  //}

  /* rtk lateral parameters //{ */

  param_loader.loadMatrixStatic("lateral/rtk/A", _A_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/B", _B_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/H", _H_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/R", _R_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/Q", _Q_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/P", _P_lat_rtk_);
  param_loader.loadParam("lateral/rtk_fuse_sps", _rtk_fuse_sps_);

  //}

  /* brick lateral parameters //{ */

  param_loader.loadParam("lateral/brick/timeout", _brick_timeout_);

  //}

  /* lateral correction saturation //{ */

  // TODO revise
  param_loader.loadParam("lateral/saturate_mavros_position", _saturate_mavros_position_);
  param_loader.loadParam("lateral/max_mavros_pos_correction", _max_mavros_pos_correction_);
  param_loader.loadParam("lateral/max_vio_pos_correction", _max_vio_pos_correction_);
  param_loader.loadParam("lateral/max_vslam_pos_correction", _max_vslam_pos_correction_);
  param_loader.loadParam("lateral/max_brick_pos_correction", _max_brick_pos_correction_);
  param_loader.loadParam("lateral/max_rtk_pos_correction", _max_rtk_pos_correction_);
  param_loader.loadParam("lateral/max_t265_vel", _max_t265_vel_);
  double max_safe_brick_jump_tmp = 0.0;
  param_loader.loadParam("lateral/max_safe_brick_jump", max_safe_brick_jump_tmp);
  max_safe_brick_jump_sq_ = std::pow(max_safe_brick_jump_tmp, 2);

  //}

  /* lateral twist median filters //{ */

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Optflow twist median filter
  param_loader.loadParam("lateral/median_filter/optflow/use", _use_lat_mf_optflow_twist_);
  param_loader.loadParam("lateral/median_filter/optflow/buffer_size", buffer_size);
  param_loader.loadParam("lateral/median_filter/optflow/max_diff", max_diff);
  lat_mf_optflow_twist_x_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);
  lat_mf_optflow_twist_y_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  // ICP twist median filter
  param_loader.loadParam("lateral/median_filter/icp/use", _use_lat_mf_icp_twist_);
  param_loader.loadParam("lateral/median_filter/icp/buffer_size", buffer_size);
  param_loader.loadParam("lateral/median_filter/icp/max_diff", max_diff);
  lat_mf_icp_twist_x_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);
  lat_mf_icp_twist_y_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  //}

  /* lateral twist min and max value gates //{ */

  param_loader.loadParam("lateral/gate/optflow/max", _optflow_max_valid_twist_);

  param_loader.loadParam("lateral/gate/icp/max", _icp_max_valid_twist_);

  //}

  /* lateral takeoff estimator type //{ */

  std::string _takeoff_estimator_;
  param_loader.loadParam("lateral_estimator", _takeoff_estimator_);

  std::transform(_takeoff_estimator_.begin(), _takeoff_estimator_.end(), _takeoff_estimator_.begin(), ::toupper);
  size_t pos = std::distance(_estimator_type_names_.begin(), find(_estimator_type_names_.begin(), _estimator_type_names_.end(), _takeoff_estimator_));
  _estimator_type_takeoff_.name = _takeoff_estimator_;
  _estimator_type_takeoff_.type = (int)pos;

  //}

  /* lateral measurements to estimators mapping //{ */
  for (std::vector<std::string>::iterator it = _active_state_estimators_names_.begin(); it != _active_state_estimators_names_.end(); ++it) {

    /* set active estimators //{ */

    if (*it == "GPS") {
      gps_active_   = true;
      gps_reliable_ = true;
    }
    if (*it == "RTK") {
      rtk_active_   = true;
      rtk_reliable_ = true;
    }
    if (*it == "OPTFLOW") {
      optflow_active_   = true;
      optflow_reliable_ = true;
    }
    if (*it == "OPTFLOWGPS") {
      optflowgps_active_ = true;
    }
    if (*it == "HECTOR") {
      hector_active_   = true;
      hector_reliable_ = true;
    }
    if (*it == "ALOAM") {
      aloam_active_   = true;
      aloam_reliable_ = true;
    }
    if (*it == "ALOAMGARM") {
      aloamgarm_active_ = true;
      aloam_reliable_   = true;
    }
    if (*it == "ALOAMREP") {
      aloamrep_active_ = true;
      aloam_reliable_  = true;
    }
    if (*it == "LIOSAM") {
      liosam_active_   = true;
      liosam_reliable_ = true;
    }
    if (*it == "ICP") {
      icp_active_   = true;
      icp_reliable_ = true;
    }
    if (*it == "VIO") {
      vio_active_   = true;
      vio_reliable_ = true;
    }
    if (*it == "VSLAM") {
      vslam_active_   = true;
      vslam_reliable_ = true;
    }
    if (*it == "BRICK") {
      brick_active_   = true;
      brick_reliable_ = true;
    }
    if (*it == "T265") {
      t265_active_   = true;
      t265_reliable_ = true;
    }

    //}

    std::vector<std::string> temp_vector;
    param_loader.loadParam("state_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _measurement_names_)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_estimator_measurement_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  if (_pass_rtk_as_odom_ && !rtk_active_) {
    ROS_ERROR("[Odometry]: cannot have _pass_rtk_as_odom_ TRUE when RTK estimator is not active");
    ros::shutdown();
  }

  //}

  /* lateral measurement to model states mapping //{ */

  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it != _measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("state_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_measurement_state_.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  //}

  /* lateral model state mapping (name to id) //{ */

  for (std::vector<std::string>::iterator it = _model_state_names_.begin(); it != _model_state_names_.end(); ++it) {

    LatStateCol1D temp_P;
    param_loader.loadMatrixStatic("state_estimators/state_mapping/" + *it, temp_P);

    map_states_.insert(std::pair<std::string, LatStateCol1D>(*it, temp_P));
  }

  //}

  /* lateral measurements mapping (name to id) //{ */

  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it < _measurement_names_.end(); it++) {
    map_measurement_name_id_.insert(std::pair<std::string, int>(*it, (int)std::distance(_measurement_names_.begin(), it)));
  }

  //}

  /* list of estimators that should publish tf //{ */


  for (std::vector<std::string>::iterator it = _state_estimators_names_.begin(); it != _state_estimators_names_.end(); ++it) {

    bool publish_tf;
    param_loader.loadParam("state_estimators/publish_tf/" + *it, publish_tf, true);

    map_estimator_publish_tf_.insert(std::pair<std::string, bool>(*it, publish_tf));
  }

  //}

  /* lateral measurement covariances (R matrices) //{ */

  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it != _measurement_names_.end(); ++it) {

    Mat1 temp_matrix;
    param_loader.loadMatrixStatic("lateral/R/" + *it, temp_matrix);

    map_measurement_covariance_.insert(std::pair<std::string, Mat1>(*it, temp_matrix));
  }

  //}

  /* lateral process covariance (Q matrix) //{ */

  param_loader.loadMatrixStatic("lateral/Q", _Q_lat_);

  //}

  //}

  /*  //{ create lateral estimators */

  ROS_INFO("[Odometry]: Creating lateral estimators");

  for (std::vector<std::string>::iterator it = _active_state_estimators_names_.begin(); it != _active_state_estimators_names_.end(); ++it) {

    std::vector<bool>          fusing_measurement;
    std::vector<LatStateCol1D> P_arr_lat;
    std::vector<Mat1>          R_arr_lat;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_estimator_measurement_.find(*it);

    // Loop through all measurements
    for (std::vector<std::string>::iterator it2 = _measurement_names_.begin(); it2 != _measurement_names_.end(); ++it2) {

      // Check whether measurement is fused by the estimator
      if (stringInVector(*it2, temp_vec->second)) {
        fusing_measurement.push_back(true);
      } else {
        fusing_measurement.push_back(false);
      }

      // Find state name
      std::map<std::string, std::string>::iterator pair_measurement_state = map_measurement_state_.find(*it2);

      // Find state mapping
      std::map<std::string, LatStateCol1D>::iterator pair_state_matrix = map_states_.find(pair_measurement_state->second);
      P_arr_lat.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, Mat1>::iterator pair_measurement_covariance = map_measurement_covariance_.find(*it2);
      if (*it2 == "vel_optflow") {
        R_arr_lat.push_back(Mat1::Ones() * pair_measurement_covariance->second(0) * 1000);
      } else {
        R_arr_lat.push_back(Mat1::Ones() * pair_measurement_covariance->second(0));
      }
    }

    // Add state estimator to array
    if (*it == "ALOAMREP") {
      _lateral_estimators_.insert(std::pair<std::string, std::shared_ptr<StateEstimator>>(
          *it, std::make_shared<StateEstimator>(*it, fusing_measurement, _Q_lat_, P_arr_lat, R_arr_lat, true)));
    } else if (*it == "ALOAMGARM") {
      _lateral_estimators_.insert(std::pair<std::string, std::shared_ptr<StateEstimator>>(
          *it, std::make_shared<StateEstimator>(*it, fusing_measurement, _Q_lat_, P_arr_lat, R_arr_lat, true)));
    } else {
      _lateral_estimators_.insert(std::pair<std::string, std::shared_ptr<StateEstimator>>(
          *it, std::make_shared<StateEstimator>(*it, fusing_measurement, _Q_lat_, P_arr_lat, R_arr_lat)));
    }

    estimator_rtk_ = std::make_unique<lkf_rtk_t>(_A_lat_rtk_, _B_lat_rtk_, _H_lat_rtk_);

    const lkf_rtk_t::x_t        x0    = lkf_rtk_t::x_t::Zero();
    lkf_rtk_t::P_t              P_tmp = lkf_rtk_t::P_t::Identity();
    const lkf_rtk_t::P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
    const lkf_rtk_t::statecov_t sc0({x0, P0});
    sc_lat_rtk_ = sc0;

    // Map odometry to estimator name
    nav_msgs::Odometry odom;
    std::string        estimator_name = *it;
    std::transform(estimator_name.begin(), estimator_name.end(), estimator_name.begin(), ::tolower);
    odom.child_frame_id = estimator_name;
    map_estimator_odom_.insert(std::pair<std::string, nav_msgs::Odometry>(*it, odom));

    // Map publisher to estimator name
    ros::Publisher pub = nh_.advertise<nav_msgs::Odometry>("odom_" + estimator_name + "_out", 1);
    map_estimator_pub_.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }
  is_lateral_estimator_initialized_ = true;
  //}

  /* //{ heading estimation parameters */

  ROS_INFO("[Odometry]: Loading heading estimation parameters");

  param_loader.loadParam("heading/gyro_fallback", _gyro_fallback_);

  /* heading estimators string names //{ */

  param_loader.loadParam("heading_estimators/model_states", _hdg_model_state_names_);
  param_loader.loadParam("heading_estimators/measurements", _hdg_measurement_names_);
  param_loader.loadParam("heading_estimators/heading_estimators", _heading_estimators_names_);
  param_loader.loadParam("heading_estimators/active", _active_heading_estimators_names_);

  //}

  /* heading correction saturation //{ */

  param_loader.loadParam("heading/max_brick_hdg_correction", _max_brick_hdg_correction_);
  param_loader.loadParam("heading/accum_hdg_brick_alpha", _accum_hdg_brick_alpha_);
  double max_safe_brick_hdg_jump_tmp = 0.0;
  param_loader.loadParam("heading/max_safe_brick_hdg_jump", max_safe_brick_hdg_jump_tmp);
  max_safe_brick_hdg_jump_sq_ = std::pow(max_safe_brick_hdg_jump_tmp, 2);

  //}

  /* heading rate median filters //{ */

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Optflow heading rate median filter
  param_loader.loadParam("heading/median_filter/optflow/use", _use_hdg_mf_optflow_rate_);
  param_loader.loadParam("heading/median_filter/optflow/buffer_size", buffer_size);
  param_loader.loadParam("heading/median_filter/optflow/max_diff", max_diff);
  hdg_mf_optflow_rate_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  // ICP heading rate median filter
  param_loader.loadParam("heading/median_filter/icp/use", _use_hdg_mf_icp_rate_);
  param_loader.loadParam("heading/median_filter/icp/buffer_size", buffer_size);
  param_loader.loadParam("heading/median_filter/icp/max_diff", max_diff);
  hdg_mf_icp_rate_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff);

  //}

  /* heading rate min and max value gates //{ */

  param_loader.loadParam("heading/gate/optflow/max", _optflow_max_valid_hdg_rate_);

  param_loader.loadParam("heading/gate/icp/max", _icp_max_valid_hdg_rate_);

  //}

  /* takeoff estimator //{ */

  param_loader.loadParam("heading_estimator", _heading_estimator_name_);

  size_t pos_hdg = std::distance(_heading_type_names_.begin(), std::find(_heading_type_names_.begin(), _heading_type_names_.end(), _heading_estimator_name_));

  _hdg_estimator_type_takeoff_.name = _heading_estimator_name_;
  _hdg_estimator_type_takeoff_.type = (int)pos_hdg;

  // Check whether PIXHAWK is active
  if (!stringInVector("PIXHAWK", _active_heading_estimators_names_)) {
    gps_active_ = false;
    ROS_INFO("[Odometry]: PIXHAWK is not in the list of active heading estimators.");
  }


  //}

  /* heading measurements to estimators mapping //{ */

  for (std::vector<std::string>::iterator it = _active_heading_estimators_names_.begin(); it != _active_heading_estimators_names_.end(); ++it) {

    if (*it == "COMPASS") {
      compass_active_ = true;
    }

    if (!stringInVector(*it, _heading_estimators_names_)) {
      ROS_ERROR("[Odometry]: %s in the list of active heading estimators is not a valid heading estimator.", it->c_str());
      if (*it == "GPS") {
        ROS_ERROR("[Odometry]: You probably want PIXHAWK heading estimator.");
      }
      ros::shutdown();
    }

    std::vector<std::string> temp_vector;
    param_loader.loadParam("heading_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _hdg_measurement_names_)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_hdg_estimator_measurement_.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  //}

  /* heading measurements to model states mapping //{ */

  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it != _hdg_measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("heading_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _hdg_model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_hdg_measurement_state_.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  //}

  /* heading model state mapping (name to id) //{ */

  for (std::vector<std::string>::iterator it = _hdg_model_state_names_.begin(); it != _hdg_model_state_names_.end(); ++it) {

    hdg_H_t hdg_H;
    param_loader.loadMatrixStatic("heading_estimators/state_mapping/" + *it, hdg_H);

    map_hdg_states_.insert(std::pair<std::string, hdg_H_t>(*it, hdg_H));
  }

  //}

  /* heading measurements mapping (name to id) //{ */

  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it < _hdg_measurement_names_.end(); it++) {
    map_hdg_measurement_name_id_.insert(std::pair<std::string, int>(*it, (int)std::distance(_hdg_measurement_names_.begin(), it)));
  }
  for (auto &it : map_hdg_measurement_name_id_) {
    ROS_INFO("[Odometry]: heading measurement mapping: %s - %d", it.first.c_str(), it.second);
  }

  //}

  /* heading measurement covariances (R matrices) //{ */

  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it != _hdg_measurement_names_.end(); ++it) {

    hdg_R_t hdg_R;
    param_loader.loadMatrixStatic("heading/R/" + *it, hdg_R);

    map_hdg_measurement_covariance_.insert(std::pair<std::string, hdg_R_t>(*it, hdg_R));
  }

  //}

  /* heading process covariance (Q matrix) //{ */

  param_loader.loadMatrixStatic("heading/Q", _Q_hdg_);

  //}

  //}

  /* create heading estimator //{ */

  // Loop through all estimators
  for (std::vector<std::string>::iterator it = _active_heading_estimators_names_.begin(); it != _active_heading_estimators_names_.end(); ++it) {

    std::vector<bool>    hdg_fusing_measurement;
    std::vector<hdg_H_t> H_multi_hdg;
    std::vector<hdg_R_t> R_multi_hdg;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_hdg_estimator_measurement_.find(*it);

    // Loop through all measurements
    for (std::vector<std::string>::iterator it2 = _hdg_measurement_names_.begin(); it2 != _hdg_measurement_names_.end(); ++it2) {

      // Check whether measurement is fused by the estimator
      if (stringInVector(*it2, temp_vec->second)) {
        hdg_fusing_measurement.push_back(true);
      } else {
        hdg_fusing_measurement.push_back(false);
      }
      ROS_WARN("[Odometry]: estimator: %s measurement: %s fusing: %s", it->c_str(), it2->c_str(), btoa(stringInVector(*it2, temp_vec->second)));

      // Find state name
      std::map<std::string, std::string>::iterator pair_measurement_state = map_hdg_measurement_state_.find(*it2);

      // Find measurement to state mapping
      std::map<std::string, hdg_H_t>::iterator pair_state_matrix = map_hdg_states_.find(pair_measurement_state->second);
      H_multi_hdg.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, hdg_R_t>::iterator pair_measurement_covariance = map_hdg_measurement_covariance_.find(*it2);
      R_multi_hdg.push_back(pair_measurement_covariance->second);
    }

    std::cout << "H:" << H_multi_hdg.size() << std::endl;
    std::cout << "R:" << R_multi_hdg.size() << std::endl;

    // Add pointer to heading estimator to array
    if (*it == "ALOAMREP") {
      heading_estimators_.insert(std::pair<std::string, std::shared_ptr<HeadingEstimator>>(
          *it, std::make_shared<HeadingEstimator>(*it, hdg_fusing_measurement, H_multi_hdg, _Q_hdg_, R_multi_hdg, true)));
    } else {
      heading_estimators_.insert(std::pair<std::string, std::shared_ptr<HeadingEstimator>>(
          *it, std::make_shared<HeadingEstimator>(*it, hdg_fusing_measurement, H_multi_hdg, _Q_hdg_, R_multi_hdg)));
    }

    // Map odometry to estimator name
    mrs_msgs::Float64ArrayStamped hdg_msg;
    std::string                   hdg_estimator_name = *it;
    std::transform(hdg_estimator_name.begin(), hdg_estimator_name.end(), hdg_estimator_name.begin(), ::tolower);
    map_hdg_estimator_msg_.insert(std::pair<std::string, mrs_msgs::Float64ArrayStamped>(*it, hdg_msg));

    // Map publisher to heading estimator name
    ros::Publisher pub = nh_.advertise<mrs_msgs::Float64ArrayStamped>("hdg_" + hdg_estimator_name + "_out", 1);
    map_hdg_estimator_pub_.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }

  ROS_INFO("[Odometry]: Heading estimators initialized");

  //}

  // --------------------------------------------------------------
  // |                         tf listener                        |
  // --------------------------------------------------------------
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, "mrs_uav_odometry");
  transformer_     = mrs_lib::Transformer("Odometry", _uav_name_);


  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "Odometry", _profiler_enabled_);

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  /* //{ publishers */
  pub_uav_state_            = nh_.advertise<mrs_msgs::UavState>("uav_state_out", 1);
  pub_odom_main_            = nh_.advertise<nav_msgs::Odometry>("odom_main_out", 1);
  pub_odom_main_inno_       = nh_.advertise<nav_msgs::Odometry>("odom_main_inno_out", 1);
  pub_odom_local_           = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);
  pub_odom_stable_          = nh_.advertise<nav_msgs::Odometry>("odom_stable_out", 1);
  pub_slow_odom_            = nh_.advertise<nav_msgs::Odometry>("slow_odom_out", 1);
  pub_odom_mavros_          = nh_.advertise<nav_msgs::Odometry>("odom_mavros_out", 1);
  pub_esp_odom_             = nh_.advertise<mrs_msgs::EspOdometry>("esp_odom_out", 1);
  pub_odometry_diag_        = nh_.advertise<mrs_msgs::OdometryDiag>("odometry_diag_out", 1);
  pub_altitude_             = nh_.advertise<mrs_msgs::Float64Stamped>("altitude_out", 1);
  pub_height_               = nh_.advertise<mrs_msgs::Float64Stamped>("height_out", 1);
  pub_max_altitude_         = nh_.advertise<mrs_msgs::Float64Stamped>("max_altitude_out", 1);
  pub_orientation_          = nh_.advertise<nav_msgs::Odometry>("orientation_out", 1);
  pub_lkf_states_x_         = nh_.advertise<mrs_msgs::LkfStates>("lkf_states_x_out", 1);
  pub_lkf_states_y_         = nh_.advertise<mrs_msgs::LkfStates>("lkf_states_y_out", 1);
  pub_heading_states_       = nh_.advertise<mrs_msgs::EstimatedState>("heading_state_out", 1);
  pub_altitude_state_       = nh_.advertise<mrs_msgs::EstimatedState>("altitude_state_out", 1);
  pub_alt_cov_              = nh_.advertise<mrs_msgs::Float64ArrayStamped>("altitude_covariance_out", 1);
  pub_debug_optflow_filter_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("optflow_filtered_out", 1);
  pub_imu_untilted_         = nh_.advertise<sensor_msgs::Imu>("imu_untilted_out", 1);
  pub_brick_diag_           = nh_.advertise<mrs_msgs::ReferenceStamped>("brick_diag_out", 1);

  // republisher for rtk local
  pub_rtk_local_ = nh_.advertise<mrs_msgs::RtkGps>("rtk_local_out", 1);

  // republisher for rtk local odometry (e.g. for rviz)
  pub_rtk_local_odom_ = nh_.advertise<nav_msgs::Odometry>("rtk_local_odom_out", 1);

  // republisher for gps local odometry (e.g. for rviz)
  pub_gps_local_odom_ = nh_.advertise<nav_msgs::Odometry>("gps_local_odom_out", 1);

  // publisher for resetting hector map
  pub_hector_reset_ = nh_.advertise<std_msgs::String>("hector_map_reset_out", 1);

  // publisher for tf
  broadcaster_ = new mrs_lib::TransformBroadcaster();

  // publishers for orientations in local_origin frame
  pub_des_attitude_global_ = nh_.advertise<geometry_msgs::Vector3Stamped>("des_attitude_global_out", 1);
  pub_orientation_gt_      = nh_.advertise<geometry_msgs::Vector3Stamped>("orientation_gt_out", 1);
  pub_orientation_mavros_  = nh_.advertise<geometry_msgs::Vector3Stamped>("orientation_mavros_out", 1);

  // publishers for heading

  if (_debug_publish_corrections_) {
    pub_compass_hdg_corr_   = nh_.advertise<mrs_msgs::Float64Stamped>("debug_compass_hdg_corr_out", 1);
    pub_hector_hdg_corr_    = nh_.advertise<mrs_msgs::Float64Stamped>("debug_hector_hdg_corr_out", 1);
    pub_aloam_hdg_corr_     = nh_.advertise<mrs_msgs::Float64Stamped>("debug_aloam_hdg_corr_out", 1);
    pub_liosam_hdg_corr_    = nh_.advertise<mrs_msgs::Float64Stamped>("debug_liosam_hdg_corr_out", 1);
    pub_brick_hdg_corr_     = nh_.advertise<mrs_msgs::Float64Stamped>("debug_brick_hdg_corr_out", 1);
    pub_vio_hdg_corr_       = nh_.advertise<mrs_msgs::Float64Stamped>("debug_vio_hdg_corr_out", 1);
    pub_vslam_hdg_corr_     = nh_.advertise<mrs_msgs::Float64Stamped>("debug_vslam_hdg_corr_out", 1);
    pub_cmd_hdg_input_      = nh_.advertise<mrs_msgs::Float64Stamped>("debug_cmd_hdg_input_out", 1);
    pub_cmd_hdg_rate_input_ = nh_.advertise<mrs_msgs::Float64Stamped>("debug_cmd_hdg_rate_input_out", 1);

    pub_vel_baro_corr_           = nh_.advertise<mrs_msgs::Float64Stamped>("debug_vel_baro_out", 1);
    pub_vel_liosam_twist_z_corr_ = nh_.advertise<mrs_msgs::Float64Stamped>("debug_vel_liosam_twist_z_out", 1);
  }

  pub_debug_aloam_delay_ = nh_.advertise<mrs_msgs::Float64Stamped>("debug_aloam_delay", 1);

  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  /* //{ subscribers */
  // subsribe to target attitude
  sub_attitude_command_ = nh_.subscribe("attitude_command_in", 1, &Odometry::callbackAttitudeCommand, this, ros::TransportHints().tcpNoDelay());

  // subscribe to pixhawk imu
  sub_pixhawk_imu_ = nh_.subscribe("pixhawk_imu_in", 1, &Odometry::callbackPixhawkImu, this, ros::TransportHints().tcpNoDelay());

  // subscribe to compass heading
  if (compass_active_) {
    sub_pixhawk_compass_ = nh_.subscribe("pixhawk_compass_in", 1, &Odometry::callbackPixhawkCompassHdg, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to mavros odometry
  sub_pixhawk_ = nh_.subscribe("pixhawk_odom_in", 1, &Odometry::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());

  // subscriber to t265 odometry
  if (t265_active_) {
    sub_t265_odom_ = nh_.subscribe("t265_odom_in", 1, &Odometry::callbackT265Odometry, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to optflow velocity
  if (optflow_active_) {
    sub_optflow_ = nh_.subscribe("optflow_in", 1, &Odometry::callbackOptflowTwist, this, ros::TransportHints().tcpNoDelay());
    if (_use_optflow_low_) {
      sub_optflow_low_ = nh_.subscribe("optflow_low_in", 1, &Odometry::callbackOptflowTwistLow, this, ros::TransportHints().tcpNoDelay());
    }
  }

  // subscriber to visual odometry
  if (vio_active_) {
    sub_vio_ = nh_.subscribe("vio_in", 1, &Odometry::callbackVioOdometry, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to visual slam (pose)
  if (vslam_active_) {
    sub_vslam_ = nh_.subscribe("vslam_in", 1, &Odometry::callbackVslamPose, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to brick odometry
  if (brick_active_) {
    sub_brick_pose_ = nh_.subscribe("brick_pose_in", 1, &Odometry::callbackBrickPose, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for differential gps
  if (rtk_active_) {
    rtk_gps_sub_ = nh_.subscribe("rtk_gps_in", 1, &Odometry::callbackRtkGps, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for hector slam pose
  if (hector_active_) {
    sub_hector_pose_ = nh_.subscribe("hector_pose_in", 1, &Odometry::callbackHectorPose, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for icp twist
  if (icp_active_) {
    sub_icp_twist_ = nh_.subscribe("icp_twist_in", 1, &Odometry::callbackICPTwist, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for aloam odometry
  if (aloam_active_ || aloamgarm_active_) {
    sub_aloam_odom_ = nh_.subscribe("aloam_odom_in", 1, &Odometry::callbackAloamOdom, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for LIOSAM odometry
  if (liosam_active_) {
    sub_liosam_odom_ = nh_.subscribe("liosam_odom_in", 1, &Odometry::callbackLioSamOdom, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for garmin range
  sub_garmin_ = nh_.subscribe("garmin_in", 1, &Odometry::callbackGarmin, this, ros::TransportHints().tcpNoDelay());

  // subscriber for sonar range
  sub_sonar_ = nh_.subscribe("sonar_in", 1, &Odometry::callbackSonar, this, ros::TransportHints().tcpNoDelay());

  // subscriber for plane range
  sub_plane_ = nh_.subscribe("plane_in", 1, &Odometry::callbackPlane, this, ros::TransportHints().tcpNoDelay());

  // subscriber for ground truth
  sub_ground_truth_ = nh_.subscribe("ground_truth_in", 1, &Odometry::callbackGroundTruth, this, ros::TransportHints().tcpNoDelay());

  // subscribe for utm coordinates
  sub_global_position_ = nh_.subscribe("global_position_in", 1, &Odometry::callbackPixhawkUtm, this, ros::TransportHints().tcpNoDelay());

  // subscribe for control manager diagnostics
  sub_control_manager_diag_ = nh_.subscribe("control_manager_diag_in", 1, &Odometry::callbackControlManagerDiag, this, ros::TransportHints().tcpNoDelay());

  // subscribe for uav mass estimate
  sub_uav_mass_estimate_ = nh_.subscribe("uav_mass_estimate_in", 1, &Odometry::callbackUavMassEstimate, this, ros::TransportHints().tcpNoDelay());

  // subscribe for gps covariance
  if (_gps_fallback_allowed_) {
    sub_gps_covariance_ = nh_.subscribe("gps_covariance_in", 1, &Odometry::callbackGPSCovariance, this, ros::TransportHints().tcpNoDelay());
  }

  //}

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  /* //{ services */

  // subscribe for reset kalman service
  ser_reset_lateral_kalman_ = nh_.advertiseService("reset_lateral_kalman_in", &Odometry::callbackResetEstimator, this);

  // subscribe for reset hector service
  ser_reset_hector_ = nh_.advertiseService("reset_hector_in", &Odometry::callbackResetHector, this);

  // subscribe for reliable hector service
  ser_reliable_hector_ = nh_.advertiseService("reliable_hector_in", &Odometry::callbackReliableHector, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh_.advertiseService("toggle_garmin_in", &Odometry::callbackToggleGarmin, this);

  // change odometry source
  ser_change_odometry_source_ = nh_.advertiseService("change_odometry_source_in", &Odometry::callbackChangeOdometrySource, this);

  // change current estimator
  ser_change_estimator_type_ = nh_.advertiseService("change_estimator_type_in", &Odometry::callbackChangeEstimator, this);

  // change current estimator
  ser_change_estimator_type_string_ = nh_.advertiseService("change_estimator_type_string_in", &Odometry::callbackChangeEstimatorString, this);

  ser_change_hdg_estimator_type_ = nh_.advertiseService("change_hdg_estimator_type_in", &Odometry::callbackChangeHdgEstimator, this);

  ser_change_hdg_estimator_type_string_ = nh_.advertiseService("change_hdg_estimator_type_string_in", &Odometry::callbackChangeHdgEstimatorString, this);

  ser_change_alt_estimator_type_ = nh_.advertiseService("change_alt_estimator_type_in", &Odometry::callbackChangeAltEstimator, this);

  ser_change_alt_estimator_type_string_ = nh_.advertiseService("change_alt_estimator_type_string_in", &Odometry::callbackChangeAltEstimatorString, this);

  // subscribe for callbacks toggle service
  ser_toggle_callbacks_ = nh_.advertiseService("toggle_callbacks_in", &Odometry::callbackToggleCallbacks, this);

  // initialized service clients
  ser_client_failsafe_         = nh_.serviceClient<std_srvs::Trigger>("failsafe_out");
  ser_client_hover_            = nh_.serviceClient<std_srvs::Trigger>("hover_out");
  ser_client_reference_        = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("ereference_out");
  ser_client_ehover_           = nh_.serviceClient<std_srvs::Trigger>("ehover_out");
  ser_client_enable_callbacks_ = nh_.serviceClient<std_srvs::SetBool>("enable_callbacks_out");
  ser_client_tracker_          = nh_.serviceClient<mrs_msgs::String>("tracker_out");
  ser_client_controller_       = nh_.serviceClient<mrs_msgs::String>("controller_out");
  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timers //{ */

  main_timer_                 = nh_.createTimer(ros::Rate(_main_rate_), &Odometry::mainTimer, this);
  slow_odom_timer_            = nh_.createTimer(ros::Rate(_slow_odom_rate_), &Odometry::slowOdomTimer, this);
  diag_timer_                 = nh_.createTimer(ros::Rate(_diag_rate_), &Odometry::diagTimer, this);
  lat_states_timer_           = nh_.createTimer(ros::Rate(_lkf_states_rate_), &Odometry::lkfStatesTimer, this);
  max_altitude_timer_         = nh_.createTimer(ros::Rate(_max_altitude_rate_), &Odometry::maxAltitudeTimer, this);
  topic_watcher_timer_        = nh_.createTimer(ros::Rate(topic_watcher_rate_), &Odometry::topicWatcherTimer, this);
  hector_reset_routine_timer_ = nh_.createTimer(ros::Duration(0.00001), &Odometry::callbackTimerHectorResetRoutine, this, true, false);

  //}

  /* check validity and set takeoff estimator //{ */

  // If required sensor is not available shutdown
  ROS_INFO_ONCE("[Odometry]: Requested %s type for takeoff.", _estimator_type_takeoff_.name.c_str());
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::OPTFLOW && !optflow_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. OPTFLOW estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::GPS && !gps_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. GPS estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::OPTFLOWGPS && !optflow_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. OPTFLOW not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::RTK && !rtk_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. RTK estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::T265 && !t265_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. T265 estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::HECTOR && !hector_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. HECTOR esimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAM && !aloam_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ALOAM estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAMGARM && !aloamgarm_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ALOAMGARM estimator not active. Shutting down.",
              _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAMREP && !aloamrep_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ALOAMREP estimator not active. Shutting down.",
              _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::LIOSAM && !liosam_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. LIOSAM estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ICP && !icp_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ICP estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::VIO && !vio_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. VIO estimator not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::VSLAM && !vslam_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Visual SLAM localization not available. Shutting down.",
              _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::BRICKFLOW && !optflow_active_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. OPTFLOW not active. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::BRICK) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Takeoff in this odometry mode is not supported. Shutting down.",
              _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }

  //}

  /* switch to takeoff estimator //{ */

  bool success;

  success = changeCurrentHeadingEstimator(_hdg_estimator_type_takeoff_);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff heading estimator type %s could not be set. Shutting down.", _hdg_estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }

  success = changeCurrentAltitudeEstimator(_alt_estimator_type_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff altitude estimator type %s could not be set. Shutting down.", _alt_estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  success = changeCurrentEstimator(_estimator_type_takeoff_);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Shutting down.", _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  //}

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  /* pass current covariances to dynamic reconfigure //{ */

  // Lateral position measurement covariances
  last_drs_config_.R_pos_mavros = map_measurement_covariance_.find("pos_mavros")->second(0);
  last_drs_config_.R_pos_vio    = map_measurement_covariance_.find("pos_vio")->second(0);
  last_drs_config_.R_pos_vslam  = map_measurement_covariance_.find("pos_vslam")->second(0);
  last_drs_config_.R_pos_rtk    = map_measurement_covariance_.find("pos_rtk")->second(0);
  last_drs_config_.R_pos_brick  = map_measurement_covariance_.find("pos_brick")->second(0);
  last_drs_config_.R_pos_hector = map_measurement_covariance_.find("pos_hector")->second(0);
  last_drs_config_.R_pos_aloam  = map_measurement_covariance_.find("pos_aloam")->second(0);
  last_drs_config_.R_pos_liosam = map_measurement_covariance_.find("pos_liosam")->second(0);

  // Lateral velocity measurement covariances
  last_drs_config_.R_vel_mavros  = map_measurement_covariance_.find("vel_mavros")->second(0);
  last_drs_config_.R_vel_vio     = map_measurement_covariance_.find("vel_vio")->second(0);
  last_drs_config_.R_vel_icp     = map_measurement_covariance_.find("vel_icp")->second(0);
  last_drs_config_.R_vel_optflow = map_measurement_covariance_.find("vel_optflow")->second(0);
  last_drs_config_.R_vel_rtk     = map_measurement_covariance_.find("vel_rtk")->second(0);
  last_drs_config_.R_vel_liosam  = map_measurement_covariance_.find("vel_liosam")->second(0);

  // Lateral imu accelerations measurement covariances
  last_drs_config_.R_acc_imu_lat = map_measurement_covariance_.find("acc_imu")->second(0);

  // Lateral process covariances
  last_drs_config_.Q_lat_pos = _Q_lat_(0, 0);
  last_drs_config_.Q_lat_vel = _Q_lat_(1, 1);
  last_drs_config_.Q_lat_acc = _Q_lat_(2, 2);

  // Altitude measurement covariances
  last_drs_config_.R_height_range  = map_alt_measurement_covariance_.find("height_range")->second(0);
  last_drs_config_.R_height_plane  = map_alt_measurement_covariance_.find("height_plane")->second(0);
  last_drs_config_.R_height_brick  = map_alt_measurement_covariance_.find("height_brick")->second(0);
  last_drs_config_.R_height_vio    = map_alt_measurement_covariance_.find("height_vio")->second(0);
  last_drs_config_.R_height_aloam  = map_alt_measurement_covariance_.find("height_aloam")->second(0);
  last_drs_config_.R_height_liosam = map_alt_measurement_covariance_.find("height_liosam")->second(0);
  last_drs_config_.R_height_baro   = map_alt_measurement_covariance_.find("height_baro")->second(0);

  // Altitude process covariances
  last_drs_config_.Q_alt_pos = _Q_alt_(0, 0);
  last_drs_config_.Q_alt_vel = _Q_alt_(1, 1);
  last_drs_config_.Q_alt_acc = _Q_alt_(2, 2);

  // Altitude velocity measurement covariances
  last_drs_config_.R_vel_baro     = map_alt_measurement_covariance_.find("vel_baro")->second(0);
  last_drs_config_.R_vel_liosam_z = map_alt_measurement_covariance_.find("vel_liosam_z")->second(0);

  // Altitude acceleration measurement covariances
  last_drs_config_.R_acc_imu = map_alt_measurement_covariance_.find("acc_imu")->second(0);

  // Altitude input coefficient
  last_drs_config_.alt_input_coeff = 0.1;

  // Heading measurement covariances
  last_drs_config_.R_hdg_compass = map_hdg_measurement_covariance_.find("hdg_compass")->second(0);
  last_drs_config_.R_hdg_hector  = map_hdg_measurement_covariance_.find("hdg_hector")->second(0);
  last_drs_config_.R_hdg_aloam   = map_hdg_measurement_covariance_.find("hdg_aloam")->second(0);
  last_drs_config_.R_hdg_liosam  = map_hdg_measurement_covariance_.find("hdg_liosam")->second(0);
  last_drs_config_.R_hdg_brick   = map_hdg_measurement_covariance_.find("hdg_brick")->second(0);
  last_drs_config_.R_hdg_vio     = map_hdg_measurement_covariance_.find("hdg_vio")->second(0);
  last_drs_config_.R_hdg_vslam   = map_hdg_measurement_covariance_.find("hdg_vslam")->second(0);

  // Heading rate measurement covariances
  last_drs_config_.R_rate_gyro    = map_hdg_measurement_covariance_.find("rate_gyro")->second(0);
  last_drs_config_.R_rate_optflow = map_hdg_measurement_covariance_.find("rate_optflow")->second(0);
  last_drs_config_.R_rate_icp     = map_hdg_measurement_covariance_.find("rate_icp")->second(0);
  last_drs_config_.R_rate_liosam  = map_hdg_measurement_covariance_.find("rate_liosam")->second(0);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(last_drs_config_);
  ReconfigureServer::CallbackType f = boost::bind(&Odometry::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  //}

  // | ----------------------- finish init ---------------------- |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Odometry]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[Odometry]: initialized, version %s", VERSION);
}  // namespace mrs_uav_odometry

//}

/* isReadyToTakeoff() //{ */

bool Odometry::isReadyToTakeoff() {

  // Wait for necessary msgs
  ROS_INFO_ONCE("[Odometry]: Requested %s type for takeoff.", _estimator_type_takeoff_.name.c_str());
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::OPTFLOW) {
    if (got_optflow_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for OPTFLOW msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::GPS) {
    if (got_odom_pixhawk_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for PIXHAWK msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    if (got_optflow_ && got_odom_pixhawk_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for optic flow and pixhawk msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::RTK) {
    // return true, since RTK can work even with normal GPS
    return true;
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::T265) {
    if (got_odom_t265_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for T265 msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::VIO) {
    if (got_vio_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for VIO msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::VSLAM) {
    if (got_vslam_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for VSLAM msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::HECTOR) {
    if (got_hector_pose_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for HECTOR pose msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAM) {
    if (got_aloam_odom_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ALOAM odometry msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAMGARM) {
    if (got_aloam_odom_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ALOAM odometry msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ALOAMREP) {
    if (got_aloam_odom_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ALOAM odometry msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::LIOSAM) {
    if (got_liosam_odom_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for LIOSAM odometry msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::ICP) {
    if (got_icp_twist_) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ICP twist msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff_.type == mrs_msgs::EstimatorType::BRICK || _estimator_type_takeoff_.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Takeoff in this odometry mode is not supported. Shutting down.",
              _estimator_type_takeoff_.name.c_str());
    ros::shutdown();
  }
  return false;
}

//}

/* //{ isUavFlying() */

bool Odometry::isUavFlying() {

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  if (got_control_manager_diag_) {

    if (control_manager_diag.active_tracker == null_tracker_) {

      return false;
    } else {

      return true;
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: control manager diagnostics not available");
    return false;
  }
}

//}

/* //{ isUavLandoff() */

bool Odometry::isUavLandoff() {

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  if (got_control_manager_diag_) {

    if (control_manager_diag_.active_tracker == landoff_tracker_) {

      return true;
    } else {

      return false;
    }

  } else {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: control manager diagnostics not available");
    return false;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ mainTimer() */

void Odometry::mainTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("mainTimer", _main_rate_, 0.01, event);

  ros::Time t_start = ros::Time::now();

  // calculate time since last main timer tick
  double    dt;
  ros::Time time_now    = ros::Time::now();
  dt                    = (time_now - time_main_timer_prev_).toSec();
  time_main_timer_prev_ = time_now;

  // do not do anything the first main timer tick
  if (first_main_timer_tick_) {
    ROS_INFO("[Odometry]: Skipping first main timer tick.");
    first_main_timer_tick_ = false;
    return;
  }


  /* height estimator prediction //{ */

  // prediction step of height estimator (filtered garmin without baro)
  mrs_msgs::Float64Stamped height_msg;
  height_msg.header.frame_id = fcu_untilted_frame_id_;
  height_msg.header.stamp    = ros::Time::now();


  // publish estimated height only when we have garmin
  // otherwise publish -1, which is invalid height value
  // control will recognize this invalid value and will not use height for landing
  if (garmin_enabled_) {
    std::scoped_lock lock(mutex_estimator_height_);

    lkf_height_t::u_t u;
    u << 0;
    sc_height_       = estimator_height_->predict(sc_height_, u, _Q_height_, dt);
    height_msg.value = sc_height_.x(0);

  } else {
    height_msg.value = -1;
  }

  try {
    pub_height_.publish(height_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_height_.getTopic().c_str());
  }

  //}

  /* altitude estimator prediction //{ */

  double    alt_input       = mrs_lib::get_mutexed(mutex_alt_input_, alt_input_);
  ros::Time alt_input_stamp = mrs_lib::get_mutexed(mutex_alt_input_, alt_input_stamp_);
  altitudeEstimatorsPrediction(alt_input, dt, alt_input_stamp, time_now);

  //}

  /* heading estimator prediction //{ */

  double des_hdg;
  double des_hdg_rate;

  // set target attitude input to zero when not receiving target attitude msgs
  if (got_attitude_command_) {
    std::scoped_lock lock(mutex_attitude_command_);

    des_hdg      = des_hdg_;
    des_hdg_rate = des_hdg_rate_;
  } else {
    des_hdg      = 0.0;
    des_hdg_rate = 0.0;
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Not receiving target attitude.");
  }

  // apply prediction step to all heading estimators
  headingEstimatorsPrediction(des_hdg, des_hdg_rate, dt, alt_input_stamp, time_now);

  //}

  /* lateral estimator prediction //{ */

  geometry_msgs::Point   des_acc_point = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_.desired_acceleration);
  geometry_msgs::Vector3 des_acc;
  des_acc.x               = des_acc_point.x;
  des_acc.y               = des_acc_point.y;
  des_acc.z               = des_acc_point.z;
  ros::Time des_acc_stamp = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_.header.stamp);

  if (!is_updating_state_) {

    if (isUavFlying() && des_acc_stamp.toSec() > time_now.toSec() - 0.5) {
      stateEstimatorsPrediction(des_acc, dt, des_acc_stamp, time_now);
    } else {
      des_acc.x = 0.0;
      des_acc.y = 0.0;
      des_acc.z = 0.0;
      ROS_WARN_THROTTLE(2.0, "[Odometry] Last attitude command too old, passing zero input to state estimator.");
      stateEstimatorsPrediction(des_acc, dt, time_now, time_now);
    }

    // correction step for hector
    if (got_hector_pose_ && hector_corr_ready_) {
      auto pos_hector_x_tmp = mrs_lib::get_mutexed(mutex_hector_, pos_hector_x_);
      auto pos_hector_y_tmp = mrs_lib::get_mutexed(mutex_hector_, pos_hector_y_);
      stateEstimatorsCorrection(pos_hector_x_tmp, pos_hector_y_tmp, "pos_hector");
      auto hdg_hector_corr_tmp = mrs_lib::get_mutexed(mutex_hector_, hdg_hector_corr_);
      headingEstimatorsCorrection(hdg_hector_corr_tmp, "hdg_hector");
    }

    // correction step for aloam
    if (got_aloam_odom_ && aloam_corr_ready_) {
      auto pos_aloam_x_tmp     = mrs_lib::get_mutexed(mutex_aloam_, pos_aloam_x_);
      auto pos_aloam_y_tmp     = mrs_lib::get_mutexed(mutex_aloam_, pos_aloam_y_);
      auto aloam_timestamp_tmp = mrs_lib::get_mutexed(mutex_aloam_, aloam_timestamp_);
      stateEstimatorsCorrection(pos_aloam_x_tmp, pos_aloam_y_tmp, "pos_aloam", aloam_timestamp_tmp, time_now);
    }

    // correction step for liosam
    if (got_liosam_odom_ && liosam_corr_ready_) {
      auto pos_liosam_x_tmp = mrs_lib::get_mutexed(mutex_liosam_, pos_liosam_x_);
      auto pos_liosam_y_tmp = mrs_lib::get_mutexed(mutex_liosam_, pos_liosam_y_);
      stateEstimatorsCorrection(pos_liosam_x_tmp, pos_liosam_y_tmp, "pos_liosam");
    }
  } else {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Rotating lateral state. Skipping prediction.");
  }

  ROS_INFO_ONCE("[Odometry]: Prediction step of all state estimators running.");

  //}

  // return without publishing when pixhawk or rangefinder measurements are missing
  if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_)) {
    ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for altitude data from sensors - received? pixhawk: %s, ranger: %s", got_odom_pixhawk_ ? "TRUE" : "FALSE",
                      got_range_ ? "TRUE" : "FALSE");
    return;
  }

  /* publish altitude  //{ */

  got_altitude_sensors_ = true;

  mrs_msgs::Float64Stamped new_altitude;
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);

    new_altitude.header = odom_pixhawk_.header;
    new_altitude.value  = odom_pixhawk_.pose.pose.position.z;
  }

  new_altitude.header.frame_id = local_origin_frame_id_;
  new_altitude.header.stamp    = ros::Time::now();

  // update the altitude state
  alt_x_t alt_x = alt_x.Zero();
  {
    std::scoped_lock lock(mutex_altitude_estimator_);
    if (!current_alt_estimator_->getStates(alt_x)) {
      ROS_WARN("[Odometry]: Altitude estimator not initialized.");
      return;
    }
    if (alt_estimator_type_.type == mrs_msgs::AltitudeType::HEIGHT) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::PLANE) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::BRICK) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::VIO) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::ALOAM) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::ALOAMGARM) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::ALOAMREP) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::BARO) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::RTK) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (alt_estimator_type_.type == mrs_msgs::AltitudeType::LIOSAM) {
      new_altitude.value = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: unknown altitude type: %d, available types: %d, %d, %d, %d, %d, %d, %d, %d. Publishing mavros altitude instead.",
                         alt_estimator_type_.type, mrs_msgs::AltitudeType::HEIGHT, mrs_msgs::AltitudeType::PLANE, mrs_msgs::AltitudeType::BRICK,
                         mrs_msgs::AltitudeType::VIO, mrs_msgs::AltitudeType::ALOAM, mrs_msgs::AltitudeType::BARO, mrs_msgs::AltitudeType::RTK,
                         mrs_msgs::AltitudeType::LIOSAM);
    }
    ROS_INFO_ONCE("[Odometry]: Publishing altitude from estimator type: %d", alt_estimator_type_.type);
  }

  try {
    pub_altitude_.publish(new_altitude);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_altitude_.getTopic().c_str());
  }

  //}

  /* publish altitude state //{ */

  alt_P_t alt_P = alt_P.Zero();
  current_alt_estimator_->getStates(alt_x);
  current_alt_estimator_->getCovariance(alt_P);

  mrs_msgs::EstimatedState altitude_state_msg;
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);
    altitude_state_msg.header = odom_pixhawk_.header;
  }

  altitude_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < alt_x.size(); i++) {
    altitude_state_msg.state.push_back(alt_x(i));
    altitude_state_msg.covariance.push_back(alt_P(i, i));
  }
  try {
    pub_altitude_state_.publish(altitude_state_msg);
    ROS_INFO_ONCE("[Odometry]: Publishing altitude");
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_altitude_state_.getTopic().c_str());
  }

  //}


  /* initialize heading estimators //{ */

  // wait for initial averaging of compass heading
  if (!init_hdg_avg_done_ && toUppercase(current_hdg_estimator_name_) == "COMPASS") {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Waiting for averaging of initial heading.");
    return;
  }

  // initialize heading estimators
  if (!is_heading_estimator_initialized_) {

    // prepare initial state
    hdg_P_t init_cov = init_cov.Identity();
    init_cov *= 1000;
    double hdg = init_hdg_avg_;

    // set initial state to all estimators
    for (auto &estimator : heading_estimators_) {
      estimator.second->setState(0, hdg);
      estimator.second->setState(1, 0);
      estimator.second->setState(2, 0);
      estimator.second->setCovariance(init_cov);
    }
    is_heading_estimator_initialized_ = true;
  }

  //}

  /* publish heading  //{ */

  // odometry_msg for compatibility with other packages
  nav_msgs::Odometry orientation;

  // initialize with pixhawk orientation
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);
    orientation.header                = odom_pixhawk_.header;
    orientation.header.frame_id       = _uav_name_ + "/" + "gps_origin";
    orientation.child_frame_id        = fcu_frame_id_;
    orientation.pose.pose.orientation = odom_pixhawk_.pose.pose.orientation;
  }

  // get correct hdg if current heading estimator is not pixhawk
  if (current_hdg_estimator_->getName() != "PIXHAWK") {

    double hdg, hdg_rate;

    {
      std::scoped_lock lock(mutex_current_hdg_estimator_);

      current_hdg_estimator_->getState(0, hdg);
      current_hdg_estimator_->getState(1, hdg_rate);
    }

    hdg = radians::wrap(hdg);

    try {
      orientation.pose.pose.orientation = mrs_lib::AttitudeConverter(orientation.pose.pose.orientation).setHeading(hdg);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: %s:%d, could not set heading", __FILE__, __LINE__);
      orientation.pose.pose.orientation = mrs_lib::AttitudeConverter(orientation.pose.pose.orientation);
    }

    {
      std::scoped_lock lock(mutex_current_hdg_estimator_);
      orientation.header.frame_id = current_lat_estimator_->getName();
    }
  }

  try {
    pub_orientation_.publish(orientation);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_orientation_.getTopic().c_str());
  }

  //}

  /* sensor checking //{ */

  if (!is_ready_to_takeoff_) {
    is_ready_to_takeoff_ = isReadyToTakeoff();
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not ready to takeoff.");
    return;
  }

  if (failsafe_called_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Failsafe called");
    return;
  }

  // Fallback from PLANE
  if (current_alt_estimator_name_ == "PLANE") {
    if (!plane_reliable_) {
      ROS_WARN("[Odometry]: PLANE not reliable. Switching to HEIGHT type.");
      mrs_msgs::AltitudeType altitude_type;
      altitude_type.type = mrs_msgs::AltitudeType::HEIGHT;
      if (!changeCurrentAltitudeEstimator(altitude_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback height estimator not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
  }

  // Fallback from RTK
  if (estimator_type_.type == mrs_msgs::EstimatorType::RTK) {
    if (!gps_reliable_ && optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: RTK not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_)) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE", got_rtk_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from GPS
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::GPS) {
    if (!gps_reliable_ && optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_)) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from OPTFLOWGPS
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    if (!gps_reliable_ && optflow_active_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_optflow_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_optflow_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }


    // Fallback from T265
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::T265) {
    if ((!got_odom_t265_ || !t265_reliable_) && optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: T265 not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    } else if ((!got_odom_t265_ || !t265_reliable_) && gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
      ROS_WARN("[Odometry]: T265 not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_odom_t265_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, t265: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_odom_t265_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from Hector Slam
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::HECTOR) {

    if (!got_hector_pose_ || !hector_reliable_) {

      if (icp_active_ && got_icp_twist_) {

        if (_perform_hector_reset_routine_ && !hector_reset_routine_running_) {

          ROS_WARN("[Odometry]: HECTOR not reliable. Performing HECTOR reset routine.");
          hector_reset_routine_timer_.start();

        } else {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Hector heading not reliable. Switching to ICP heading estimator.");
          mrs_msgs::HeadingType desired_estimator;
          desired_estimator.type = mrs_msgs::HeadingType::ICP;
          desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
          changeCurrentHeadingEstimator(desired_estimator);
          ROS_WARN("[Odometry]: HECTOR not reliable. Switching to ICP type.");
          mrs_msgs::EstimatorType icp_type;
          icp_type.type = mrs_msgs::EstimatorType::ICP;
          if (!changeCurrentEstimator(icp_type)) {
            ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
            std_srvs::Trigger failsafe_out;
            ser_client_failsafe_.call(failsafe_out);
            failsafe_called_ = true;
          }
        }
      } else if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        if (_perform_hector_reset_routine_ && !hector_reset_routine_running_) {

          ROS_WARN("[Odometry]: HECTOR not reliable. Performing HECTOR reset routine.");
          hector_reset_routine_timer_.start();

        } else {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Hector heading not reliable. Switching to OPTFLOW heading estimator.");
          mrs_msgs::HeadingType desired_estimator;
          desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
          desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
          changeCurrentHeadingEstimator(desired_estimator);
          ROS_WARN("[Odometry]: HECTOR not reliable. Switching to OPTFLOW type.");
          mrs_msgs::EstimatorType optflow_type;
          optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
          if (!changeCurrentEstimator(optflow_type)) {
            ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
            std_srvs::Trigger failsafe_out;
            ser_client_failsafe_.call(failsafe_out);
            failsafe_called_ = true;
          }
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Hector heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: HECTOR not reliable. Switching to PIXHAWK type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else {
        if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_hector_pose_) {
          ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, hector: %s",
                            got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                            got_hector_pose_ ? "TRUE" : "FALSE");
          if (got_lateral_sensors_ && !failsafe_called_) {
            ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
            std_srvs::Trigger failsafe_out;
            ser_client_failsafe_.call(failsafe_out);
            failsafe_called_ = true;
          }
          return;
        }
      }
    }

    // Fallback from ALOAM Slam
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::ALOAM) {
    if (!got_aloam_odom_ || !aloam_reliable_) {
      if (icp_active_ && got_icp_twist_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to ICP heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to ICP type.");
        mrs_msgs::EstimatorType icp_type;
        icp_type.type = mrs_msgs::EstimatorType::ICP;
        if (!changeCurrentEstimator(icp_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_aloam_odom_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, aloam: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_aloam_odom_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from ALOAMGARM Slam
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::ALOAMGARM) {
    if (!got_aloam_odom_ || !aloam_reliable_) {
      if (icp_active_ && got_icp_twist_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to ICP heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to ICP type.");
        mrs_msgs::EstimatorType icp_type;
        icp_type.type = mrs_msgs::EstimatorType::ICP;
        if (!changeCurrentEstimator(icp_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_aloam_odom_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, aloam: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_aloam_odom_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from ALOAMREP Slam
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::ALOAMREP) {
    if (!got_aloam_odom_ || !aloam_reliable_) {
      if (icp_active_ && got_icp_twist_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to ICP heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to ICP type.");
        mrs_msgs::EstimatorType icp_type;
        icp_type.type = mrs_msgs::EstimatorType::ICP;
        if (!changeCurrentEstimator(icp_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_aloam_odom_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, aloam: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_aloam_odom_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from LIOSAM SLAM
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::LIOSAM) {
    if (!got_liosam_odom_ || !liosam_reliable_) {
      if (icp_active_ && got_icp_twist_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: LIOSAM heading not reliable. Switching to ICP heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: LIOSAM not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType icp_type;
        icp_type.type = mrs_msgs::EstimatorType::ICP;
        if (!changeCurrentEstimator(icp_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: LIOSAM heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: LIOSAM not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: LIOSAM heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: LIOSAM not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_liosam_odom_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, liosam: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_liosam_odom_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

    // Fallback from ICP
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::ICP) {

    if (!got_icp_twist_ || !icp_reliable_) {
      if (optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ICP not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ICP not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_icp_twist_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, icp: %s",
                        got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                        got_hector_pose_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }
    // Fallback from BRICK
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::BRICK) {
    if (!got_brick_pose_ || !brick_reliable_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: BRICK altitude not reliable. Switching to HEIGHT altitude estimator.");
      mrs_msgs::AltitudeType desired_alt_estimator;
      desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
      desired_alt_estimator.name = _altitude_estimators_names_[desired_alt_estimator.type];
      changeCurrentAltitudeEstimator(desired_alt_estimator);
      ROS_WARN_THROTTLE(1.0, "[Odometry]: BRICK heading not reliable. Switching to fallback heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = fallback_brick_hdg_estimator_type_.type;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      ROS_WARN("[Odometry]: BRICK not reliable. Switching to %s type.", _estimator_type_names_[fallback_brick_estimator_type_.type].c_str());
      if (!changeCurrentEstimator(fallback_brick_estimator_type_)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }

    // Fallback from OPTFLOW
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::OPTFLOW) {
    if (gps_active_ && gps_reliable_) {

      if (!optflow_reliable_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: OTPFLOW heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: OPTFLOW not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_optflow_) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                          got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                          got_optflow_ ? "TRUE" : "FALSE");
        if (got_lateral_sensors_ && !failsafe_called_) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
        return;
      }

    } else {
      if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_optflow_) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, optflow: %s", got_odom_pixhawk_ ? "TRUE" : "FALSE",
                          got_range_ ? "TRUE" : "FALSE", got_optflow_ ? "TRUE" : "FALSE");
        if (got_lateral_sensors_ && !failsafe_called_) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
        return;
      }
    }

    // Fallback from BRICKFLOW
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    if (gps_active_ && gps_reliable_) {

      if (!optflow_reliable_) {
        ROS_WARN("[Odometry]: BRICKFLOW not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
      } else if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_optflow_) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                          got_odom_pixhawk_ ? "TRUE" : "FALSE", got_range_ ? "TRUE" : "FALSE", got_pixhawk_utm_ ? "TRUE" : "FALSE",
                          got_optflow_ ? "TRUE" : "FALSE");
        if (got_lateral_sensors_ && !failsafe_called_) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
        return;
      }

    } else {
      if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_optflow_) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, optflow: %s", got_odom_pixhawk_ ? "TRUE" : "FALSE",
                          got_range_ ? "TRUE" : "FALSE", got_optflow_ ? "TRUE" : "FALSE");
        if (got_lateral_sensors_ && !failsafe_called_) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called_ = true;
        }
        return;
      }
    }
    // Fallback from VIO
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::VIO) {
    if (!vio_reliable_ && optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: VIO not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    } else if (!vio_reliable_ && gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
      ROS_WARN("[Odometry]: VIO not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_vio_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vio: %s", got_odom_pixhawk_ ? "TRUE" : "FALSE",
                        got_range_ ? "TRUE" : "FALSE", got_vio_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }
    // Fallback from VSLAM
  } else if (estimator_type_.type == mrs_msgs::EstimatorType::VSLAM) {
    if (!vslam_reliable_ && optflow_active_ && got_optflow_ && alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: VSLAM not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    } else if (!vslam_reliable_ && gps_active_ && gps_reliable_ && got_odom_pixhawk_) {
      ROS_WARN("[Odometry]: VSLAM not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
    }
    if (!got_odom_pixhawk_ || (!got_range_ && garmin_enabled_) || !got_vslam_) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vslam: %s", got_odom_pixhawk_ ? "TRUE" : "FALSE",
                        got_range_ ? "TRUE" : "FALSE", got_vslam_ ? "TRUE" : "FALSE");
      if (got_lateral_sensors_ && !failsafe_called_) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called_ = true;
      }
      return;
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Unknown odometry type. Not checking sensors.");
  }

  got_lateral_sensors_ = true;
  ROS_INFO_ONCE("[Odometry]: Lateral sensors ready");


  //}

  /* reset Hector map after takeoff //{ */


  // Call reset of hector map after taking off - cleans up garbage integrated into map during takeoff
  if (_reset_hector_after_takeoff_ && isUavFlying() && !isUavLandoff() && !hector_reset_called_) {

    for (auto &estimator : _lateral_estimators_) {
      if (estimator.first == "HECTOR") {
        Vec2 new_offset;
        estimator.second->getState(0, new_offset);
        hector_offset_ += new_offset;
      }
    }

    for (auto &estimator : heading_estimators_) {
      if (estimator.first == "HECTOR") {
        double tmp_hdg_offset;
        estimator.second->getState(0, tmp_hdg_offset);
        hector_offset_hdg_ += tmp_hdg_offset;
      }
    }

    ROS_INFO("[Odometry]: Calling Hector map reset.");
    std_msgs::String reset_msg;
    reset_msg.data = "reset";
    try {
      pub_hector_reset_.publish(reset_msg);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_hector_reset_.getTopic().c_str());
    }
    hector_reset_called_ = true;
    ROS_INFO("[Odometry]: Hector map reset called.");
  }

  //}

  // Just return without publishing - the t265 odometry is republished in callback at faster rate
  if (toUppercase(current_lat_estimator_name_) == "T265") {

    return;
  }

  // Prepare mavros odometry for later use
  auto odom_pixhawk_shifted_local = mrs_lib::get_mutexed(mutex_odom_pixhawk_shifted_, odom_pixhawk_shifted_);

  geometry_msgs::Quaternion mavros_orientation = odom_pixhawk_shifted_local.pose.pose.orientation;

  /* publish aux odometry //{ */

  // Loop through each estimator
  for (auto &estimator : _lateral_estimators_) {

    std::map<std::string, nav_msgs::Odometry>::iterator odom_aux = map_estimator_odom_.find(estimator.first);

    // Initialize odom_aux with pixhawk_odom to obtain attitude and attitude_rate which are not estimated by us
    mrs_lib::set_mutexed(mutex_odom_pixhawk_shifted_, odom_pixhawk_shifted_, odom_aux->second);

    std::string estimator_name = estimator.first;
    std::transform(estimator_name.begin(), estimator_name.end(), estimator_name.begin(), ::tolower);
    odom_aux->second.header.frame_id = _uav_name_ + "/" + estimator_name + "_origin";
    odom_aux->second.header.stamp    = time_now;
    odom_aux->second.child_frame_id  = fcu_frame_id_;

    alt_x_t alt_x = alt_x.Zero();
    // update the altitude state
    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      if (!current_alt_estimator_->getStates(alt_x)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    double alt;
    if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "PLANE") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    } else if (estimator.first == "PLANE") {
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "PLANE") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    } else if (estimator.first == "VIO") {
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "VIO") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    } else if (estimator.first == "ALOAM") {
      // Prevent publishing of TF before corrections are available
      if (!got_aloam_odom_) {
        continue;
      }
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "ALOAM") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    } else if (estimator.first == "ALOAMGARM") {
      // Prevent publishing of TF before corrections are available
      if (!got_aloam_odom_) {
        continue;
      }
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "ALOAMGARM") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
      // we might want other than height estimator when in GPS (baro)
    } else if (estimator.first == "ALOAMREP") {
      // Prevent publishing of TF before corrections are available
      if (!got_aloam_odom_) {
        continue;
      }
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "ALOAMREP") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }

      for (auto &alt_estimator : _altitude_estimators_) {
        if (_use_rtk_altitude_ && alt_estimator.first == "RTK") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    } else if (estimator.first == "LIOSAM") {
      // Prevent publishing of TF before corrections are available
      if (!got_liosam_odom_) {
        continue;
      }
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "LIOSAM") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
      // we might want other than height estimator when in GPS (baro)
    } else if (estimator.first == "GPS") {
      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        if (!current_alt_estimator_->getStates(alt_x)) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
          return;
        }
        odom_aux->second.pose.pose.position.z = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
      }
    } else {
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "HEIGHT") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt;
        }
      }
    }


    if (alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) == _fcu_height_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Suspicious height detected: %f, %f, %f. Check if altitude fusion is running correctly",
                        alt_x(mrs_msgs::AltitudeStateNames::HEIGHT), alt_x(mrs_msgs::AltitudeStateNames::VELOCITY),
                        alt_x(mrs_msgs::AltitudeStateNames::ACCELERATION));
    }

    Vec2 pos_vec;
    Vec2 vel_vec;

    if (toUppercase(estimator.second->getName()) == "RTK") {
      {
        std::scoped_lock lock(mutex_rtk_est_);

        pos_vec(0) = sc_lat_rtk_.x(0);
        pos_vec(1) = sc_lat_rtk_.x(1);
      }
    } else {

      estimator.second->getState(0, pos_vec);
    }
    estimator.second->getState(1, vel_vec);

    odom_aux->second.pose.pose.position.x = pos_vec(0);
    odom_aux->second.pose.pose.position.y = pos_vec(1);


    // Loop through each heading estimator
    double hdg;
    for (auto &hdg_estimator : heading_estimators_) {

      if (hdg_estimator.first == estimator.first || (hdg_estimator.first == "BRICK" && estimator.first == "BRICKFLOW") ||
          (hdg_estimator.first == "ALOAM" && (estimator.first == "ALOAMGARM" || estimator.first == "ALOAMREP"))) {

        hdg_estimator.second->getState(0, hdg);

        // Obtain mavros orientation
        tf2::Quaternion tf2_mavros_orient = mrs_lib::AttitudeConverter(mavros_orientation);

        // Obtain heading from mavros orientation
        double mavros_hdg = 0;
        try {
          mavros_hdg = mrs_lib::AttitudeConverter(mavros_orientation).getHeading();
        }
        catch (...) {
          ROS_WARN("[Odometry]: failed to getHeading() from mavros_orientation");
        }

        // Build rotation matrix from difference between new heading and mavros heading
        tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - mavros_hdg, Eigen::Vector3d::UnitZ()));

        // Transform the mavros orientation by the rotation matrix
        geometry_msgs::Quaternion new_orientation = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_mavros_orient);

        odom_aux->second.pose.pose.orientation = new_orientation;
      }
    }

    // Pass RTK through
    if (estimator.second->getName() == "RTK" && _pass_rtk_as_odom_) {

      mrs_lib::set_mutexed(mutex_rtk_local_odom_, rtk_local_odom_, odom_aux->second);
    }

    // Get inverse trasnform
    tf2::Transform tf_inv        = mrs_uav_odometry::tf2FromPose(odom_aux->second.pose.pose);
    tf_inv                       = tf_inv.inverse();
    geometry_msgs::Pose pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

    // publish TF
    if (map_estimator_publish_tf_.find(estimator.second->getName())->second) {
      geometry_msgs::TransformStamped tf;
      tf.header.stamp          = time_now;
      tf.header.frame_id       = fcu_frame_id_;
      tf.child_frame_id        = odom_aux->second.header.frame_id;
      tf.transform.translation = pointToVector3(pose_inv.position);
      tf.transform.rotation    = pose_inv.orientation;
      if (noNans(tf)) {
        try {
          broadcaster_->sendTransform(tf);
        }
        catch (...) {
          ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
        }
      } else {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Indian flatbread detected in transform from %s to %s. Not publishing tf.", odom_aux->second.header.frame_id.c_str(),
                          fcu_frame_id_.c_str());
      }
    }

    // Transform global velocity to twist in fcu frame
    geometry_msgs::Vector3Stamped global_vel;
    global_vel.header.frame_id = odom_aux->second.header.frame_id;
    global_vel.header.stamp    = ros::Time::now();
    global_vel.vector.x        = vel_vec(0);
    global_vel.vector.y        = vel_vec(1);
    global_vel.vector.z        = alt_x(mrs_msgs::AltitudeStateNames::VELOCITY);

    geometry_msgs::Vector3Stamped body_vel;
    auto                          response = transformer_.transformSingle(fcu_frame_id_, global_vel);
    if (response) {
      body_vel = response.value();
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed when publishing odom_aux.", global_vel.header.frame_id.c_str(), fcu_frame_id_.c_str());
      return;
    }

    odom_aux->second.twist.twist.linear.x = body_vel.vector.x;
    odom_aux->second.twist.twist.linear.y = body_vel.vector.y;
    odom_aux->second.twist.twist.linear.z = body_vel.vector.z;

    std::map<std::string, ros::Publisher>::iterator pub_odom_aux = map_estimator_pub_.find(estimator.second->getName());

    // Publish odom
    try {
      pub_odom_aux->second.publish(odom_aux->second);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_aux->second.getTopic().c_str());
    }
  }

  // publish the static transform between utm and local gps origin
  if (gps_active_) {

    // publish TF
    geometry_msgs::TransformStamped tf;

    tf.header.stamp            = ros::Time::now();
    tf.header.frame_id         = _uav_name_ + "/gps_origin";
    tf.child_frame_id          = _uav_name_ + "/utm_origin";
    tf.transform.translation.x = -_utm_origin_x_;
    tf.transform.translation.y = -_utm_origin_y_;

    tf.transform.rotation.x = 0;
    tf.transform.rotation.y = 0;
    tf.transform.rotation.z = 0;
    tf.transform.rotation.w = 1;

    if (noNans(tf)) {
      try {
        broadcaster_->sendTransform(tf);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
      }
    }
  }

  // Loop through each heading estimator
  for (auto &estimator : heading_estimators_) {

    mrs_msgs::Float64ArrayStamped heading_aux;

    heading_aux.header.frame_id = _uav_name_ + "/" + toLowercase(estimator.first) + "_origin";
    heading_aux.header.stamp    = time_now;

    hdg_x_t current_heading = current_heading.Zero();
    // update the altitude state
    {
      std::scoped_lock lock(mutex_heading_estimator_);
      if (!estimator.second->getStates(current_heading)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Heading estimator not initialized.");
        return;
      }
    }

    current_heading(0) = radians::wrap(current_heading(0));

    for (int i = 0; i < current_heading.rows(); i++) {
      heading_aux.values.push_back(current_heading(i));
    }

    std::map<std::string, ros::Publisher>::iterator pub_hdg_aux = map_hdg_estimator_pub_.find(estimator.second->getName());

    try {
      pub_hdg_aux->second.publish(heading_aux);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_hdg_aux->second.getTopic().c_str());
    }
  }
  ROS_INFO_ONCE("[Odometry]: Publishing auxiliary odometry");

  //}

  /* publish fused odometry //{ */

  // blocking/returning when cannot calculate utm_origin_offset
  if (gps_active_ && !calculatePixhawkOdomOffset()) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Cannot calculate pixhawk odom offset.");
    return;
  }

  if (gps_active_ && !got_pixhawk_odom_shifted_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for pixhawk odom offset");
    return;
  }

  // Initialize uav_state
  mrs_msgs::UavState uav_state;
  uav_state.pose.orientation.w        = 1.0;
  uav_state.estimator_iteration       = 0;
  uav_state.estimator_horizontal.type = mrs_msgs::EstimatorType::TYPE_COUNT;
  uav_state.estimator_vertical.type   = mrs_msgs::AltitudeType::TYPE_COUNT;
  uav_state.estimator_heading.type    = mrs_msgs::HeadingType::TYPE_COUNT;

  // initialized odom_main from pixhawk odometry to obtain attitude and angular rate which is not estimated by us
  nav_msgs::Odometry odom_main;

  odom_main                  = odom_pixhawk_shifted_local;
  uav_state.pose.position    = odom_pixhawk_shifted_local.pose.pose.position;
  uav_state.velocity.linear  = odom_pixhawk_shifted_local.twist.twist.linear;
  uav_state.pose.orientation = odom_pixhawk_shifted_local.pose.pose.orientation;
  uav_state.velocity.angular = odom_pixhawk_shifted_local.twist.twist.angular;

  // Fill in odometry headers according to the uav name and current estimator
  odom_main.header.stamp    = ros::Time::now();
  odom_main.header.frame_id = _uav_name_ + "/" + toLowercase(current_lat_estimator_name_) + "_origin";
  odom_main.child_frame_id  = fcu_frame_id_;
  uav_state.header.stamp    = ros::Time::now();
  uav_state.header.frame_id = _uav_name_ + "/" + toLowercase(current_lat_estimator_name_) + "_origin";
  uav_state.child_frame_id  = fcu_frame_id_;

  /* initialize lateral kalman filters //{ */

  if (!odometry_published_) {

    ROS_INFO("[Odometry]: Initializing the states of all estimators");

    for (auto &estimator : _lateral_estimators_) {

      Eigen::VectorXd pos_state(2);
      Eigen::VectorXd vel_state(2);
      vel_state << 0, 0;

      // estimators not based on GNSS
      if (estimator.second->getName() == "OPTFLOW" || estimator.second->getName() == "HECTOR" || estimator.second->getName() == "ALOAM" ||
          estimator.second->getName() == "ALOAMGARM" || estimator.second->getName() == "LIOSAM" || estimator.second->getName() == "BRICK" ||
          estimator.second->getName() == "VIO" || estimator.second->getName() == "VSLAM" || estimator.second->getName() == "BRICKFLOW" ||
          estimator.second->getName() == "ICP" || estimator.second->getName() == "ALOAMREP") {

        pos_state << _init_gps_offset_x_, _init_gps_offset_y_;
        estimator.second->setState(0, pos_state);
        estimator.second->setState(1, vel_state);


        // GNSS based estimators (GPS)
      } else {

        double pos_x = odom_pixhawk_shifted_local.pose.pose.position.x;
        double pos_y = odom_pixhawk_shifted_local.pose.pose.position.y;
        pos_state << pos_x, pos_y;
        estimator.second->setState(0, pos_state);
        estimator.second->setState(1, vel_state);
      }

      // RTK estimator

      pos_state(0) = odom_pixhawk_shifted_local.pose.pose.position.x;
      pos_state(1) = odom_pixhawk_shifted_local.pose.pose.position.y;
      {
        std::scoped_lock lock(mutex_rtk_est_);
        sc_lat_rtk_.x = pos_state;
      }
    }

    if (toUppercase(current_lat_estimator_name_) != "GPS" && toUppercase(current_lat_estimator_name_) != "RTK") {
      odom_main.pose.pose.position.x = _init_gps_offset_x_;
      odom_main.pose.pose.position.y = _init_gps_offset_y_;
    }

    // initialize stable odometry
    odom_stable_      = odom_main;
    last_stable_name_ = odom_main.header.frame_id;
    first_frame_      = odom_main.header.frame_id;

    // initialize offset of stable_origin
    odom_stable_pos_offset_.setX(0.0);
    odom_stable_pos_offset_.setY(0.0);
    odom_stable_pos_offset_.setZ(0.0);
    odom_stable_rot_offset_ = mrs_lib::AttitudeConverter(0, 0, 0);

    // initialize local odometry
    if (toUppercase(current_lat_estimator_name_) == "GPS" || toUppercase(current_lat_estimator_name_) == "RTK") {
      odom_local_ = odom_main;
      pos_odom_offset_.setX(odom_main.pose.pose.position.x);
      pos_odom_offset_.setY(odom_main.pose.pose.position.y);
      pos_odom_offset_.setZ(odom_main.pose.pose.position.z);
    } else {
      odom_local_.header = odom_main.header;
      pos_odom_offset_.setX(0.0);
      pos_odom_offset_.setY(0.0);
      pos_odom_offset_.setZ(0.0);
    }
    last_local_name_ = odom_main.header.frame_id;

    {
      std::scoped_lock lock(mutex_altitude_estimator_);

      if (!current_alt_estimator_->getStates(alt_x)) {
        ROS_WARN("[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    /* pos_odom_offset_.setZ(alt_x(mrs_msgs::AltitudeStateNames::HEIGHT)); */

    if (toUppercase(current_lat_estimator_name_) == "GPS" || toUppercase(current_lat_estimator_name_) == "RTK") {
      double hdg_tmp;
      try {
        hdg_tmp = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during obtaining heading (initialization of rot_odom_offset_)");
      }
      rot_odom_offset_ = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(hdg_tmp);
    } else {
      rot_odom_offset_ = mrs_lib::AttitudeConverter(0, 0, 0);
    }

    ROS_INFO("[Odometry]: Initialized the states of all estimators");
  }

  //}

  if (_publish_fused_odom_) {

    // do not publish when switching estimators
    if (is_updating_state_) {
      ROS_INFO("[Odometry]: Not publishing odometry until states finish updating.");
      return;
    }

    /* get altitude states from current filter //{ */

    {
      std::scoped_lock lock(mutex_altitude_estimator_);

      if (!current_alt_estimator_->getStates(alt_x)) {
        ROS_WARN("[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    //}

    /* get lateral states from current filter //{ */

    Vec2 pos_vec, vel_vec, acc_vec;
    {
      std::scoped_lock lock(mutex_current_lat_estimator_);

      current_lat_estimator_->getState(0, pos_vec);
      current_lat_estimator_->getState(1, vel_vec);
      current_lat_estimator_->getState(2, acc_vec);
    }

    odom_main.pose.pose.position.z = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);
    uav_state.pose.position.z      = odom_main.pose.pose.position.z;

    //}

    // Transform global velocity to twist in fcu frame
    geometry_msgs::Vector3Stamped global_vel;
    global_vel.header.frame_id = odom_main.header.frame_id;
    global_vel.header.stamp    = ros::Time::now();
    global_vel.vector.x        = vel_vec(0);
    global_vel.vector.y        = vel_vec(1);
    global_vel.vector.z        = alt_x(mrs_msgs::AltitudeStateNames::VELOCITY);

    geometry_msgs::Vector3Stamped body_vel;
    auto                          response = transformer_.transformSingle(fcu_frame_id_, global_vel);
    if (response) {
      body_vel = response.value();
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed when publishing odom_main.", global_vel.header.frame_id.c_str(),
                        fcu_frame_id_.c_str());
      return;  // TODO how to handle better? With GPS velocity directly from pixhawk can be used, similarly for optflow?
    }

    /* fill in current estimator types //{ */

    uav_state.estimator_horizontal = estimator_type_;
    uav_state.estimator_vertical   = alt_estimator_type_;
    uav_state.estimator_heading    = hdg_estimator_type_;
    uav_state.estimator_iteration  = estimator_iteration_;

    //}

    /* fill in the current heading //{ */

    // get heading states from current filter
    std::string current_hdg_estimator_name_local;
    {
      std::scoped_lock lock(mutex_current_hdg_estimator_);
      current_hdg_estimator_name_local = current_hdg_estimator_->getName();
    }

    if (current_hdg_estimator_name_local != "PIXHAWK") {

      double hdg;

      {
        std::scoped_lock lock(mutex_current_hdg_estimator_);

        current_hdg_estimator_->getState(0, hdg);
      }

      // Obtain mavros orientation
      tf2::Quaternion tf2_mavros_orient = mrs_lib::AttitudeConverter(mavros_orientation);

      // Obtain heading from mavros orientation
      double mavros_hdg = 0;
      try {
        mavros_hdg = mrs_lib::AttitudeConverter(mavros_orientation).getHeading();
      }
      catch (...) {
        ROS_WARN("[Odometry]: failed to getHeading() from mavros_orientation");
      }

      // Build rotation matrix from difference between new heading nad mavros heading
      tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(hdg - mavros_hdg, Eigen::Vector3d::UnitZ()));

      // Transform the mavros orientation by the rotation matrix
      geometry_msgs::Quaternion new_orientation = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_mavros_orient);

      odom_main.pose.pose.orientation = new_orientation;
      uav_state.pose.orientation      = new_orientation;
    }

    //}

    /* fill in the current position //{ */

    if (toUppercase(current_lat_estimator_name_) == "RTK") {
      {
        std::scoped_lock lock(mutex_rtk_est_);

        odom_main.pose.pose.position.x = sc_lat_rtk_.x(0);
        odom_main.pose.pose.position.y = sc_lat_rtk_.x(1);
        uav_state.pose.position.x      = sc_lat_rtk_.x(0);
        uav_state.pose.position.y      = sc_lat_rtk_.x(1);
      }
    } else {
      odom_main.pose.pose.position.x = pos_vec(0);
      odom_main.pose.pose.position.y = pos_vec(1);
      uav_state.pose.position.x      = pos_vec(0);
      uav_state.pose.position.y      = pos_vec(1);
    }

    //}

    /* fill in the velocity //{ */

    odom_main.twist.twist.linear.x = body_vel.vector.x;
    odom_main.twist.twist.linear.y = body_vel.vector.y;
    odom_main.twist.twist.linear.z = body_vel.vector.z;


    uav_state.velocity.linear.x = vel_vec(0);
    uav_state.velocity.linear.y = vel_vec(1);
    uav_state.velocity.linear.z = alt_x(mrs_msgs::AltitudeStateNames::VELOCITY);


    //}

    /* fill in the accelerations //{ */

    uav_state.acceleration.linear.x = acc_vec(0);
    uav_state.acceleration.linear.y = acc_vec(1);
    uav_state.acceleration.linear.z = alt_x(mrs_msgs::AltitudeStateNames::ACCELERATION);

    //}

    /* pass rtk as odometry //{ */

    if (_pass_rtk_as_odom_) {

      auto rtk_local_odom_tmp = mrs_lib::get_mutexed(mutex_rtk_local_odom_, rtk_local_odom_);

      // TODO transform twist to body frame
      odom_main.pose.pose.position    = rtk_local_odom_tmp.pose.pose.position;
      odom_main.pose.pose.orientation = rtk_local_odom_tmp.pose.pose.orientation;
      odom_main.header.frame_id       = _uav_name_ + "/rtk_origin";
      odom_main.child_frame_id        = fcu_frame_id_;

      uav_state.header.frame_id  = _uav_name_ + "/rtk_origin";
      uav_state.pose.position    = rtk_local_odom_tmp.pose.pose.position;
      uav_state.pose.orientation = rtk_local_odom_tmp.pose.pose.orientation;
      uav_state.velocity         = rtk_local_odom_tmp.twist.twist;
    }

    //}

    /* obtain local odom offset //{ */

    if (odom_main.header.frame_id != last_local_name_) {
      ROS_WARN("[Odometry]: Changing odometry estimator from %s to %s. Updating offset for stable odometry.", last_local_name_.c_str(),
               odom_main.header.frame_id.c_str());

      last_local_name_ = odom_main.header.frame_id;
      tf2::Vector3 v1, v2;
      tf2::fromMsg(odom_main.pose.pose.position, v1);
      tf2::fromMsg(odom_local_.pose.pose.position, v2);
      tf2::Vector3 pos_diff = v1 - v2;
      pos_odom_offset_      = pos_diff;

      if (odom_local_.pose.pose.orientation.w == 0.0) {
        ROS_WARN("[Odometry]: Odom stable quaternion x: %f y: %f z: %f w: %f", odom_local_.pose.pose.orientation.x, odom_local_.pose.pose.orientation.y,
                 odom_local_.pose.pose.orientation.z, odom_local_.pose.pose.orientation.w);
        odom_local_.pose.pose.orientation = odom_main.pose.pose.orientation;  // this can cause problems TODO find out why it is happening
      }
      tf2::Quaternion q1       = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation);
      tf2::Quaternion q2       = mrs_lib::AttitudeConverter(odom_local_.pose.pose.orientation);
      tf2::Quaternion rot_diff = q2 * q1.inverse();
      rot_odom_offset_         = rot_diff;
      rot_odom_offset_.normalize();
      /* ROS_WARN("[Odometry]: odometry change stable_q: %f, %f, %f, %f", odom_local_.pose.pose.orientation.x, odom_local_.pose.pose.orientation.y,
       * odom_local_.pose.pose.orientation.z, odom_local_.pose.pose.orientation.w); */
      /* ROS_WARN("[Odometry]: q1: %f, %f, %f, %f,\t q2: %f, %f, %f, %f", q1.x(), q1.y(), q1.z(), q1.w(), q2.x(), q2.y(), q2.z(), q2.w()); */
      ROS_WARN("[Odometry]: pos_diff: x: %f y: %f z: %f", pos_diff.getX(), pos_diff.getY(), pos_diff.getZ());
    }

    /* ROS_WARN("[Odometry]: before stable_q: %f, %f, %f, %f", odom_local_.pose.pose.orientation.x, odom_local_.pose.pose.orientation.y, */
    /*  odom_local_.pose.pose.orientation.z, odom_local_.pose.pose.orientation.w); */
    odom_local_ = applyOdomOffset(odom_main, pos_odom_offset_, rot_odom_offset_);
    /* ROS_WARN("[Odometry]: after stable_q: %f, %f, %f, %f", odom_local_.pose.pose.orientation.x, odom_local_.pose.pose.orientation.y, */
    /* odom_local_.pose.pose.orientation.z, odom_local_.pose.pose.orientation.w); */
    odom_local_.header.frame_id = local_origin_frame_id_;
    /* ROS_INFO_THROTTLE(1.0, "[Odometry]: pos offset: x: %f y: %f z: %f rot offset: x: %f y: %f z: %f w: %f", pos_odom_offset_.getX(),
     * pos_odom_offset_.getY(), pos_odom_offset_.getZ(), rot_odom_offset_.getX(), rot_odom_offset_.getY(), rot_odom_offset_.getZ(),
     * rot_odom_offset_.getW()); */

    //}

    /* publish local odometry //{ */

    try {
      pub_odom_local_.publish(odom_local_);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_local_.getTopic().c_str());
    }

    //}

    /* publish local origin tf //{ */

    // Get inverse trasnform
    tf2::Transform tf_inv        = mrs_uav_odometry::tf2FromPose(odom_local_.pose.pose);
    tf_inv                       = tf_inv.inverse();
    geometry_msgs::Pose pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp          = time_now;
    tf.header.frame_id       = fcu_frame_id_;
    tf.child_frame_id        = local_origin_frame_id_;
    tf.transform.translation = mrs_uav_odometry::pointToVector3(pose_inv.position);
    tf.transform.rotation    = pose_inv.orientation;
    if (noNans(tf)) {
      try {
        broadcaster_->sendTransform(tf);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Indian flatbread detected in transform from %s to %s. Not publishing tf.", tf.header.frame_id.c_str(),
                        tf.child_frame_id.c_str());
    }

    //}

  } else {

    ROS_WARN_THROTTLE(10.0, "[Odometry]: Publishing unfused pixhawk odometry.");
  }

  mrs_lib::set_mutexed(mutex_shared_odometry_, odom_main, shared_odom_);

  /* publish main odom //{ */

  try {
    pub_odom_main_.publish(odom_main);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_main_.getTopic().c_str());
  }

  //}

  /* publish uav state //{ */

  try {
    pub_uav_state_.publish(uav_state);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_uav_state_.getTopic().c_str());
  }
  ROS_INFO_ONCE("[Odometry]: Publishing odometry");

  //}

  /* publish measurement innnovation //{ */

  try {
    pub_odom_main_inno_.publish(odom_main_inno_);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_main_inno_.getTopic().c_str());
  }

  //}

  odometry_published_ = true;

  //}

  /* publish stable odometry //{ */

  nav_msgs::Odometry odom_stable_tmp;
  odom_stable_tmp = odom_main;

  bool got_stable = false;

  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = odom_main.header;
  pose_tmp.pose   = odom_main.pose.pose;
  auto response   = transformer_.transformSingle(first_frame_, pose_tmp);
  if (response) {
    got_stable                = true;
    odom_stable_tmp.pose.pose = response.value().pose;
  } else {
    got_stable = false;
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pose_tmp.header.frame_id.c_str(), first_frame_.c_str());
  }
  odom_stable_tmp.header.frame_id = stable_origin_frame_id_;

  if (got_stable) {
    try {
      pub_odom_stable_.publish(odom_stable_tmp);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_stable_.getTopic().c_str());
    }
  }

  //}

  /* publish stable origin tf //{ */

  if (got_stable) {
    // Get inverse trasnform
    tf2::Transform tf_stable_inv        = mrs_uav_odometry::tf2FromPose(odom_stable_tmp.pose.pose);
    tf_stable_inv                       = tf_stable_inv.inverse();
    geometry_msgs::Pose pose_stable_inv = mrs_uav_odometry::poseFromTf2(tf_stable_inv);

    geometry_msgs::TransformStamped tf_stable;
    tf_stable.header.stamp          = time_now;
    tf_stable.header.frame_id       = fcu_frame_id_;
    tf_stable.child_frame_id        = stable_origin_frame_id_;
    tf_stable.transform.translation = mrs_uav_odometry::pointToVector3(pose_stable_inv.position);
    tf_stable.transform.rotation    = pose_stable_inv.orientation;
    if (noNans(tf_stable)) {
      try {
        broadcaster_->sendTransform(tf_stable);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf_stable.child_frame_id.c_str(), tf_stable.header.frame_id.c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Indian flatbread detected in transform from %s to %s. Not publishing tf.", tf_stable.header.frame_id.c_str(),
                        tf_stable.child_frame_id.c_str());
    }
  }

  //}

  /* publish aloam mapping origin tf //{ */

  float cache_size   = 0.3;
  bool  clear_needed = false;
  if (got_stable && got_aloam_odom_) {
    // Mavros odom chache//{
    {
      std::scoped_lock lock(mutex_vec_odom_mavros_);
      // Add new data
      vec_odom_mavros_.push_back(odom_pixhawk_shifted_local);
      // Delete old data
      size_t index_delete = 0;
      for (size_t i = 0; i < vec_odom_mavros_.size(); i++) {
        if (time_now - vec_odom_mavros_.at(i).header.stamp > ros::Duration(cache_size)) {
          index_delete = i;
          clear_needed = true;
        } else {
          break;
        }
      }
      if (clear_needed) {
        for (int i = (int)index_delete; i >= 0; i--) {
          vec_odom_mavros_.erase(vec_odom_mavros_.begin() + i);
        }
        clear_needed = false;
      }
      /* ROS_INFO_THROTTLE(1.0,"Mavros odom cache size: %d", (int)vec_odom_mavros_.size()); */
    }
    /*//}*/


    // Local odom cache//{
    {
      std::scoped_lock lock(mutex_vec_odom_local_);
      // Add new data
      vec_odom_local_.push_back(odom_local_);
      // Delete old data
      size_t index_delete = 0;
      for (size_t i = 0; i < vec_odom_local_.size(); i++) {
        if (time_now - vec_odom_local_.at(i).header.stamp > ros::Duration(cache_size)) {
          index_delete = i;
          clear_needed = true;
        } else {
          break;
        }
      }
      if (clear_needed) {
        for (int i = (int)index_delete; i >= 0; i--) {
          vec_odom_local_.erase(vec_odom_local_.begin() + i);
        }
      }
      /* ROS_INFO_THROTTLE(1.0,"Local odom cache size: %d", (int)vec_odom_local_.size()); */
    }
    /*//}*/
  }

  if (got_stable && got_aloam_odom_ && aloam_updated_mapping_tf_) {
    std::scoped_lock lock(mutex_vec_odom_mavros_);

    // Copy aloam odometry
    nav_msgs::Odometry aloam_odom_tmp;
    aloam_odom_tmp = aloam_odom_;

    // Find corresponding mavros orientation
    tf2::Quaternion           tf2_mavros_orient;
    geometry_msgs::Quaternion mavros_orientation_temp = mavros_orientation;
    for (int i = 0; i < (int)vec_odom_mavros_.size(); i++) {
      if (aloam_timestamp_ < vec_odom_mavros_.at(i).header.stamp) {
        // Choose mavros orientation with closest timestamp
        float time_diff      = std::fabs(vec_odom_mavros_.at(i).header.stamp.toSec() - aloam_timestamp_.toSec());
        float time_diff_prev = 999999;
        if (i > 0) {
          time_diff_prev = std::fabs(vec_odom_mavros_.at(i - 1).header.stamp.toSec() - aloam_timestamp_.toSec());
        }
        if (time_diff_prev < time_diff && i > 0) {
          i = i - 1;
        }
        // Cache is too small if it is full and its oldest element is used
        if (clear_needed && i == 0) {
          ROS_WARN_THROTTLE(1.0, "[Odometry] Mavros orientation cache is too small.");
        }
        mavros_orientation_temp = vec_odom_mavros_.at(i).pose.pose.orientation;
        tf2_mavros_orient       = mrs_lib::AttitudeConverter(mavros_orientation_temp);
        /* ROS_INFO("aloam_timestamp: %.6f, using mavros stamp: %.6f, curr time: %.6f, timestamp diff: %.6f, delay: %.6f", aloam_timestamp_.toSec(),
         * vec_odom_mavros_.at(i).header.stamp.toSec(), time_now.toSec(),
         * vec_odom_mavros_.at(i).header.stamp.toSec()-aloam_timestamp_.toSec(),time_now.toSec()-aloam_timestamp_.toSec()); */
        /* ROS_INFO_THROTTLE(1.0, "[Odometry] ALOAM delay: %.6f", time_now.toSec() - aloam_timestamp_.toSec()); */
        break;
      }
    }

    // Obtain heading from mavros orientation
    double mavros_hdg = 0;
    try {
      mavros_hdg = mrs_lib::AttitudeConverter(mavros_orientation_temp).getHeading();
    }
    catch (...) {
      ROS_WARN("[Odometry]: failed to getHeading() from mavros_orientation_temp");
    }

    // Build rotation matrix from difference between new heading nad mavros heading
    tf2::Matrix3x3 rot_mat = mrs_lib::AttitudeConverter(Eigen::AngleAxisd(aloam_hdg_previous_ - mavros_hdg, Eigen::Vector3d::UnitZ()));

    // Transform the mavros orientation by the rotation matrix
    geometry_msgs::Quaternion new_orientation = mrs_lib::AttitudeConverter(tf2::Transform(rot_mat) * tf2_mavros_orient);

    // Set new orientation
    aloam_odom_tmp.pose.pose.orientation = new_orientation;

    // Find corresponding local odom
    double local_odom_z = 0;
    for (int i = 0; i < (int)vec_odom_local_.size(); i++) {
      if (aloam_timestamp_ < vec_odom_local_.at(i).header.stamp) {
        // Choose mavros orientation with closest timestamp
        float time_diff      = std::fabs(vec_odom_local_.at(i).header.stamp.toSec() - aloam_timestamp_.toSec());
        float time_diff_prev = 999999;
        if (i > 0) {
          time_diff_prev = std::fabs(vec_odom_local_.at(i - 1).header.stamp.toSec() - aloam_timestamp_.toSec());
        }
        if (time_diff_prev < time_diff && i > 0) {
          i = i - 1;
        }
        // Cache is too small if it is full and its oldest element is used
        if (clear_needed && i == 0) {
          ROS_WARN_THROTTLE(1.0, "[Odometry] Local odom cache (for aloam mapping tf) is too small.");
        }
        local_odom_z = vec_odom_local_.at(i).pose.pose.position.z;
        /* ROS_INFO("aloam_timestamp: %.6f, using local odom stamp: %.6f, curr time: %.6f, timestamp diff: %.6f, delay: %.6f", aloam_timestamp_.toSec(),
         * vec_odom_local_.at(i).header.stamp.toSec(), time_now.toSec(),
         * vec_odom_local_.at(i).header.stamp.toSec()-aloam_timestamp_.toSec(),time_now.toSec()-aloam_timestamp_.toSec()); */
        break;
      }
    }
    // Set z-coordinate from stable odometry
    aloam_odom_tmp.pose.pose.position.z = local_odom_z;

    // Get inverse transform
    tf2::Transform tf_aloam_inv        = mrs_uav_odometry::tf2FromPose(aloam_odom_tmp.pose.pose);
    tf_aloam_inv                       = tf_aloam_inv.inverse();
    geometry_msgs::Pose pose_aloam_inv = mrs_uav_odometry::poseFromTf2(tf_aloam_inv);

    geometry_msgs::TransformStamped tf_mapping;
    tf_mapping.header.stamp          = aloam_timestamp_;
    tf_mapping.header.frame_id       = fcu_frame_id_;
    tf_mapping.child_frame_id        = aloam_mapping_origin_frame_id_;
    tf_mapping.transform.translation = mrs_uav_odometry::pointToVector3(pose_aloam_inv.position);
    tf_mapping.transform.rotation    = pose_aloam_inv.orientation;
    if (noNans(tf_mapping)) {
      try {
        broadcaster_->sendTransform(tf_mapping);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf_mapping.child_frame_id.c_str(), tf_mapping.header.frame_id.c_str());
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Indian flatbread detected in transform from %s to %s. Not publishing tf.", tf_mapping.header.frame_id.c_str(),
                        tf_mapping.child_frame_id.c_str());
    }

    aloam_updated_mapping_tf_ = false;
  }

  //}

  ros::Time t_end          = ros::Time::now();
  double    dur_main_timer = (t_end - t_start).toSec();
  if (dur_main_timer > _hiccup_thr_) {
    int c_hiccup_tmp = mrs_lib::get_mutexed(mutex_c_hiccup_, c_hiccup_);
    c_hiccup_tmp++;
    mrs_lib::set_mutexed(mutex_c_hiccup_, c_hiccup_tmp, c_hiccup_);
    ROS_WARN("[Odometry]: Hiccup detected! mainTimer took: %.6f s.", dur_main_timer);
  }
}

//}

/* //{ slowOdomTimer() */

void Odometry::slowOdomTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("slowOdomTimer", _slow_odom_rate_, 0.01, event);

  nav_msgs::Odometry slow_odom;

  {
    std::scoped_lock lock(mutex_shared_odometry_);

    slow_odom = shared_odom_;
  }

  try {
    pub_slow_odom_.publish(slow_odom);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_slow_odom_.getTopic().c_str());
  }

  mrs_msgs::EspOdometry esp_odom;
  esp_odom.header = slow_odom.header;
  esp_odom.posx   = slow_odom.pose.pose.position.x;
  esp_odom.posy   = slow_odom.pose.pose.position.y;
  esp_odom.posz   = slow_odom.pose.pose.position.z;
  esp_odom.velx   = slow_odom.twist.twist.linear.x;
  esp_odom.vely   = slow_odom.twist.twist.linear.y;
  esp_odom.velz   = slow_odom.twist.twist.linear.z;

  try {
    pub_esp_odom_.publish(esp_odom);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_esp_odom_.getTopic().c_str());
  }
}

//}

/* //{ diagTimer() */

void Odometry::diagTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("diagTimer", _diag_rate_, 0.01, event);

  static ros::Time t_start = ros::Time::now();

  auto c_hiccup_tmp = mrs_lib::get_mutexed(mutex_c_hiccup_, c_hiccup_);
  if (gps_active_) {
    auto gps_cov_tmp = mrs_lib::get_mutexed(mutex_gps_covariance_, gps_covariance_);
    if (c_hiccup_tmp > 0) {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. GPS Cov: %.2f. Max alt: %.2f. Hiccups > %.2f: %d",
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_lat_estimator_name_).c_str(),
                        toUppercase(current_alt_estimator_name_).c_str(), toUppercase(current_hdg_estimator_name_).c_str(), gps_cov_tmp, max_altitude_,
                        _hiccup_thr_, c_hiccup_tmp);
    } else {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. GPS Cov: %.2f. Max alt: %.2f.",
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_lat_estimator_name_).c_str(),
                        toUppercase(current_alt_estimator_name_).c_str(), toUppercase(current_hdg_estimator_name_).c_str(), gps_cov_tmp, max_altitude_);
    }
  } else {
    if (c_hiccup_tmp > 0) {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. Max alt: %.2f. Hiccups > %.2f: %d",
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_lat_estimator_name_).c_str(),
                        toUppercase(current_alt_estimator_name_).c_str(), toUppercase(current_hdg_estimator_name_).c_str(), max_altitude_, _hiccup_thr_,
                        c_hiccup_tmp);
    } else {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. Max alt: %.2f.", (ros::Time::now() - t_start).toSec(),
                        toUppercase(current_lat_estimator_name_).c_str(), toUppercase(current_alt_estimator_name_).c_str(),
                        toUppercase(current_hdg_estimator_name_).c_str(), max_altitude_);
    }
  }

  mrs_msgs::OdometryDiag odometry_diag;

  odometry_diag.header.stamp = ros::Time::now();

  odometry_diag.estimator_type = estimator_type_;
  odometry_diag.altitude_type  = alt_estimator_type_;
  odometry_diag.heading_type   = hdg_estimator_type_;

  odometry_diag.available_lat_estimators = _active_state_estimators_names_;
  odometry_diag.available_hdg_estimators = _active_heading_estimators_names_;


  std::vector<std::string> active_alt_estimators;
  if (stringInVector("HEIGHT", _altitude_estimators_names_) && height_active_) {
    active_alt_estimators.push_back("HEIGHT");
  }
  if (stringInVector("PLANE", _altitude_estimators_names_) && plane_reliable_) {
    active_alt_estimators.push_back("PLANE");
  }
  if (stringInVector("BRICK", _altitude_estimators_names_) && brick_active_) {
    active_alt_estimators.push_back("BRICK");
  }
  if (stringInVector("VIO", _altitude_estimators_names_) && vio_active_) {
    active_alt_estimators.push_back("VIO");
  }
  if (stringInVector("ALOAM", _altitude_estimators_names_) && aloam_active_) {
    active_alt_estimators.push_back("ALOAM");
  }
  if (stringInVector("ALOAMGARM", _altitude_estimators_names_) && aloam_active_) {
    active_alt_estimators.push_back("ALOAMGARM");
  }
  if (stringInVector("ALOAMREP", _altitude_estimators_names_) && aloam_active_) {
    active_alt_estimators.push_back("ALOAMREP");
  }
  if (stringInVector("BARO", _altitude_estimators_names_)) {
    active_alt_estimators.push_back("BARO");
  }
  if (stringInVector("RTK", _altitude_estimators_names_)) {
    active_alt_estimators.push_back("RTK");
  }
  if (stringInVector("LIOSAM", _altitude_estimators_names_) && liosam_active_) {
    active_alt_estimators.push_back("LIOSAM");
  }
  odometry_diag.available_alt_estimators = active_alt_estimators;

  odometry_diag.max_altitude = mrs_lib::get_mutexed(mutex_max_altitude_, max_altitude_);

  try {
    pub_odometry_diag_.publish(odometry_diag);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odometry_diag_.getTopic().c_str());
  }

  if (_debug_publish_servoing_) {
    mrs_msgs::ReferenceStamped servoing_diag_out;

    servoing_diag_out.header.stamp    = ros::Time::now();
    servoing_diag_out.header.frame_id = "visual_servoing_debug";

    servoing_diag_out.reference.position.x = c_failed_brick_x_;
    servoing_diag_out.reference.position.y = c_failed_brick_y_;
    servoing_diag_out.reference.position.z = c_failed_brick_timeout_;
    servoing_diag_out.reference.heading    = c_failed_brick_hdg_;

    try {
      pub_brick_diag_.publish(servoing_diag_out);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_brick_diag_.getTopic().c_str());
    }
  }
}

//}

/* //{ lkfStatesTimer() */

void Odometry::lkfStatesTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("lkfStatesTimer", _lkf_states_rate_, 0.01, event);

  LatState2D states_mat;

  // get states and covariances from lateral kalman
  {
    std::scoped_lock lock(mutex_current_lat_estimator_);

    current_lat_estimator_->getStates(states_mat);
  }

  /* // fill the message */
  mrs_msgs::LkfStates lkf_states_x;
  mrs_msgs::LkfStates lkf_states_y;

  lkf_states_x.header.stamp = ros::Time::now();
  lkf_states_x.pos          = states_mat(0, 0);
  lkf_states_x.vel          = states_mat(1, 0);
  lkf_states_x.acc          = states_mat(2, 0);

  lkf_states_y.header.stamp = ros::Time::now();
  lkf_states_y.pos          = states_mat(0, 1);
  lkf_states_y.vel          = states_mat(1, 1);
  lkf_states_y.acc          = states_mat(2, 1);

  try {
    pub_lkf_states_x_.publish(lkf_states_x);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_lkf_states_x_.getTopic().c_str());
  }

  try {
    pub_lkf_states_y_.publish(lkf_states_y);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_lkf_states_y_.getTopic().c_str());
  }

  hdg_x_t hdg_state;
  hdg_P_t hdg_covariance;

  mrs_msgs::EstimatedState hdg_state_msg;
  hdg_state_msg.header.stamp = ros::Time::now();

  if (current_hdg_estimator_->getName() == "PIXHAWK") {

    double             hdg, hdg_rate;
    nav_msgs::Odometry odom_pixhawk_tmp = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_);

    try {
      hdg = mrs_lib::AttitudeConverter(odom_pixhawk_tmp.pose.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading (odom_pixhawk_ orientation)");
    }
    hdg_state_msg.state.push_back(hdg);

    try {
      hdg_rate = mrs_lib::AttitudeConverter(odom_pixhawk_tmp.pose.pose.orientation).getHeadingRate(odom_pixhawk_tmp.twist.twist.angular);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading rate (odom_pixhawk_ angular twist)");
    }
    hdg_state_msg.state.push_back(hdg_rate);

  } else {
    {
      std::scoped_lock lock(mutex_current_hdg_estimator_);
      current_hdg_estimator_->getStates(hdg_state);
      current_hdg_estimator_->getCovariance(hdg_covariance);
    }
    hdg_state(0, 0) = radians::wrap(hdg_state(0, 0));
    for (int i = 0; i < hdg_state.size(); i++) {
      hdg_state_msg.state.push_back(hdg_state(i, 0));
      hdg_state_msg.covariance.push_back(hdg_covariance(i, i));
    }
  }

  try {
    pub_heading_states_.publish(hdg_state_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_heading_states_.getTopic().c_str());
  }
}

//}

/* //{ maxAltitudeTimer() */

void Odometry::maxAltitudeTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("maxAltitudeTimer", _max_altitude_rate_, 0.01, event);

  mrs_msgs::Float64Stamped max_altitude_m_sg;
  max_altitude_m_sg.header.frame_id = _uav_name_ + "/" + toLowercase(current_lat_estimator_name_) + "_origin";
  max_altitude_m_sg.header.stamp    = ros::Time::now();

  max_altitude_m_sg.value = mrs_lib::get_mutexed(mutex_max_altitude_, max_altitude_);

  try {
    pub_max_altitude_.publish(max_altitude_m_sg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_max_altitude_.getTopic().c_str());
  }
}

//}

/* //{ rtkRateTimer() */

void Odometry::rtkRateTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("rtkRateTimer", 1, 0.01, event);

  if (got_rtk_) {

    if (got_rtk_counter_ < 15) {

      ROS_ERROR_THROTTLE(1.0, "[Odometry]: RTK comming at slow rate (%d Hz)!", got_rtk_counter_);
    }

    got_rtk_counter_ = 0;
  }
}

//}

/* //{ topicWatcherTimer() */

void Odometry::topicWatcherTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("topicWatcherTimer", topic_watcher_rate_, 0.01, event);

  ros::Duration interval;

  // garmin range
  interval = ros::Time::now() - garmin_last_update_;
  if (height_active_ && interval.toSec() > 0.5) {
    ROS_WARN("[Odometry]: Garmin range not received for %f seconds.", interval.toSec());
    height_active_ = false;
  }

  // pixhawk odometry
  interval = ros::Time::now() - odom_pixhawk_last_update_;
  if (got_odom_pixhawk_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Pixhawk odometry not received for %f seconds.", interval.toSec());
    got_odom_pixhawk_ = false;
  }

  // optflow velocities (corrections of lateral kf)
  interval = ros::Time::now() - optflow_twist_last_update_;
  if (got_optflow_ && interval.toSec() > 0.1) {
    ROS_WARN("[Odometry]: Optflow twist not received for %f seconds.", interval.toSec());
    if (got_optflow_ && interval.toSec() > 1.0) {
      got_optflow_      = false;
      optflow_reliable_ = false;
    }
  }

  // hector pose (corrections of lateral kf)
  interval = ros::Time::now() - hector_pose_last_update_;
  if (got_hector_pose_ && interval.toSec() > 0.1) {
    ROS_WARN("[Odometry]: Hector pose not received for %f seconds.", interval.toSec());
    if (got_hector_pose_ && interval.toSec() > 0.5) {
      got_hector_pose_ = false;
      hector_reliable_ = false;
    }
  }

  //  target attitude (input to lateral kf)
  interval = ros::Time::now() - attitude_command_last_update_;
  if (got_attitude_command_ && interval.toSec() > 0.1) {
    ROS_WARN("[Odometry]: Attitude command not received for %f seconds.", interval.toSec());
    if (got_attitude_command_ && interval.toSec() > 1.0) {
      got_attitude_command_ = false;
    }
  }

  // control acceleration (input to altitude kf)
  interval = ros::Time::now() - control_accel_last_update_;
  if (got_control_accel_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Control acceleration not received for %f seconds.", interval.toSec());
    got_control_accel_ = false;
  }

  // IMU data (corrections to altitude, lateral and heading kf)
  interval = ros::Time::now() - pixhawk_imu_last_update_;
  if (got_pixhawk_imu_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: IMU data not received for %f seconds.", interval.toSec());
    got_pixhawk_imu_ = false;
  }

  //  vio odometry (corrections of lateral kf)
  interval = ros::Time::now() - odom_vio_last_update_;
  if (got_vio_ && interval.toSec() > 0.5) {
    ROS_WARN("[Odometry]: VIO odometry not received for %f seconds.", interval.toSec());
    got_vio_ = false;
  }

  //  vslam pose (corrections of lateral kf)
  interval = ros::Time::now() - pose_vslam_last_update_;
  if (got_vslam_ && interval.toSec() > 0.5) {
    ROS_WARN("[Odometry]: VSLAM odometry not received for %f seconds.", interval.toSec());
    got_vslam_ = false;
  }

  //  brick odometry (corrections of lateral kf)
  interval = ros::Time::now() - brick_pose_last_update_;
  if (got_brick_pose_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: BRICK odometry not received for %f seconds.", interval.toSec());
    got_brick_pose_ = false;
    brick_reliable_ = false;
  }

  //  icp twist global
  interval = ros::Time::now() - icp_twist_last_update_;
  if (got_icp_twist_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP velocities not received for %f seconds.", interval.toSec());
    got_icp_twist_ = false;
  }

  //  aloam odometry (corrections of lateral kf)
  interval = ros::Time::now() - aloam_odom_last_update_;
  if (got_aloam_odom_ && interval.toSec() > 0.5) {
    ROS_WARN("[Odometry]: ALOAM odometry not received for %f seconds.", interval.toSec());
    got_aloam_odom_ = false;
  }

  //  LIOSAM odometry (corrections of lateral kf)
  interval = ros::Time::now() - liosam_odom_last_update_;
  if (got_liosam_odom_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: LIOSAM odometry not received for %f seconds. Current time: %0.2f, last msg: %0.2f.", interval.toSec(), ros::Time::now().toSec(),
             liosam_odom_last_update_.toSec());
    got_liosam_odom_ = false;
  }
}

//}

/* //{ callbackTimerHectorResetRoutine() */

void Odometry::callbackTimerHectorResetRoutine(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  hector_reset_routine_running_ = true;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackTimerHectorResetRoutine", topic_watcher_rate_, 0.01, event);

  // Change estimator to ICP
  bool in_icp = false;
  if (icp_active_ && got_icp_twist_) {
    ROS_WARN("[Odometry]: HECTOR not reliable. Switching to ICP type.");
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::ICP;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    mrs_msgs::EstimatorType icp_type;
    icp_type.type = mrs_msgs::EstimatorType::ICP;
    if (!changeCurrentHeadingEstimator(desired_estimator) || !changeCurrentEstimator(icp_type)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
      std_srvs::Trigger failsafe_out;
      ser_client_failsafe_.call(failsafe_out);
      failsafe_called_ = true;
      return;
    } else {
      in_icp = true;
    }
  }

  // Change estimator to optflow
  if (optflow_active_ && got_optflow_ && !in_icp) {
    ROS_WARN("[Odometry]: HECTOR not reliable. Switching to OPTFLOW type.");
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    mrs_msgs::EstimatorType optflow_type;
    optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
    if (!changeCurrentHeadingEstimator(desired_estimator) || !changeCurrentEstimator(optflow_type)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
      std_srvs::Trigger failsafe_out;
      ser_client_failsafe_.call(failsafe_out);
      failsafe_called_ = true;
      return;
    }
  }

  // Disable odometry service callbacks
  callbacks_enabled_ = false;

  // Call hover service
  callHover();

  // Call MpcController service
  callMpcController();

  // Disable control callbacks
  callDisableControlCallbacks();

  // Reset HECTOR map
  ROS_INFO("[Odometry]: Calling Hector map reset.");
  std_msgs::String reset_msg;
  reset_msg.data = "reset";
  try {
    pub_hector_reset_.publish(reset_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_hector_reset_.getTopic().c_str());
  }
  hector_reset_called_ = true;
  ROS_INFO("[Odometry]: Hector map reset called.");

  // Reset HECTOR heading
  for (auto &estimator : heading_estimators_) {
    if (estimator.first == "HECTOR") {
      estimator.second->setState(0, 0);
      estimator.second->setState(1, 0);
      estimator.second->setState(2, 0);
    }
  }

  // Reset HECTOR position
  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "HECTOR") {
      Vec2 pos_vec, vel_vec, acc_vec;
      pos_vec << 0, 0;
      vel_vec << 0, 0;
      acc_vec << 0, 0;
      estimator.second->setState(0, pos_vec);
      estimator.second->setState(1, vel_vec);
      estimator.second->setState(2, acc_vec);
    }
  }

  // Let the estimator converge
  int wait_msgs = 60;
  ROS_INFO("[Odometry]: Waiting for %d HECTOR msgs after map reset.", wait_msgs);
  c_hector_msg_ = 0;
  while (c_hector_msg_ < wait_msgs) {
    ros::Duration(0.1).sleep();
  }

  // Switch back to HECTOR
  ROS_INFO("[Odometry]: %d HECTOR msgs after map reset arrived. Switching to HECTOR type", wait_msgs);
  mrs_msgs::HeadingType desired_estimator;
  desired_estimator.type = mrs_msgs::HeadingType::HECTOR;
  desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
  changeCurrentHeadingEstimator(desired_estimator);
  hector_reliable_ = true;
  mrs_msgs::EstimatorType hector_type;
  hector_type.type = mrs_msgs::EstimatorType::HECTOR;
  if (!changeCurrentEstimator(hector_type)) {
    ROS_WARN("[Odometry]: Switching back to HECTOR type after map reset failed.");
  }

  // Enable callbacks
  callEnableControlCallbacks();

  // Prepare timer for next run
  hector_reset_routine_running_ = false;
  hector_reset_routine_timer_.stop();
  hector_reset_routine_timer_.setPeriod(ros::Duration(0.00001));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | ------------------ subscriber callbacks ------------------ |

/* //{ callbackAttitudeCommand() */
void Odometry::callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackAttitudeCommand");

  {
    std::scoped_lock lock(mutex_attitude_command_);

    if (got_attitude_command_) {

      attitude_command_prev_ = attitude_command_;
      attitude_command_      = *msg;

      attitude_command_last_update_ = ros::Time::now();

    } else {

      attitude_command_      = *msg;
      attitude_command_prev_ = attitude_command_;

      attitude_command_last_update_ = ros::Time::now();
      got_attitude_command_         = true;
      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------


  {
    std::scoped_lock lock(mutex_attitude_command_);
    if (!isTimestampOK(attitude_command_.header.stamp.toSec(), attitude_command_prev_.header.stamp.toSec())) {
      ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Target attitude timestamp not OK, skipping prediction of lateral estimators.");
      return;
    }
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  double dt = (attitude_command_.header.stamp - attitude_command_prev_.header.stamp).toSec();

  if (!std::isfinite(dt)) {
    dt = 0;
    ROS_INFO_THROTTLE(1.0, "[Odometry]: NaN detected in attitude cmd variable \"dt\", setting it to 0 and returning!!!");
    return;
  } else if (dt > 1) {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Attitude cmd variable \"dt\" > 1, setting it to 1 and returning!!!");
    dt = 1;
    return;
  } else if (dt < 0) {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Attitude cmd variable \"dt\" < 0, setting it to 0 and returning!!!");
    dt = 0;
    return;
  }
  auto attitude_command = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_);

  /* attitude command //{ */

  double des_hdg;

  try {
    des_hdg = mrs_lib::AttitudeConverter(attitude_command.attitude).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading (attitude command)");
    des_hdg = 0.0;
  }

  des_hdg_ = des_hdg;

  if (_debug_publish_corrections_) {
    mrs_msgs::Float64Stamped des_hdg_msg;
    des_hdg_msg.header = attitude_command.header;
    des_hdg_msg.value  = des_hdg;

    try {
      pub_cmd_hdg_input_.publish(des_hdg_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_cmd_hdg_input_.getTopic().c_str());
    }
  }

  //}

  /* attitude rate command //{ */

  geometry_msgs::Vector3 attitude_rate;
  attitude_rate.x = attitude_command.attitude_rate.x;
  attitude_rate.y = attitude_command.attitude_rate.y;
  attitude_rate.z = attitude_command.attitude_rate.z;

  double des_hdg_rate;
  try {
    des_hdg_rate = mrs_lib::AttitudeConverter(attitude_command.attitude).getHeadingRate(attitude_rate);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading rate (attitude command)");
    des_hdg_rate = 0.0;
  }


  if (!std::isfinite(des_hdg_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"des_hdg_rate_\", prediction with zero input!!!");
    des_hdg_rate = 0.0;
  }

  if (!isUavFlying()) {
    des_hdg_rate = 0.0;
  }

  des_hdg_rate_ = des_hdg_rate;

  if (_debug_publish_corrections_) {
    mrs_msgs::Float64Stamped des_hdg_rate_msg;
    des_hdg_rate_msg.header = attitude_command.header;
    des_hdg_rate_msg.value  = des_hdg_rate;

    try {
      pub_cmd_hdg_rate_input_.publish(des_hdg_rate_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_cmd_hdg_rate_input_.getTopic().c_str());
    }
  }

  //}

  /* altitude acceleration command //{ */

  {
    if (got_odom_pixhawk_) {

      std::scoped_lock lock(mutex_odom_pixhawk_);

      nav_msgs::Odometry odom_pixhawk_local = odom_pixhawk_;

      Eigen::Matrix3d uav_orientation = mrs_lib::AttitudeConverter(odom_pixhawk_.pose.pose.orientation);

      Eigen::Vector3d desired_acceleration_fcu;
      desired_acceleration_fcu[0] = attitude_command.desired_acceleration.x;
      desired_acceleration_fcu[1] = attitude_command.desired_acceleration.y;
      desired_acceleration_fcu[2] = attitude_command.desired_acceleration.z;

      Eigen::Vector3d desired_acceleration_fcu_untilted = uav_orientation * desired_acceleration_fcu;

      mrs_lib::set_mutexed(mutex_alt_input_, desired_acceleration_fcu_untilted[2], alt_input_);
      mrs_lib::set_mutexed(mutex_alt_input_, attitude_command.header.stamp, alt_input_stamp_);
    }
  }

  //}
}

//}

/* //{ callbackMavrosOdometry() */
void Odometry::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {


  if (!is_initialized_)
    return;

  if (is_updating_state_) {
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOdometry");

  if (got_odom_pixhawk_) {

    {
      std::scoped_lock lock(mutex_odom_pixhawk_);

      odom_pixhawk_previous_         = odom_pixhawk_;
      odom_pixhawk_previous_shifted_ = odom_pixhawk_shifted_;
      odom_pixhawk_                  = *msg;
      odom_pixhawk_shifted_          = *msg;

      // shift the odom_pixhawk_shifted_
      if (got_pixhawk_odom_offset_) {
        odom_pixhawk_shifted_.pose.pose.position.x += pixhawk_odom_offset_x_;
        odom_pixhawk_shifted_.pose.pose.position.y += pixhawk_odom_offset_y_;
        got_pixhawk_odom_shifted_ = true;
      }
    }

  } else {

    {
      std::scoped_lock lock(mutex_odom_pixhawk_);

      odom_pixhawk_previous_ = *msg;
      odom_pixhawk_          = *msg;

      // store the initial magnetic heading (corresponding to 0 of non-magnetic heading estimators)
      try {
        init_magnetic_heading_ = mrs_lib::AttitudeConverter(odom_pixhawk_.pose.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during getting heading (init_magnetic_heading)");
      }
    }

    if (_simulation_) {
      mavros_glitch_.x = 0.0;
      mavros_glitch_.y = 0.0;
      mavros_glitch_.z = 0.0;
    }

    got_odom_pixhawk_         = true;
    gps_reliable_             = true;
    odom_pixhawk_last_update_ = ros::Time::now();
    ROS_INFO("[Odometry]: Received the first pixhawk odom message");
    return;
  }

  odom_pixhawk_last_update_ = ros::Time::now();

  if (!isTimestampOK(odom_pixhawk_.header.stamp.toSec(), odom_pixhawk_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk odom timestamp not OK, not fusing correction.");
    return;
  }

  // Publish transform of fcu_untilted frame
  tf2::Quaternion q;
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);

    double heading;

    try {
      heading = mrs_lib::AttitudeConverter(odom_pixhawk_.pose.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading");
      return;
    }

    // we need to undo the heading

    Eigen::Matrix3d odom_pixhawk_R = mrs_lib::AttitudeConverter(odom_pixhawk_.pose.pose.orientation);
    Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

    q = mrs_lib::AttitudeConverter(undo_heading_R * odom_pixhawk_R);
  }
  q = q.inverse();

  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();  // TODO test whether odom_pixhawk_.header.stamp is more stable
  tf.header.frame_id         = fcu_frame_id_;
  tf.child_frame_id          = fcu_untilted_frame_id_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(q);
  if (noNans(tf)) {
    try {
      broadcaster_->sendTransform(tf);
      got_fcu_untilted_ = true;
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
    }
  }
  double dt;
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);

    dt = (odom_pixhawk_.header.stamp - odom_pixhawk_previous_.header.stamp).toSec();
  }

  // Negate weird simulation position jump glitches (caused by gazebo timestamp glitches - should be handled by isTimestampOK(), this check is for redundancy)
  // (smaller jumps in GPS position are handled by safety control mechanisms)
  if (_simulation_) {

    if (std::fabs(odom_pixhawk_.pose.pose.position.x - odom_pixhawk_previous_.pose.pose.position.x) > 100) {
      mavros_glitch_.x = odom_pixhawk_.pose.pose.position.x - odom_pixhawk_previous_.pose.pose.position.x;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current x: %f, Previous x: %f", odom_pixhawk_.pose.pose.position.x,
               odom_pixhawk_previous_.pose.pose.position.x);
    }
    if (std::fabs(odom_pixhawk_.pose.pose.position.y - odom_pixhawk_previous_.pose.pose.position.y) > 100) {
      mavros_glitch_.y = odom_pixhawk_.pose.pose.position.y - odom_pixhawk_previous_.pose.pose.position.y;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current y: %f, Previous y: %f", odom_pixhawk_.pose.pose.position.y,
               odom_pixhawk_previous_.pose.pose.position.y);
    }
    if (std::fabs(odom_pixhawk_.pose.pose.position.z - odom_pixhawk_previous_.pose.pose.position.z) > 100) {
      mavros_glitch_.z = odom_pixhawk_.pose.pose.position.z - odom_pixhawk_previous_.pose.pose.position.z;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current z: %f, Previous z: %f", odom_pixhawk_.pose.pose.position.z,
               odom_pixhawk_previous_.pose.pose.position.z);
    }

    odom_pixhawk_shifted_.pose.pose.position.x -= mavros_glitch_.x;
    odom_pixhawk_shifted_.pose.pose.position.y -= mavros_glitch_.y;
    odom_pixhawk_shifted_.pose.pose.position.z -= mavros_glitch_.z;
  }

  if (!got_init_heading_) {

    double hdg        = getCurrentHeading();
    init_heading_     = hdg;
    got_init_heading_ = true;
  }

  if (!got_range_ && garmin_enabled_) {

    return;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------


  // Transform twist from body frame to ENU frame
  tf2::Transform         tf_pixhawk(mrs_lib::AttitudeConverter(odom_pixhawk_.pose.pose.orientation));
  tf2::Vector3           vel_body(odom_pixhawk_.twist.twist.linear.x, odom_pixhawk_.twist.twist.linear.y, odom_pixhawk_.twist.twist.linear.z);
  tf2::Vector3           vel_enu_tmp = tf_pixhawk * vel_body;
  geometry_msgs::Vector3 vel_enu;
  vel_enu.x = vel_enu_tmp.getX();
  vel_enu.y = vel_enu_tmp.getY();
  vel_enu.z = vel_enu_tmp.getZ();

  /* altitude estimator update //{ */

  /* publish covariance of altitude states //{ */

  alt_P_t P = P.Zero();
  {
    std::scoped_lock lock(mutex_altitude_estimator_);
    current_alt_estimator_->getCovariance(P);
  }
  mrs_msgs::Float64ArrayStamped cov_msg;
  cov_msg.header.stamp    = ros::Time::now();
  cov_msg.header.frame_id = local_origin_frame_id_;
  for (int i = 0; i < P.rows(); i++) {
    cov_msg.values.push_back(P(i, i));
  }
  try {
    pub_alt_cov_.publish(cov_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_alt_cov_.getTopic().c_str());
  }

  //}

  /* do correction of barometer altitude //{ */

  // fusing measurements as velocity allows compensating barometer offset with garmin measurements
  alt_x_t alt_x = alt_x.Zero();

  if (!current_alt_estimator_->getStates(alt_x)) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
    return;
  }

  double    altitude;
  double    twist_z;
  ros::Time meas_stamp;
  ros::Time time_now = ros::Time::now();
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);
    altitude   = odom_pixhawk_.pose.pose.position.z - baro_offset_;
    twist_z    = vel_enu.z;
    meas_stamp = odom_pixhawk_.header.stamp;
  }

  // fuse zero into baro estimator when on the ground
  auto range_garmin_tmp = mrs_lib::get_mutexed(mutex_range_garmin_, range_garmin_);
  if (!isUavFlying()) {
    altitude        = 0.0;
    baro_corrected_ = false;
  } else if (!baro_corrected_ && std::isfinite(range_garmin_tmp.range)) {

    auto odom_pixhawk_tmp = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_);
    baro_offset_          = odom_pixhawk_tmp.pose.pose.position.z - range_garmin_tmp.range;
    baro_corrected_       = true;
  }


  {
    std::scoped_lock lock(mutex_altitude_estimator_);
    altitudeEstimatorCorrection(twist_z, "vel_baro", meas_stamp, time_now);
    altitudeEstimatorCorrection(altitude, "height_baro", meas_stamp, time_now);
  }

  if (_debug_publish_corrections_) {

    mrs_msgs::Float64Stamped vel_baro_corr_out;
    vel_baro_corr_out.header.stamp    = ros::Time::now();
    vel_baro_corr_out.header.frame_id = local_origin_frame_id_;
    vel_baro_corr_out.value           = twist_z;
    try {
      pub_vel_baro_corr_.publish(vel_baro_corr_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_vel_baro_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing barometer altitude");

  //}

  //}

  /* state estimators update //{ */

  if (!got_pixhawk_odom_offset_) {
    return;
  }

  /* mavros tilts in inertial frame //{ */

  geometry_msgs::Quaternion orient;

  {
    std::scoped_lock lock(mutex_odom_pixhawk_shifted_);

    orient                     = odom_pixhawk_shifted_.pose.pose.orientation;
    orientation_mavros_.header = odom_pixhawk_shifted_.header;
  }

  [[maybe_unused]] double hdg = getCurrentHeading();  // TODO unused?

  auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(orient);

  // publish orientation for debugging
  orientation_mavros_.vector.x = roll;
  orientation_mavros_.vector.y = pitch;
  orientation_mavros_.vector.z = yaw;
  try {
    pub_orientation_mavros_.publish(orientation_mavros_);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_orientation_mavros_.getTopic().c_str());
  }

  //}

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros odom. Waiting for other sensors.");
    return;
  }

  /* //{ fuse mavros velocity */

  if (gps_active_) {

    double vel_mavros_x, vel_mavros_y;
    double diff_mavros_x, diff_mavros_y;
    {
      std::scoped_lock lock(mutex_odom_pixhawk_);

      diff_mavros_x = (odom_pixhawk_shifted_.pose.pose.position.x - odom_pixhawk_previous_shifted_.pose.pose.position.x) / dt;
      diff_mavros_y = (odom_pixhawk_shifted_.pose.pose.position.y - odom_pixhawk_previous_shifted_.pose.pose.position.y) / dt;
      vel_mavros_x  = odom_pixhawk_shifted_.twist.twist.linear.x;
      vel_mavros_y  = odom_pixhawk_shifted_.twist.twist.linear.y;
    }

    // Apply correction step to all state estimators
    if (fabs(vel_mavros_x) < 100 && fabs(vel_mavros_y) < 100) {
      stateEstimatorsCorrection(vel_mavros_x, vel_mavros_y, "vel_mavros");
      ROS_INFO_ONCE("[Odometry]: Fusing mavros velocity");
    }

    // Set innoation variable if ccurnet estimator is GPS
    if (current_lat_estimator_->getName() == "GPS") {
      Vec2 vel_vec, innovation;
      current_lat_estimator_->getState(1, vel_vec);

      innovation(0) = vel_mavros_x - vel_vec(0);
      innovation(1) = vel_mavros_y - vel_vec(1);
      {
        std::scoped_lock lock(mutex_odom_main_inno_);
        odom_main_inno_.twist.twist.linear.x = innovation(0);
        odom_main_inno_.twist.twist.linear.y = innovation(1);
      }
    }

    if (rtk_active_) {
      Eigen::VectorXd rtk_input(2);
      rtk_input << diff_mavros_x, diff_mavros_y;

      // RTK estimator prediction step
      {
        std::scoped_lock lock(mutex_rtk_est_);

        lkf_rtk_t::B_t B_new;
        B_new << dt, 0, 0, dt;
        estimator_rtk_->B = B_new;
        try {
          sc_lat_rtk_ = estimator_rtk_->predict(sc_lat_rtk_, rtk_input, _Q_lat_rtk_, dt);
        }
        catch (const std::exception &e) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: RTK LKF prediction step failed: %s", e.what());
        }
      }
    }
  }

  //}

  /* //{ fuse mavros position */

  if (gps_active_) {

    if (!got_odom_pixhawk_ || !got_pixhawk_utm_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Mavros position. Global position not averaged.");
      return;
    }

    double             pos_mavros_x, pos_mavros_y;
    nav_msgs::Odometry odom_mavros_out;
    {
      std::scoped_lock lock(mutex_odom_pixhawk_shifted_);

      odom_mavros_out = odom_pixhawk_shifted_;
      pos_mavros_x    = odom_pixhawk_shifted_.pose.pose.position.x;
      pos_mavros_y    = odom_pixhawk_shifted_.pose.pose.position.y;
    }

    if (!lat_gps_est_pos_init_) {

      // Set estimator to correct position before first correction
      Vec2 zero_state = zero_state.Zero();
      Vec2 pos;
      pos << pos_mavros_x, pos_mavros_y;
      ROS_INFO("[Odometry]: Initializing GPS and RTK estimators to x: %f y: %f", pos(0), pos(1));
      for (auto &estimator : _lateral_estimators_) {
        if (estimator.first == "GPS" || estimator.first == "RTK") {

          estimator.second->setState(0, pos);
          estimator.second->setState(1, zero_state);
          estimator.second->setState(2, zero_state);

          Vec2 pos_init;
          estimator.second->getState(0, pos_init);
          ROS_INFO("[Odometry]: Initialized %s estimator to x: %f y: %f", estimator.first.c_str(), pos_init(0), pos_init(1));
        }
      }
      lat_gps_est_pos_init_ = true;
    }

    try {
      pub_odom_mavros_.publish(odom_mavros_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_odom_mavros_.getTopic().c_str());
    }

    // Saturate correction
    for (auto &estimator : _lateral_estimators_) {
      if (estimator.first == "GPS") {
        Vec2 pos_vec, innovation;
        estimator.second->getState(0, pos_vec);

        innovation(0) = pos_mavros_x - pos_vec(0);
        innovation(1) = pos_mavros_y - pos_vec(1);

        if (_saturate_mavros_position_) {

          // X position
          if (!std::isfinite(pos_mavros_x)) {
            pos_mavros_x = 0;
            ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"pos_mavros_x\", setting it to 0 and returning!!!");
            return;
          } else if (innovation(0) > _max_mavros_pos_correction_) {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating GPS X pos correction %f -> %f", innovation(0), _max_mavros_pos_correction_);
            pos_mavros_x = pos_vec(0) + _max_mavros_pos_correction_;
          } else if (innovation(0) < -_max_mavros_pos_correction_) {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating GPS X pos correction %f -> %f", innovation(0), -_max_mavros_pos_correction_);
            pos_mavros_x = pos_vec(0) - _max_mavros_pos_correction_;
          }

          // Y position
          if (!std::isfinite(pos_mavros_y)) {
            pos_mavros_y = 0;
            ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"pos_mavros_y\", setting it to 0 and returning!!!");
            return;
          } else if (innovation(1) > _max_mavros_pos_correction_) {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating GPS Y pos correction %f -> %f", innovation(1), _max_mavros_pos_correction_);
            pos_mavros_y = pos_vec(1) + _max_mavros_pos_correction_;
          } else if (innovation(1) < -_max_mavros_pos_correction_) {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating GPS Y pos correction %f -> %f", innovation(1), -_max_mavros_pos_correction_);
            pos_mavros_y = pos_vec(1) - _max_mavros_pos_correction_;
          }
        }

        // Set innoation variable if ccurnet estimator is GPS
        if (current_lat_estimator_->getName() == "GPS") {
          odom_main_inno_.pose.pose.position.x = innovation(0);
          odom_main_inno_.pose.pose.position.y = innovation(1);
          odom_main_inno_.pose.pose.position.z = 0;
        }
      }
    }

    if (finished_state_update_) {
      ROS_INFO("[Odometry]: finished state update");
      for (auto &estimator : _lateral_estimators_) {
        if (estimator.first == "GPS") {
          LatState2D state;
          estimator.second->getStates(state);
          ROS_INFO_STREAM("[Odometry]: state after rotation:" << state);
          ROS_INFO("[Odometry]: mavros position correction after state rotation: x: %2.2f y: %2.2f", pos_mavros_x, pos_mavros_y);
        }
      }
      finished_state_update_ = false;
    }

    // Apply correction step to all state estimators
    stateEstimatorsCorrection(pos_mavros_x, pos_mavros_y, "pos_mavros");

    ROS_INFO_ONCE("[Odometry]: Fusing mavros position");
    //}

    //}
  }
}

//}

/* //{ callbackPixhawkImu() */

void Odometry::callbackPixhawkImu(const sensor_msgs::ImuConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPixhawkImu");

  pixhawk_imu_last_update_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_pixhawk_imu_);

    if (got_pixhawk_imu_) {

      pixhawk_imu_previous_ = pixhawk_imu_;
      pixhawk_imu_          = *msg;

    } else {

      pixhawk_imu_previous_ = *msg;
      pixhawk_imu_          = *msg;
      got_pixhawk_imu_      = true;

      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(pixhawk_imu_.header.stamp.toSec(), pixhawk_imu_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk IMU timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_fcu_untilted_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not publishing IMU. Waiting for fcu_untilted_tf");
    return;
  }


  // transform imu accelerations to untilted frame
  geometry_msgs::Vector3Stamped acc_untilted;
  acc_untilted.vector          = pixhawk_imu_.linear_acceleration;
  acc_untilted.header          = pixhawk_imu_.header;
  acc_untilted.header.frame_id = fcu_frame_id_;
  auto response_acc            = transformer_.transformSingle(fcu_untilted_frame_id_, acc_untilted);

  if (response_acc) {
    acc_untilted = response_acc.value();

    if (!std::isfinite(acc_untilted.vector.x)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"acc_untilted.x\"!!!");
      return;
    }

    if (!std::isfinite(acc_untilted.vector.y)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"acc_untilted.y\"!!!");
      return;
    }
    stateEstimatorsCorrection(acc_untilted.vector.x, acc_untilted.vector.y, "acc_imu");

    ROS_INFO_ONCE("[Odometry]: Fusing untilted accelerations");

  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu_.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  // transform imu angular rates to untilted frame
  geometry_msgs::Vector3Stamped ang_vel_untilted;
  ang_vel_untilted.vector          = pixhawk_imu_.angular_velocity;
  ang_vel_untilted.header          = pixhawk_imu_.header;
  ang_vel_untilted.header.frame_id = fcu_frame_id_;
  auto response_ang_vel            = transformer_.transformSingle(fcu_untilted_frame_id_, ang_vel_untilted);

  if (response_ang_vel) {
    ang_vel_untilted = response_ang_vel.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu_.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  // transform imu attitude to untilted frame
  geometry_msgs::QuaternionStamped attitude_untilted;
  attitude_untilted.quaternion      = pixhawk_imu_.orientation;
  attitude_untilted.header          = pixhawk_imu_.header;
  attitude_untilted.header.frame_id = fcu_frame_id_;
  auto response_attitude            = transformer_.transformSingle(fcu_untilted_frame_id_, attitude_untilted);
  if (response_attitude) {
    attitude_untilted = response_attitude.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu_.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  sensor_msgs::Imu untilted_imu    = pixhawk_imu_;
  untilted_imu.header              = acc_untilted.header;
  untilted_imu.linear_acceleration = acc_untilted.vector;
  untilted_imu.angular_velocity    = ang_vel_untilted.vector;
  untilted_imu.orientation         = attitude_untilted.quaternion;

  try {
    pub_imu_untilted_.publish(untilted_imu);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_imu_untilted_.getTopic().c_str());
  }

  //////////////////// Fuse Heading Kalman ////////////////////

  double    hdg_rate;
  ros::Time time_now = ros::Time::now();
  ros::Time time_imu;

  auto odom_pixhawk_tmp = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_);
  {
    std::scoped_lock lock(mutex_pixhawk_imu_);

    try {
      hdg_rate = mrs_lib::AttitudeConverter(odom_pixhawk_tmp.pose.pose.orientation).getHeadingRate(pixhawk_imu_.angular_velocity);
      time_imu = pixhawk_imu_.header.stamp;
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading rate (pixhawk_imu_)");
    }
  }

  if (std::isfinite(hdg_rate)) {
    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(hdg_rate, "rate_gyro", time_imu, time_now);

    ROS_INFO_ONCE("[Odometry]: Fusing gyro hdg rate from PixHawk IMU");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in PixHawk IMU variable \"hdg_rate\", not fusing!!!");
  }

  //////////////////// Fuse Linear Z Acceleration ////////////////////

  geometry_msgs::Quaternion q_body;
  {
    std::scoped_lock lock(mutex_odom_pixhawk_);
    q_body = odom_pixhawk_.pose.pose.orientation;
  }
}

//}

/* //{ callbackPixhawkCompassHdg() */

void Odometry::callbackPixhawkCompassHdg(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPixhawkCompassHdg");

  {
    std::scoped_lock lock(mutex_compass_hdg_);

    if (got_compass_hdg_) {

      compass_hdg_previous_ = compass_hdg_;
      compass_hdg_          = *msg;

    } else {

      compass_hdg_previous_    = *msg;
      compass_hdg_             = *msg;
      hdg_previous_            = msg->data / 180 * M_PI;
      got_compass_hdg_         = true;
      compass_hdg_last_update_ = ros::Time::now();

      return;
    }
  }

  if (!init_hdg_avg_done_) {

    ROS_INFO_ONCE("[Odometry]: Averaging initial compass heading.");

    double hdg = compass_hdg_.data;

    // convert from degrees to radians
    hdg = hdg / 180 * M_PI;

    if (got_compass_hdg_) {
      hdg = radians::unwrap(hdg, hdg_previous_);
    }

    init_hdg_avg_ += M_PI / 2 - hdg;
    if (++init_hdg_avg_samples_ > 100) {
      init_hdg_avg_ /= 100;
      init_hdg_avg_done_ = true;
      ROS_INFO("[Odometry]: Initial compass heading averaged to %f", init_hdg_avg_);
    }
    return;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  // Compass heading msg does not have timestamp - check at least time of msg arrival
  if (!isTimestampOK(ros::Time::now().toSec(), compass_hdg_last_update_.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk compass heading time between msgs not OK, not fusing correction.");
    compass_hdg_last_update_ = ros::Time::now();
    return;
  }
  compass_hdg_last_update_ = ros::Time::now();

  //////////////////// Fuse Heading Kalman ////////////////////

  double hdg = mrs_lib::get_mutexed(mutex_compass_hdg_, compass_hdg_.data);

  hdg           = hdg / 180 * M_PI;
  hdg           = radians::unwrap(hdg, hdg_previous_);
  hdg_previous_ = hdg;
  hdg           = M_PI / 2 - hdg;  // TODO is the compass heading still this weird?

  if (toUppercase(current_hdg_estimator_name_) == "COMPASS" && _gyro_fallback_ && compass_inconsistent_samples_ > 20) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Compass inconsistent. Swtiching to GYRO heading estimator.");
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::GYRO;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    changeCurrentHeadingEstimator(desired_estimator);
    compass_inconsistent_samples_ = 0;
  }
  return;

  if (!std::isfinite(hdg)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in PixHawk compass variable \"hdg\", not fusing!!!");
    return;
  }

  compass_inconsistent_samples_ = 0;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(hdg, "hdg_compass");

  if (_debug_publish_corrections_) {
    hdg = radians::wrap(hdg);
    mrs_msgs::Float64Stamped compass_hdg_out;
    compass_hdg_out.header.stamp    = ros::Time::now();
    compass_hdg_out.header.frame_id = local_origin_frame_id_;
    compass_hdg_out.value           = hdg;
    try {
      pub_compass_hdg_corr_.publish(compass_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_compass_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from PixHawk compass");
}

//}

/* //{ callbackOptflowTwist() */

void Odometry::callbackOptflowTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  if (_use_optflow_low_ && (isUavLandoff() || !isUavFlying())) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow regular.");
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOptflowTwist");

  optflow_twist_last_update_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_optflow_);

    if (got_optflow_) {

      optflow_twist_previous_ = optflow_twist_;
      optflow_twist_          = *msg;

    } else {

      optflow_twist_previous_ = *msg;
      optflow_twist_          = *msg;

      got_optflow_      = true;
      optflow_reliable_ = true;

      return;
    }
  }

  if (!got_range_ && garmin_enabled_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optic flow. No range msgs.");
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(optflow_twist_.header.stamp.toSec(), optflow_twist_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Optflow twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow velocity. Waiting for other sensors.");
    return;
  }

  // Change to OPTFLOW hdg estimator if OPTFLOW low was being used
  if (_use_optflow_low_ && fusing_optflow_low_) {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Switching from OPTFLOW low to OPTFLOW regular");
    fusing_optflow_low_ = false;
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    changeCurrentHeadingEstimator(desired_estimator);
  }

  static int    measurement_id     = 0;
  static bool   got_init_optflow_Q = false;
  static double init_Q             = 0.0;

  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "OPTFLOW" || estimator.first == "BRICKFLOW") {

      // Get initial Q
      if (!got_init_optflow_Q) {
        std::string                          measurement_name  = "vel_optflow";
        std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id_.find(measurement_name);
        if (it_measurement_id == map_measurement_name_id_.end()) {
          ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
          return;
        }

        measurement_id = it_measurement_id->second;

        estimator.second->getR(init_Q, measurement_id);
        got_init_optflow_Q = true;
      }
    }
  }

  if (_dynamic_optflow_cov_) {
    double twist_q_x = optflow_twist_.twist.covariance[0];
    double twist_q_y = optflow_twist_.twist.covariance[7];

    if (std::isfinite(twist_q_x)) {

      // Scale covariance
      twist_q_x *= _dynamic_optflow_cov_scale_;
      twist_q_y *= _dynamic_optflow_cov_scale_;

      double twist_q = std::max(twist_q_x, twist_q_y);

      std::string                          measurement_name  = "vel_optflow";
      std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id_.find(measurement_name);
      if (it_measurement_id == map_measurement_name_id_.end()) {
        ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
        return;
      }

      for (auto &estimator : _lateral_estimators_) {
        if (estimator.first == "OPTFLOW" || estimator.first == "OPTFLOWGPS" || estimator.first == "BRICKFLOW") {
          estimator.second->setR(twist_q, it_measurement_id->second);
          ROS_INFO_THROTTLE(5.0, "[Odometry]: estimator: %s setting Q_optflow_twist to: %f", estimator.first.c_str(), twist_q);
        }
      }
    } else {
      twist_q_x = twist_q_x_prev_;
      twist_q_y = twist_q_y_prev_;
    }
    twist_q_x_prev_ = twist_q_x;
    twist_q_y_prev_ = twist_q_y;
  }

  double optflow_vel_x = optflow_twist_.twist.twist.linear.x;
  double optflow_vel_y = optflow_twist_.twist.twist.linear.y;

  bool optflow_vel_ok = true;
  if (!isValidGate(optflow_vel_x, -_optflow_max_valid_twist_, _optflow_max_valid_twist_, "optflow twist")) {
    optflow_vel_ok = false;
  }

  if (_use_lat_mf_optflow_twist_ && optflow_vel_ok) {
    if (!lat_mf_optflow_twist_x_->isValid(optflow_vel_x)) {

      double median = lat_mf_optflow_twist_x_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow x velocity filtered by median filter. %f -> %f", optflow_vel_x, median);
      optflow_vel_x = median;
    }

    if (!lat_mf_optflow_twist_y_->isValid(optflow_vel_y)) {
      double median = lat_mf_optflow_twist_y_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow y velocity filtered by median filter. %f -> %f", optflow_vel_y, median);
      optflow_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped optflow_filtered = optflow_twist_;
  optflow_filtered.twist.twist.linear.x                      = optflow_vel_x;
  optflow_filtered.twist.twist.linear.y                      = optflow_vel_y;

  try {
    pub_debug_optflow_filter_.publish(optflow_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_optflow_filter_.getTopic().c_str());
  }

  // Set innoation variable if ccurnet estimator is OPTFLOW
  if (current_lat_estimator_->getName() == "OPTFLOW") {
    Vec2 vel_vec, innovation;
    current_lat_estimator_->getState(1, vel_vec);

    innovation(0) = optflow_vel_x - vel_vec(0);
    innovation(1) = optflow_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = 0;
      odom_main_inno_.pose.pose.position.y = 0;
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = innovation(0);
      odom_main_inno_.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  if (optflow_vel_ok) {
    stateEstimatorsCorrection(optflow_vel_x, optflow_vel_y, "vel_optflow");

    ROS_INFO_ONCE("[Odometry]: Fusing optflow velocity");
  }

  double hdg_rate = mrs_lib::get_mutexed(mutex_optflow_, optflow_twist_.twist.twist.angular.z);

  if (!isValidGate(hdg_rate, -_optflow_max_valid_hdg_rate_, _optflow_max_valid_hdg_rate_, "optflow hdg rate")) {
    return;
  }

  if (!hdg_mf_optflow_rate_->isValid(hdg_rate) && hdg_mf_optflow_rate_->isFilled()) {
    optflow_inconsistent_samples_++;
    ROS_WARN("[Odometry]: Optflow hdg rate inconsistent: %f. Not fusing.", hdg_rate);

    if (_gyro_fallback_ && optflow_inconsistent_samples_ > 20) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optflow hdg rate inconsistent. Swtiching to GYRO heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = mrs_msgs::HeadingType::GYRO;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      --optflow_inconsistent_samples_;
      optflow_inconsistent_samples_ = std::max(0, optflow_inconsistent_samples_);
    }
    return;
  }


  if (std::isfinite(hdg_rate)) {

    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(hdg_rate, "rate_optflow");

    ROS_INFO_ONCE("[Odometry]: Fusing optflow hdg rate");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in optflow variable \"hdg_rate\", not fusing!!!");
  }
}

//}

/* //{ callbackOptflowTwistLow() */

void Odometry::callbackOptflowTwistLow(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  if (isUavFlying() && !isUavLandoff()) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow low.");
    return;
  }

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackOptflowTwistLow");

  optflow_twist_last_update_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_optflow_);

    if (got_optflow_) {

      optflow_twist_previous_ = optflow_twist_;
      optflow_twist_          = *msg;

    } else {

      optflow_twist_previous_ = *msg;
      optflow_twist_          = *msg;

      got_optflow_      = true;
      optflow_reliable_ = true;

      return;
    }
  }

  if (!got_range_ && garmin_enabled_) {
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(optflow_twist_.header.stamp.toSec(), optflow_twist_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Optflow twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow velocity. Waiting for other sensors.");
    return;
  }

  fusing_optflow_low_ = true;

  if (toUppercase(current_hdg_estimator_name_) == "OPTFLOW") {
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::GYRO;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    changeCurrentHeadingEstimator(desired_estimator);
  }

  static int    measurement_id     = 0;
  static bool   got_init_optflow_Q = false;
  static double init_Q             = 0.0;

  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "OPTFLOW" || estimator.first == "BRICKFLOW") {

      // Get initial Q
      if (!got_init_optflow_Q) {
        std::string                          measurement_name  = "vel_optflow";
        std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id_.find(measurement_name);
        if (it_measurement_id == map_measurement_name_id_.end()) {
          ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
          return;
        }

        measurement_id = it_measurement_id->second;

        estimator.second->getR(init_Q, measurement_id);
        got_init_optflow_Q = true;
      }
    }
  }

  if (_dynamic_optflow_cov_) {
    double twist_q_x = optflow_twist_.twist.covariance[0];
    double twist_q_y = optflow_twist_.twist.covariance[7];

    if (std::isfinite(twist_q_x)) {

      // Scale covariance
      twist_q_x *= _dynamic_optflow_cov_scale_;
      twist_q_y *= _dynamic_optflow_cov_scale_;

      double twist_q = std::max(twist_q_x, twist_q_y);

      std::string                          measurement_name  = "vel_optflow";
      std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id_.find(measurement_name);
      if (it_measurement_id == map_measurement_name_id_.end()) {
        ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
        return;
      }

      for (auto &estimator : _lateral_estimators_) {
        if (estimator.first == "OPTFLOW" || estimator.first == "OPTFLOWGPS" || estimator.first == "BRICKFLOW") {
          estimator.second->setR(twist_q, it_measurement_id->second);
          ROS_INFO_THROTTLE(5.0, "[Odometry]: estimator: %s setting Q_optflow_twist to: %f", estimator.first.c_str(), twist_q);
        }
      }
    } else {
      twist_q_x = twist_q_x_prev_;
      twist_q_y = twist_q_y_prev_;
    }
    twist_q_x_prev_ = twist_q_x;
    twist_q_y_prev_ = twist_q_y;
  }

  double optflow_vel_x = optflow_twist_.twist.twist.linear.x;
  double optflow_vel_y = optflow_twist_.twist.twist.linear.y;

  bool optflow_vel_ok = true;
  if (!isValidGate(optflow_vel_x, -_optflow_max_valid_twist_, _optflow_max_valid_twist_, "optflow twist")) {
    optflow_vel_ok = false;
  }

  if (lat_mf_optflow_twist_x_ && optflow_vel_ok) {
    if (!lat_mf_optflow_twist_x_->isValid(optflow_vel_x)) {

      double median = lat_mf_optflow_twist_x_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow x velocity filtered by median filter. %f -> %f", optflow_vel_x, median);
      optflow_vel_x = median;
    }

    if (!lat_mf_optflow_twist_y_->isValid(optflow_vel_y)) {
      double median = lat_mf_optflow_twist_y_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow y velocity filtered by median filter. %f -> %f", optflow_vel_y, median);
      optflow_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped optflow_filtered = optflow_twist_;
  optflow_filtered.twist.twist.linear.x                      = optflow_vel_x;
  optflow_filtered.twist.twist.linear.y                      = optflow_vel_y;

  try {
    pub_debug_optflow_filter_.publish(optflow_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_optflow_filter_.getTopic().c_str());
  }
  // Set innoation variable if ccurnet estimator is OPTFLOW
  if (current_lat_estimator_->getName() == "OPTFLOW") {
    Vec2 vel_vec, innovation;
    current_lat_estimator_->getState(1, vel_vec);

    innovation(0) = optflow_vel_x - vel_vec(0);
    innovation(1) = optflow_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = 0;
      odom_main_inno_.pose.pose.position.y = 0;
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = innovation(0);
      odom_main_inno_.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(optflow_vel_x, optflow_vel_y, "vel_optflow");

  ROS_INFO_ONCE("[Odometry]: Fusing optflow velocity from OPTFLOW low");
}

//}

/* //{ callbackICPTwist() */

void Odometry::callbackICPTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg) {

  // Note: ICP twist is coming in the UAV body frame (uav/fcu)
  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackICPTwist");

  icp_twist_last_update_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_icp_twist_);

    if (got_icp_twist_) {

      icp_twist_previous_ = icp_twist_;
      icp_twist_          = *msg;

    } else {

      icp_twist_previous_ = *msg;
      icp_twist_          = *msg;

      got_icp_twist_ = true;
      icp_reliable_  = true;

      return;
    }
  }

  if (!got_range_ && garmin_enabled_) {
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(icp_twist_.header.stamp.toSec(), icp_twist_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: ICP twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP velocity. Waiting for other sensors.");
    return;
  }

  double icp_vel_x = icp_twist_.twist.twist.linear.x;
  double icp_vel_y = icp_twist_.twist.twist.linear.y;

  bool icp_vel_ok = true;
  if (!isValidGate(icp_vel_x, -_icp_max_valid_twist_, _icp_max_valid_twist_, "icp twist")) {
    icp_vel_ok = false;
  }

  if (_use_lat_mf_icp_twist_ && icp_vel_ok) {
    if (!lat_mf_icp_twist_x_->isValid(icp_vel_x)) {

      double median = lat_mf_icp_twist_x_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP x velocity filtered by median filter. %f -> %f", icp_vel_x, median);
      icp_vel_x = median;
    }

    if (!lat_mf_icp_twist_y_->isValid(icp_vel_y)) {
      double median = lat_mf_icp_twist_y_->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP y velocity filtered by median filter. %f -> %f", icp_vel_y, median);
      icp_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped icp_twist_filtered = icp_twist_;
  icp_twist_filtered.twist.twist.linear.x                      = icp_vel_x;
  icp_twist_filtered.twist.twist.linear.y                      = icp_vel_y;

  try {
    pub_debug_icp_twist_filter_.publish(icp_twist_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_icp_twist_filter_.getTopic().c_str());
  }
  // Set innoation variable if ccurnet estimator is ICP
  if (current_lat_estimator_->getName() == "ICP") {
    Vec2 vel_vec, innovation;
    current_lat_estimator_->getState(1, vel_vec);

    innovation(0) = icp_vel_x - vel_vec(0);
    innovation(1) = icp_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = 0;
      odom_main_inno_.pose.pose.position.y = 0;
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = innovation(0);
      odom_main_inno_.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  if (icp_vel_ok) {
    ros::Time time_now = ros::Time::now();
    stateEstimatorsCorrection(icp_vel_x, icp_vel_y, "vel_icp", icp_twist_.header.stamp, time_now);

    ROS_INFO_ONCE("[Odometry]: Fusing icp velocity");
  }

  double hdg_rate = msg->twist.twist.angular.z;

  if (!isValidGate(hdg_rate, -_icp_max_valid_hdg_rate_, _icp_max_valid_hdg_rate_, "icp hdg rate")) {
    return;
  }

  if (!hdg_mf_icp_rate_->isValid(hdg_rate) && hdg_mf_icp_rate_->isFilled()) {
    icp_hdg_rate_inconsistent_samples_++;
    ROS_WARN("[Odometry]: icp hdg rate inconsistent: %f. Not fusing.", hdg_rate);

    if (_gyro_fallback_ && icp_hdg_rate_inconsistent_samples_ > 20) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: icp hdg rate inconsistent. Swtiching to GYRO heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = mrs_msgs::HeadingType::GYRO;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      --icp_hdg_rate_inconsistent_samples_;
      icp_hdg_rate_inconsistent_samples_ = std::max(0.0, icp_hdg_rate_inconsistent_samples_);
    }
    return;
  }

  if (std::isfinite(hdg_rate)) {

    // Apply correction step to all heading estimators
    ros::Time time_now = ros::Time::now();
    headingEstimatorsCorrection(hdg_rate, "rate_icp", msg->header.stamp, time_now);

    ROS_INFO_ONCE("[Odometry]: Fusing icp hdg rate");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in ICP variable \"hdg_rate\", not fusing!!!");
  }
}

//}

/* //{ callbackRtkGps() */

void Odometry::callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackRtk");

  mrs_msgs::RtkGps rtk_utm;

  if (msg->header.frame_id == "gps") {

    if (!std::isfinite(msg->gps.latitude)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in RTK variable \"msg->latitude\"!!!");
      return;
    }

    if (!std::isfinite(msg->gps.longitude)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in RTK variable \"msg->longitude\"!!!");
      return;
    }

    // convert it to UTM
    mrs_lib::UTM(msg->gps.latitude, msg->gps.longitude, &rtk_utm.pose.pose.position.x, &rtk_utm.pose.pose.position.y);
    rtk_utm.header               = msg->header;
    rtk_utm.header.frame_id      = "utm";
    rtk_utm.pose.pose.position.z = msg->gps.altitude;
    rtk_utm.fix_type             = msg->fix_type;
    rtk_utm.status               = msg->status;

  } else if (msg->header.frame_id == "utm") {

    rtk_utm = *msg;

  } else {

    ROS_INFO_THROTTLE(1.0, "[Odometry]: RTK message has unknown frame_id: '%s'", msg->header.frame_id.c_str());
  }

  {
    std::scoped_lock lock(mutex_rtk_);

    if (!isUavFlying()) {
      if (++got_rtk_counter_ <= 10) {
        rtk_local_origin_z_ += rtk_utm.pose.pose.position.z;
        ROS_INFO("[Odometry]: RTK ASL altitude sample #%d: %f", got_rtk_counter_, rtk_utm.pose.pose.position.z);
        return;

      } else {

        if (!got_rtk_local_origin_z_) {
          rtk_local_origin_z_ /= 10;
          rtk_local_origin_z_ -= _fcu_height_;
          got_rtk_local_origin_z_ = true;
          ROS_INFO("[Odometry]: RTK ASL altitude avg: %f", rtk_local_origin_z_);
        }
      }

    } else {
      if (!got_rtk_local_origin_z_) {
        rtk_local_origin_z_ = 0.0;
      }
    }

    if (_pass_rtk_as_odom_) {
      rtk_local_origin_z_ = 0.0;
    }

    got_rtk_         = true;
    rtk_reliable_    = true;
    rtk_last_update_ = ros::Time::now();

    rtk_local_previous_ = rtk_local_;
    rtk_local_          = rtk_utm;
  }

  if (!got_odom_pixhawk_ || !got_rtk_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not received RTK yet.");
    return;
  }

  if (!isTimestampOK(rtk_local_.header.stamp.toSec(), rtk_local_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: RTK local timestamp not OK, not fusing correction.");
    return;
  }

  // check whether we have rtk fix
  got_rtk_fix_ =
      (rtk_local_.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FLOAT || rtk_local_.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FIX) ? true : false;

  if (_rtk_fuse_sps_) {
    if (rtk_local_.fix_type.fix_type == mrs_msgs::RtkFixType::NO_FIX || rtk_local_.fix_type.fix_type == mrs_msgs::RtkFixType::UNKNOWN) {
      ROS_WARN_THROTTLE(1.0, "RTK fix type: NO_FIX. Not fusing RTK.");
      return;
    }
  } else {
    if (!got_rtk_fix_) {
      ROS_WARN_THROTTLE(1.0, "RTK not fusing SPS.");
      return;
    }
  }

  // | ------------- offset the rtk to local_origin ------------- |
  rtk_local_.pose.pose.position.x -= _utm_origin_x_;
  rtk_local_.pose.pose.position.y -= _utm_origin_y_;
  rtk_local_.pose.pose.position.z -= rtk_local_origin_z_;

  rtk_local_.header.frame_id = _uav_name_ + "/rtk_origin";

  // | ------------------ publish the rtk local ----------------- |
  mrs_msgs::RtkGps rtk_local_out = rtk_local_;

  try {
    pub_rtk_local_.publish(rtk_local_out);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local_.getTopic().c_str());
  }
  // | ------------- publish the rtk local odometry ------------- |
  {
    std::scoped_lock lock(mutex_rtk_local_odom_);

    rtk_local_odom_.header = rtk_local_.header;
    rtk_local_odom_.pose   = rtk_local_.pose;
    rtk_local_odom_.twist  = rtk_local_.twist;

    try {
      pub_rtk_local_odom_.publish(rtk_local_odom_);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local_odom_.getTopic().c_str());
    }
  }

  if (rtk_active_) {
    if (!std::isfinite(rtk_local_.pose.pose.position.x) || !std::isfinite(rtk_local_.pose.pose.position.y)) {

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local_.pose.pose.position.x\" or \"rtk_local_.pose.pose.position.y\" (rtk)!!!");

      return;
    }

    if (!got_lateral_sensors_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing RTK position. Waiting for other sensors.");
      return;
    }

    double x_rtk, y_rtk, z_rtk;
    {
      std::scoped_lock lock(mutex_rtk_);

      x_rtk = rtk_local_.pose.pose.position.x;
      y_rtk = rtk_local_.pose.pose.position.y;
      z_rtk = rtk_local_.pose.pose.position.z;
    }

    if (!std::isfinite(x_rtk)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"x_rtk\" (callbackRtk)!!!");
      return;
    }

    if (!std::isfinite(y_rtk)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"y_rtk\" (callbackRtk)!!!");
      return;
    }

    if (!std::isfinite(z_rtk)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"z_rtk\" (callbackRtk)!!!");
      return;
    }

    /* fuse rtk position //{ */
    // Saturate correction
    double x_est;
    double y_est;
    double z_est;
    {
      std::scoped_lock lock(mutex_rtk_est_);

      x_est = sc_lat_rtk_.x(0);
      y_est = sc_lat_rtk_.x(1);
    }

    alt_x_t alt_state = alt_state.Zero();
    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      for (auto &alt_estimator : _altitude_estimators_) {
        if (alt_estimator.first == "RTK") {
          if (!alt_estimator.second->getStates(alt_state)) {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
            return;
          }
        }
      }
    }

    z_est = alt_state(0);

    // X position
    double x_correction = x_rtk - x_est;
    if (!std::isfinite(x_rtk)) {
      x_rtk = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"x_rtk\", setting it to 0 and returning!!!");
      return;
    }
    if (x_correction > _max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK X pos correction %f -> %f", x_correction, _max_rtk_pos_correction_);
      x_correction = _max_rtk_pos_correction_;
    } else if (x_correction < -_max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK X pos correction %f -> %f", x_correction, -_max_rtk_pos_correction_);
      x_correction = -_max_rtk_pos_correction_;
    }

    // Y position
    double y_correction = y_rtk - y_est;
    if (!std::isfinite(y_rtk)) {
      y_rtk = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"y_rtk\", setting it to 0 and returning!!!");
      return;
    }
    if (y_correction > _max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK Y pos correction %f -> %f", y_correction, _max_rtk_pos_correction_);
      y_correction = _max_rtk_pos_correction_;
    } else if (y_correction < -_max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK Y pos correction %f -> %f", y_correction, -_max_rtk_pos_correction_);
      y_correction = -_max_rtk_pos_correction_;
    }

    // Z position
    double z_correction = z_rtk - z_est;
    if (!std::isfinite(z_rtk)) {
      z_rtk = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"z_rtk\", setting it to 0 and returning!!!");
      return;
    }
    if (z_correction > _max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK Z pos correction %f -> %f", z_correction, _max_rtk_pos_correction_);
      z_correction = _max_rtk_pos_correction_;
    } else if (z_correction < -_max_rtk_pos_correction_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK Z pos correction %f -> %f", z_correction, -_max_rtk_pos_correction_);
      z_correction = -_max_rtk_pos_correction_;
    }

    double z_measurement = z_est + z_correction;
    /* } */

    // Do RTK estimator lateral correction
    lkf_rtk_t::z_t rtk_meas;
    rtk_meas << x_est + x_correction, y_est + y_correction;
    {
      std::scoped_lock lock(mutex_rtk_est_);

      try {
        sc_lat_rtk_ = estimator_rtk_->correct(sc_lat_rtk_, rtk_meas, _R_lat_rtk_);
      }
      catch (const std::exception &e) {
        ROS_ERROR("[Odometry]: RTK LKF correction step failed: %s", e.what());
      }
    }
    //}

    // Do RTK estimator altitude correction
    for (auto &estimator : _altitude_estimators_) {
      altitudeEstimatorCorrection(z_measurement, "height_rtk", estimator.second);
    }

    ROS_INFO_ONCE("[Odometry]: Fusing RTK position");
  }
}
//}

/* //{ callbackVioOdometry() */

void Odometry::callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackVioOdometry");

  {
    std::scoped_lock lock(mutex_odom_vio_);

    odom_vio_last_update_ = ros::Time::now();

    if (got_vio_) {

      odom_vio_previous_ = odom_vio_;
      odom_vio_          = *msg;

    } else {

      odom_vio_previous_ = *msg;
      odom_vio_          = *msg;

      try {
        vio_hdg_previous_ = mrs_lib::AttitudeConverter(odom_vio_.pose.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during getting heading (vio_hdg_previous_)");
      }

      got_vio_      = true;
      vio_reliable_ = true;

      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  auto odom_vio_previous = mrs_lib::get_mutexed(mutex_odom_vio_, odom_vio_previous_);
  auto odom_vio          = mrs_lib::get_mutexed(mutex_odom_vio_, odom_vio_);

  if (!isTimestampOK(msg->header.stamp.toSec(), odom_vio_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: VIO timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse vio hdg //{ */
  double vio_hdg;
  try {
    vio_hdg = mrs_lib::AttitudeConverter(odom_vio.pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading (vio_hdg)");
    return;
  }

  vio_hdg           = radians::unwrap(vio_hdg, vio_hdg_previous_);
  vio_hdg_previous_ = vio_hdg;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(vio_hdg, "hdg_vio");

  // Publish VIO heading correction
  if (_debug_publish_corrections_) {
    vio_hdg = radians::wrap(vio_hdg);
    mrs_msgs::Float64Stamped vio_hdg_out;
    vio_hdg_out.header.stamp    = ros::Time::now();
    vio_hdg_out.header.frame_id = local_origin_frame_id_;
    vio_hdg_out.value           = vio_hdg;
    try {
      pub_vio_hdg_corr_.publish(vio_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_vio_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from VIO");

  //}

  /* fuse vio height //{ */

  //////////////////// Filter out vio height measurement ////////////////////

  {
    bool   vio_altitude_ok = true;
    double measurement     = msg->pose.pose.position.z;

    // Value gate
    if (!isValidGate(measurement, _vio_min_valid_alt_, _vio_max_valid_alt_, "vio altitude")) {
      vio_altitude_ok = false;
    }

    // Median filter
    if (isUavFlying() && vio_altitude_ok) {
      if (!alt_mf_vio_->isValid(measurement)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO height easurement %f declined by median filter.", measurement);
        vio_altitude_ok = false;
      }
    }

    //////////////////// Fuse main altitude kalman ////////////////////

    // Fuse vio measurement for each altitude estimator
    for (auto &estimator : _altitude_estimators_) {
      alt_x_t alt_x = alt_x.Zero();
      if (!estimator.second->getStates(alt_x)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        vio_altitude_ok = false;
      }

      if (vio_altitude_ok) {
        {
          std::scoped_lock lock(mutex_altitude_estimator_);
          altitudeEstimatorCorrection(measurement, "height_vio", estimator.second);
          if (fabs(measurement) > 100) {
            ROS_WARN("[Odometry]: VIO height correction: %f", measurement);
          }
        }
      }
    }

    ROS_INFO_ONCE("[Odometry]: Fusing height from VIO");
  }

  {
    // TODO Matej check after TOMAS

    double measurement           = msg->twist.twist.linear.z;
    bool   vio_altitude_speed_ok = true;

    //////////////////// Fuse main altitude speed kalman ////////////////////

    // Fuse vio measurement for each altitude estimator
    for (auto &estimator : _altitude_estimators_) {

      alt_x_t alt_x = alt_x.Zero();

      if (!estimator.second->getStates(alt_x)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        vio_altitude_speed_ok = false;
      }

      if (vio_altitude_speed_ok) {
        {
          std::scoped_lock lock(mutex_altitude_estimator_);
          altitudeEstimatorCorrection(measurement, "vel_vio", estimator.second);
        }
      }
    }

    ROS_INFO_ONCE("[Odometry]: Fusing height speed from VIO");
  }

  //}

  //////////////////// Fuse Lateral Kalman ////////////////////

  /* //{ fuse vio velocity */

  double vel_vio_x, vel_vio_y;
  vel_vio_x = msg->twist.twist.linear.x;
  vel_vio_y = msg->twist.twist.linear.y;

  // Set innovation variable if current estimator is VIO
  if (current_lat_estimator_->getName() == "VIO") {
    Vec2 vel_vec, innovation;
    current_lat_estimator_->getState(1, vel_vec);

    innovation(0) = vel_vio_x - vel_vec(0);
    innovation(1) = vel_vio_y - vel_vec(1);

    mrs_lib::set_mutexed(mutex_odom_main_inno_, innovation(0), odom_main_inno_.twist.twist.linear.x);
    mrs_lib::set_mutexed(mutex_odom_main_inno_, innovation(1), odom_main_inno_.twist.twist.linear.y);
  }

  if (vio_reliable_ && (vel_vio_x > 10 || vel_vio_y > 10)) {
    ROS_WARN("[Odometry]: Estimated VIO velocity > 10. VIO is not reliable.");
    vio_reliable_ = false;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vel_vio_x, vel_vio_y, "vel_vio");

  ROS_INFO_ONCE("[Odometry]: Fusing VIO velocity");
  //}

  /* //{ fuse vio position */

  double vio_pos_x, vio_pos_y;
  vio_pos_x = msg->pose.pose.position.x;
  vio_pos_y = msg->pose.pose.position.y;

  if (!std::isfinite(vio_pos_x) || !std::isfinite(vio_pos_y)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vio_pos_x\" or \"vio_pos_y\", returning!!!");
    return;
  }

  // Saturate correction
  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "VIO") {

      Vec2 pos_vec, correction;
      estimator.second->getState(0, pos_vec);

      correction(0) = vio_pos_x - pos_vec(0);
      correction(1) = vio_pos_y - pos_vec(1);

      // X position
      vio_pos_x += saturateCorrection(correction(0), _max_vio_pos_correction_, "vio_pos_x");
      vio_pos_y += saturateCorrection(correction(1), _max_vio_pos_correction_, "vio_pos_y");

      // Set innovation variable if current estimator is VIO
      if (current_lat_estimator_->getName() == "VIO") {
        mrs_lib::set_mutexed(mutex_odom_main_inno_, correction(0), odom_main_inno_.pose.pose.position.x);
        mrs_lib::set_mutexed(mutex_odom_main_inno_, correction(1), odom_main_inno_.pose.pose.position.y);
      }
    }
  }

  if (vio_reliable_ && (std::fabs(msg->pose.pose.position.x - odom_vio_previous.pose.pose.position.x) > 10 ||
                        std::fabs(msg->pose.pose.position.y - odom_vio_previous.pose.pose.position.y) > 10)) {
    ROS_WARN("[Odometry]: Estimated difference between successive VIO positions > 10. VIO is not reliable.");
    vio_reliable_ = false;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vio_pos_x, vio_pos_y, "pos_vio");

  ROS_INFO_ONCE("[Odometry]: Fusing VIO position");
  //}
}


//}

/* //{ callbackVslamPose() */

void Odometry::callbackVslamPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackVslamPose");

  geometry_msgs::PoseWithCovarianceStamped pose_vslam_previous;

  if (got_vslam_) {

    pose_vslam_previous = pose_vslam_;
    mrs_lib::set_mutexed(mutex_pose_vslam_previous_, pose_vslam_previous, pose_vslam_previous_);
    mrs_lib::set_mutexed(mutex_pose_vslam_, *msg, pose_vslam_);
    pose_vslam_last_update_ = ros::Time::now();

  } else {

    mrs_lib::set_mutexed(mutex_pose_vslam_previous_, *msg, pose_vslam_previous_);
    mrs_lib::set_mutexed(mutex_pose_vslam_, *msg, pose_vslam_);
    pose_vslam_last_update_ = ros::Time::now();
    got_vslam_              = true;

    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(msg->header.stamp.toSec(), pose_vslam_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: VSLAM timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse VSLAM heading //{ */

  double vslam_hdg_previous_;
  try {
    vslam_hdg_previous_ = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading (vslam_hdg_previous_)");
    return;
  }

  double vslam_hdg;
  try {
    vslam_hdg = mrs_lib::AttitudeConverter(msg->pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading (vslam_hdg)");
    return;
  }

  vslam_hdg           = radians::unwrap(vslam_hdg, vslam_hdg_previous_);
  vslam_hdg_previous_ = vslam_hdg;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(vslam_hdg, "hdg_vslam");

  if (_debug_publish_corrections_) {
    vslam_hdg = radians::wrap(vslam_hdg);

    mrs_msgs::Float64Stamped vslam_hdg_out;
    vslam_hdg_out.header.stamp    = ros::Time::now();
    vslam_hdg_out.header.frame_id = local_origin_frame_id_;
    vslam_hdg_out.value           = vslam_hdg;
    try {
      pub_vslam_hdg_corr_.publish(vslam_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_vslam_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from VSLAM");

  //}

  //////////////////// Fuse Lateral Kalman ////////////////////

  /* //{ fuse VSLAM position */

  double vslam_pos_x, vslam_pos_y;
  vslam_pos_x = msg->pose.pose.position.x;
  vslam_pos_y = msg->pose.pose.position.y;

  if (!std::isfinite(vslam_pos_x) || !std::isfinite(vslam_pos_y)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vslam_pos_x\" or \"vslam_pos_y\", returning!!!");
    return;
  }

  // Saturate correction
  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "VSLAM") {
      Vec2 pos_vec, correction;
      current_lat_estimator_->getState(0, pos_vec);

      correction(0) = vslam_pos_x - pos_vec(0);
      correction(1) = vslam_pos_y - pos_vec(1);

      vslam_pos_x += saturateCorrection(correction(0), _max_vslam_pos_correction_, "vio_pos_x");
      vslam_pos_y += saturateCorrection(correction(1), _max_vslam_pos_correction_, "vio_pos_y");

      // Set innovation variable if current estimator is VSLAM
      if (current_lat_estimator_->getName() == "VSLAM") {
        mrs_lib::set_mutexed(mutex_odom_main_inno_, correction(0), odom_main_inno_.pose.pose.position.x);
        mrs_lib::set_mutexed(mutex_odom_main_inno_, correction(1), odom_main_inno_.pose.pose.position.y);
      }
    }
  }

  if (vslam_reliable_ && (std::fabs(msg->pose.pose.position.x - pose_vslam_previous.pose.pose.position.x) > 10 ||
                          std::fabs(msg->pose.pose.position.y - pose_vslam_previous.pose.pose.position.y) > 10)) {
    ROS_WARN("[Odometry]: Estimated difference between VSLAM positions > 10. VSLAM is not reliable.");
    vslam_reliable_ = false;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vslam_pos_x, vslam_pos_y, "pos_vslam");

  ROS_INFO_ONCE("[Odometry]: Fusing VSLAM position");
  //}
}

//}

/* //{ callbackBrickPose() */

void Odometry::callbackBrickPose(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackBrickPose");

  brick_pose_last_update_ = ros::Time::now();

  if (got_brick_pose_) {

    brick_pose_previous_ = brick_pose_;
    brick_pose_          = *msg;

  } else {

    brick_pose_previous_ = *msg;
    brick_pose_          = *msg;
    try {
      brick_hdg_previous_ = mrs_lib::AttitudeConverter(brick_pose_.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading (brick_pose_)");
    }

    got_brick_pose_ = true;
    brick_reliable_ = true;
    return;
  }

  // detect jumps in brick measurements
  double diff_x = std::pow(brick_pose_.pose.position.x - brick_pose_previous_.pose.position.x, 2);
  if (diff_x > max_safe_brick_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump x: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_x), std::sqrt(max_safe_brick_jump_sq_));
    if (brick_reliable_ && toUppercase(current_lat_estimator_name_) == "BRICK") {
      c_failed_brick_x_++;
    }
    brick_reliable_ = false;
    return;
  }
  double diff_y = std::pow(brick_pose_.pose.position.y - brick_pose_previous_.pose.position.y, 2);
  if (diff_y > max_safe_brick_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump y: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_y), std::sqrt(max_safe_brick_jump_sq_));
    if (brick_reliable_ && toUppercase(current_lat_estimator_name_) == "BRICK") {
      c_failed_brick_y_++;
    }
    brick_reliable_ = false;
    return;
  }

  double dt = (brick_pose_.header.stamp - brick_pose_previous_.header.stamp).toSec();
  if (dt < 0.0001) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: received the same brick pose msg. returning");
    return;
  }

  // brick times out after not being received for some time
  if (brick_reliable_ || brick_semi_reliable_) {

    if ((ros::Time::now() - brick_pose_.header.stamp).toSec() > _brick_timeout_) {

      ROS_WARN_THROTTLE(1.0, "[Odometry]: brick timed out, not reliable");
      if (brick_reliable_ && toUppercase(current_lat_estimator_name_) == "BRICK") {
        c_failed_brick_timeout_++;
      }
      brick_reliable_      = false;
      brick_semi_reliable_ = false;

      return;
    }
  }

  // brick appears after not seeing it for long time -> becomes semi-reliable
  if (!brick_reliable_ && !brick_semi_reliable_) {

    if ((ros::Time::now() - brick_pose_.header.stamp).toSec() < 1.0) {

      brick_semi_reliable_         = true;
      brick_semi_reliable_started_ = ros::Time::now();

      ROS_WARN_THROTTLE(1.0, "[Odometry]: brick becomes semi-reliable.");
    }
  }

  // brick is semi-reliable for some time -> becomes reliable
  if (brick_semi_reliable_ && (ros::Time::now() - brick_semi_reliable_started_).toSec() > 1.0) {

    for (auto &estimator : _lateral_estimators_) {

      if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
        Vec2 pos_vec;
        pos_vec << brick_pose_.pose.position.x, brick_pose_.pose.position.y;
        estimator.second->setState(0, pos_vec);
      }
    }
    for (auto &estimator : heading_estimators_) {
      if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
        try {
          init_brick_hdg_ = mrs_lib::AttitudeConverter(brick_pose_.pose.orientation).getHeading();
        }
        catch (...) {
          ROS_ERROR("[Odometry]: Exception caught during setting heading (odom_aux orientation)");
        }
        double hdg = init_brick_hdg_;
        estimator.second->setState(0, hdg);
      }
    }

    ROS_WARN("[Odometry]: Brick is now reliable");
    brick_reliable_      = true;
    brick_semi_reliable_ = false;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(brick_pose_.header.stamp.toSec(), brick_pose_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: brick pose timestamp not OK, not fusing correction.");
    return;
  }

  /* brick estimator reset //{ */

  // synchronization hack
  if (brick_pose_.pose.position.z == -1.0) {
    ROS_INFO("[Odometry]: Detected -1.0 in Z position of brick pose msg. Starting BRICK estimator reset.");

    LatState2D states;
    bool       success = false;

    states(0, 0) = brick_pose_.pose.position.x;
    states(1, 0) = 0.0;
    states(2, 0) = 0.0;
    states(3, 0) = 0.0;
    states(4, 0) = 0.0;
    states(5, 0) = 0.0;
    states(0, 1) = brick_pose_.pose.position.y;
    states(1, 1) = 0.0;
    states(2, 1) = 0.0;
    states(3, 1) = 0.0;
    states(4, 1) = 0.0;
    states(5, 1) = 0.0;

    for (auto &estimator : _lateral_estimators_) {
      if (estimator.first == "BRICK") {
        success = estimator.second->reset(states);
      }
    }

    Eigen::MatrixXd hdg_states = Eigen::MatrixXd::Zero(3, 1);
    try {
      states(0, 0) = mrs_lib::AttitudeConverter(brick_pose_.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during setting heading (brick_pose_ hdg_states reset)");
    }

    if (success) {
      for (auto &estimator : heading_estimators_) {
        if (estimator.first == "BRICK") {
          success &= estimator.second->reset(hdg_states);
        }
      }
    }
    if (success) {
      ROS_INFO("[Odometry]: BRICK estimator reset finished. New brick position: x: %f y: %f, hdg: %f", states(0, 0), states(0, 1), hdg_states(0, 0));
      if (toUppercase(current_lat_estimator_name_) == "BRICK") {
        ROS_INFO("[Odometry]: Triggering control state update.");
        estimator_iteration_++;
      }
    } else {
      ROS_INFO("[Odometry]: Resetting BRICK estimator failed.");
    }
    ROS_INFO("[Odometry]: This msg triggered BRICK estimator reset. Not fusing brick pose this msg.");
    return;
  }

  //}


  double hdg_tmp;
  try {
    hdg_tmp = mrs_lib::AttitudeConverter(brick_pose_.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during getting heading (hdg_tmp)");
    return;
  }

  double hdg_brick = radians::unwrap(hdg_tmp, brick_hdg_previous_);

  double diff_hdg = std::pow(hdg_brick - brick_hdg_previous_, 2);
  if (diff_hdg > max_safe_brick_hdg_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump hdg: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_hdg), std::sqrt(max_safe_brick_hdg_jump_sq_));
    if (brick_reliable_ && toUppercase(current_lat_estimator_name_) == "BRICK") {
      c_failed_brick_hdg_++;
    }
    brick_reliable_     = false;
    brick_hdg_previous_ = hdg_brick;
    return;
  }

  brick_hdg_previous_ = hdg_brick;

  if (!std::isfinite(hdg_brick)) {
    ROS_WARN("[Odometry]: NaN in brick hdg");
    return;
  }

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(hdg_brick, "hdg_brick");

  if (_debug_publish_corrections_) {
    hdg_brick = radians::wrap(hdg_brick);

    mrs_msgs::Float64Stamped brick_hdg_out;
    brick_hdg_out.header.stamp    = ros::Time::now();
    brick_hdg_out.header.frame_id = local_origin_frame_id_;
    brick_hdg_out.value           = hdg_brick;
    try {
      pub_brick_hdg_corr_.publish(brick_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_brick_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from brick pose");

  //////////////////// Filter out brick height measurement ////////////////////
  // do not fuse brick height measurements when a height jump is detected - most likely the UAV is flying above an obstacle

  double measurement       = brick_pose_.pose.position.z;
  bool   fuse_brick_height = true;
  if (isUavFlying()) {
    if (!alt_mf_brick_->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Brick height easurement %f declined by median filter.", measurement);
      fuse_brick_height = false;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////

  if (!isValidGate(measurement, _brick_min_valid_alt_, _brick_max_valid_alt_, "brick height")) {
    fuse_brick_height = false;
    return;
  }

  // Fuse brick measurement for each altitude estimator
  if (fuse_brick_height && brick_reliable_) {
    for (auto &estimator : _altitude_estimators_) {
      alt_x_t alt_x = alt_x.Zero();
      if (!estimator.second->getStates(alt_x)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }

      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        altitudeEstimatorCorrection(measurement, "height_brick", estimator.second);
        if (fabs(measurement) > 100) {
          ROS_WARN("[Odometry]: Brick height correction: %f", measurement);
        }
      }
    }

    ROS_INFO_ONCE("[Odometry]: Brick height from brick pose");
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing brick pose. Waiting for other sensors.");
    return;
  }

  double pos_brick_x, pos_brick_y;

  {
    std::scoped_lock lock(mutex_brick_);

    pos_brick_x = brick_pose_.pose.position.x;
    pos_brick_y = brick_pose_.pose.position.y;
  }

  for (auto &estimator : heading_estimators_) {
    if (estimator.first == "BRICK") {
      double state;
      if (!estimator.second->getState(0, state)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Heading estimator not initialized.");
        return;
      }
      break;
    }
  }

  // Set innovation variable if current estimator is VIO
  if (current_lat_estimator_->getName() == "BRICK") {
    Vec2 pos_vec, innovation;
    current_lat_estimator_->getState(0, pos_vec);

    innovation(0) = pos_brick_x - pos_vec(0);
    innovation(1) = pos_brick_y - pos_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = innovation(0);
      odom_main_inno_.pose.pose.position.y = innovation(1);
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = 0;
      odom_main_inno_.twist.twist.linear.y = 0;
    }
  }

  // Apply correction step to all state estimators
  if (brick_reliable_) {
    stateEstimatorsCorrection(pos_brick_x, pos_brick_y, "pos_brick");
    ROS_INFO_THROTTLE(1.0, "[Odometry]: fusing brick: x: %f, y: %f", pos_brick_x, pos_brick_y);
  }

  ROS_INFO_ONCE("[Odometry]: Fusing brick position");
}
//}

/* //{ callbackHectorPose() */

void Odometry::callbackHectorPose(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackHectorPose");

  hector_pose_last_update_ = ros::Time::now();

  {
    std::scoped_lock lock(mutex_hector_);

    if (got_hector_pose_) {

      hector_pose_previous_ = hector_pose_;
      hector_pose_          = *msg;

    } else {

      hector_pose_previous_ = *msg;
      hector_pose_          = *msg;

      // Wait for hector convergence to initial position
      if (c_hector_init_msgs_++ < 10) {
        ROS_INFO("[Odometry]: Hector pose #%d - x: %f y: %f", c_hector_init_msgs_, hector_pose_.pose.position.x, hector_pose_.pose.position.y);
        return;
      }

      try {
        hector_hdg_previous_ = mrs_lib::AttitudeConverter(hector_pose_.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during getting heading (hector_hdg_previous_)");
      }


      // Reset estimator before first correction
      Vec2 zero_state = zero_state.Zero();
      Vec2 pos;
      pos << hector_pose_.pose.position.x, hector_pose_.pose.position.y;
      for (auto &estimator : _lateral_estimators_) {
        if (estimator.first == "HECTOR") {
          estimator.second->setState(0, pos);
          estimator.second->setState(1, zero_state);
          estimator.second->setState(2, zero_state);
        }
      }

      got_hector_pose_ = true;
      hector_reliable_ = true;
      return;
    }

    if (hector_reliable_) {

      // Detect jump since previous pose
      if (std::pow(hector_pose_.pose.position.x - hector_pose_previous_.pose.position.x, 2) > 4 ||
          std::pow(hector_pose_.pose.position.y - hector_pose_previous_.pose.position.y, 2) > 4) {
        ROS_WARN("[Odometry]: Jump detected in Hector Slam pose. Not reliable");

        hector_reliable_ = false;

        Vec2 pos_vec, vel_vec;
        for (auto &estimator : _lateral_estimators_) {
          if (estimator.first == "HECTOR") {
            estimator.second->getState(0, pos_vec);
            estimator.second->getState(1, vel_vec);
          }
        }

        hector_reliable_ = false;
      }

      if (current_lat_estimator_->getName() == "HECTOR") {
        Vec2 vel_vec;
        current_lat_estimator_->getState(1, vel_vec);
        if (vel_vec(0) > 5 || vel_vec(1) > 5) {
          ROS_WARN("[Odometry]: Hector Slam velocity too large - x: %f, y: %f. Not reliable.", vel_vec(0), vel_vec(1));

          hector_reliable_ = false;
        }
      }
    }


    if (c_hector_msg_ < 100) {
      c_hector_msg_++;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(hector_pose_.header.stamp.toSec(), hector_pose_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Hector pose timestamp not OK, not fusing correction.");
    return;
  }

  double hdg_hector = 0;
  {
    std::scoped_lock lock(mutex_hector_);
    try {
      hdg_hector = mrs_lib::AttitudeConverter(hector_pose_.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_WARN("[Odometry]: failed to getHeading() from hector_pose_ orientation");
    }
  }

  hdg_hector = radians::unwrap(hdg_hector, hector_hdg_previous_);
  hdg_hector += hector_offset_hdg_;
  hector_hdg_previous_ = hdg_hector;

  mrs_lib::set_mutexed(mutex_hector_, hdg_hector, hdg_hector_corr_);

  if (_debug_publish_corrections_) {
    hdg_hector = radians::wrap(hdg_hector);

    mrs_msgs::Float64Stamped hector_hdg_out;
    hector_hdg_out.header.stamp    = ros::Time::now();
    hector_hdg_out.header.frame_id = local_origin_frame_id_;
    hector_hdg_out.value           = hdg_hector;
    try {
      pub_hector_hdg_corr_.publish(hector_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_hector_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from Hector SLAM");

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Hector pose. Waiting for other sensors.");
    return;
  }

  {
    std::scoped_lock lock(mutex_hector_);

    pos_hector_x_      = hector_pose_.pose.position.x + hector_offset_(0);
    pos_hector_y_      = hector_pose_.pose.position.y + hector_offset_(1);
    hector_corr_ready_ = true;
  }

  // Set innovation variable if current estimator is HECTOR
  if (current_lat_estimator_->getName() == "HECTOR") {
    Vec2 pos_vec, innovation;
    current_lat_estimator_->getState(0, pos_vec);

    {
      std::scoped_lock lock(mutex_hector_);
      innovation(0) = pos_hector_x_ - pos_vec(0);
      innovation(1) = pos_hector_y_ - pos_vec(1);
    }

    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = innovation(0);
      odom_main_inno_.pose.pose.position.y = innovation(1);
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = 0;
      odom_main_inno_.twist.twist.linear.y = 0;
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing Hector position");
}
//}

/* //{ callbackAloamOdom() */

void Odometry::callbackAloamOdom(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackAloamOdom");

  {
    std::scoped_lock lock(mutex_aloam_);

    aloam_odom_last_update_ = ros::Time::now();

    if (got_aloam_odom_) {

      aloam_odom_previous_ = aloam_odom_;
      aloam_odom_          = *msg;

    } else {

      aloam_odom_previous_ = *msg;
      aloam_odom_          = *msg;

      try {
        aloam_hdg_previous_ = mrs_lib::AttitudeConverter(aloam_odom_.pose.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during getting heading (aloam_hdg_previous_)");
      }

      got_aloam_odom_ = true;
      aloam_reliable_ = true;
      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  auto aloam_odom_previous = mrs_lib::get_mutexed(mutex_aloam_, aloam_odom_previous_);
  auto aloam_odom          = mrs_lib::get_mutexed(mutex_aloam_, aloam_odom_);

  if (!isTimestampOK(aloam_odom_.header.stamp.toSec(), aloam_odom_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: ALOAM odom timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse aloam height //{ */

  //////////////////// Filter out aloam height measurement ////////////////////

  bool   aloam_height_ok = true;
  double measurement;
  {
    std::scoped_lock lock(mutex_aloam_);
    measurement = aloam_odom_.pose.pose.position.z;
  }
  if (isUavFlying()) {
    if (!alt_mf_aloam_->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO height easurement %f declined by median filter.", measurement);
      aloam_height_ok = false;
    }
  }

  /* //////////////////// Fuse main altitude kalman //////////////////// */

  if (!isValidGate(measurement, _aloam_min_valid_alt_, _aloam_max_valid_alt_, "aloam altitude")) {
    aloam_height_ok = false;
    return;
  }

  // Fuse vio measurement for each altitude estimator
  for (auto &estimator : _altitude_estimators_) {
    alt_x_t alt_x = alt_x.Zero();
    if (!estimator.second->getStates(alt_x)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      aloam_height_ok = false;
    }

    if (aloam_height_ok) {
      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        ros::Time        time_now = ros::Time::now();
        altitudeEstimatorCorrection(measurement, "height_aloam", estimator.second, aloam_odom_.header.stamp, time_now);
        if (fabs(measurement) > 100) {
          ROS_WARN("[Odometry]: ALOAM height correction: %f", measurement);
        }
      }
    }
  }

  //}

  /*//{ fuse aloam heading*/
  double hdg_aloam;
  try {
    hdg_aloam = mrs_lib::AttitudeConverter(aloam_odom.pose.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during setting heading (hdg_aloam)");
    return;
  }

  hdg_aloam = radians::unwrap(hdg_aloam, aloam_hdg_previous_);
  hdg_aloam += aloam_offset_hdg_;
  aloam_hdg_previous_ = hdg_aloam;

  // Apply correction step to all heading estimators
  ros::Time time_now = ros::Time::now();
  headingEstimatorsCorrection(hdg_aloam, "hdg_aloam", aloam_odom_.header.stamp, time_now);

  if (_debug_publish_corrections_) {
    hdg_aloam = radians::wrap(hdg_aloam);

    mrs_msgs::Float64Stamped aloam_hdg_out;
    aloam_hdg_out.header.stamp    = ros::Time::now();
    aloam_hdg_out.header.frame_id = local_origin_frame_id_;
    aloam_hdg_out.value           = hdg_aloam;
    try {
      pub_aloam_hdg_corr_.publish(aloam_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_aloam_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from ALOAM SLAM");

  /*//}*/

  /*//{ fuse aloam xy */
  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ALOAM odom. Waiting for other sensors.");
    return;
  }

  {
    std::scoped_lock lock(mutex_aloam_);

    pos_aloam_x_ = aloam_odom_.pose.pose.position.x + aloam_offset_(0);
    pos_aloam_y_ = aloam_odom_.pose.pose.position.y + aloam_offset_(1);

    aloam_corr_ready_ = true;
  }

  // Current orientation
  double hdg;

  if (current_hdg_estimator_->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk_);
    hdg = orientation_mavros_.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator_);

    current_hdg_estimator_->getState(0, hdg);
  }

  // Set innovation variable if current estimator is ALOAM
  if (current_lat_estimator_->getName() == "ALOAM" || current_lat_estimator_->getName() == "ALOAMGARM" || current_lat_estimator_->getName() == "ALOAMREP") {
    Vec2 pos_vec, innovation;
    current_lat_estimator_->getState(0, pos_vec);

    innovation(0) = pos_aloam_x_ - pos_vec(0);
    innovation(1) = pos_aloam_y_ - pos_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = innovation(0);
      odom_main_inno_.pose.pose.position.y = innovation(1);
      odom_main_inno_.pose.pose.position.z = 0;
      odom_main_inno_.twist.twist.linear.x = 0;
      odom_main_inno_.twist.twist.linear.y = 0;
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing ALOAM position");
  aloam_timestamp_          = aloam_odom_.header.stamp;
  aloam_updated_mapping_tf_ = true;
  /*//}*/
}
//}

/* //{ callbackLioSamOdom() */

void Odometry::callbackLioSamOdom(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackLioSamOdom");

  {
    std::scoped_lock lock(mutex_liosam_);

    liosam_odom_last_update_ = ros::Time::now();

    if (got_liosam_odom_) {

      liosam_odom_previous_ = liosam_odom_;
      liosam_odom_          = *msg;

    } else {

      liosam_odom_previous_ = *msg;
      liosam_odom_          = *msg;

      try {
        liosam_hdg_previous_ = mrs_lib::AttitudeConverter(liosam_odom_.pose.pose.orientation).getHeading();
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during getting heading (liosam_hdg_previous_)");
      }

      got_liosam_odom_ = true;
      liosam_reliable_ = true;
      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  auto liosam_odom_previous = mrs_lib::get_mutexed(mutex_liosam_, liosam_odom_previous_);
  auto liosam_odom          = mrs_lib::get_mutexed(mutex_liosam_, liosam_odom_);

  if (!isTimestampOK(liosam_odom.header.stamp.toSec(), liosam_odom_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: liosam odom timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse liosam height and height velocity //{ */

  //////////////////// Filter out liosam height measurement ////////////////////

  bool   liosam_height_ok = true;
  double height, vel_z;
  {
    std::scoped_lock lock(mutex_liosam_);
    height = liosam_odom.pose.pose.position.z;
    vel_z  = liosam_odom.twist.twist.linear.z;
  }
  if (isUavFlying()) {
    if (!alt_mf_liosam_->isValid(height)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: LIOSAM height measurement %f declined by median filter.", height);
      liosam_height_ok = false;
    }
  }

  /* //////////////////// Fuse main altitude kalman //////////////////// */

  if (!isValidGate(height, _liosam_min_valid_alt_, _liosam_max_valid_alt_, "liosam altitude")) {
    liosam_height_ok = false;
    return;
  }

  // Fuse LIOSAM height for each altitude estimator
  for (auto &estimator : _altitude_estimators_) {
    alt_x_t alt_x = alt_x.Zero();
    if (!estimator.second->getStates(alt_x)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      liosam_height_ok = false;
    }

    if (liosam_height_ok) {
      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        altitudeEstimatorCorrection(height, "height_liosam", estimator.second);
      }
    }
  }

  // Velocity
  if (liosam_height_ok) {
    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      altitudeEstimatorCorrection(vel_z, "vel_liosam_z");
    }
  }

  if (_debug_publish_corrections_) {

    mrs_msgs::Float64Stamped vel_liosam_twist_z_corr_out;
    vel_liosam_twist_z_corr_out.header.stamp    = ros::Time::now();
    vel_liosam_twist_z_corr_out.header.frame_id = local_origin_frame_id_;
    vel_liosam_twist_z_corr_out.value           = vel_z;
    try {
      pub_vel_liosam_twist_z_corr_.publish(vel_liosam_twist_z_corr_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_vel_liosam_twist_z_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing LIOSAM height and z-axis velocity.");

  //}

  if (!got_lateral_sensors_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing LIOSAM lateral odom. Waiting for other sensors.");
    return;
  }

  /*//{ fuse liosam heading*/
  double hdg_liosam, hdg_rate;
  try {
    hdg_liosam = mrs_lib::AttitudeConverter(liosam_odom.pose.pose.orientation).getHeading();
    hdg_rate   = mrs_lib::AttitudeConverter(liosam_odom.pose.pose.orientation).getHeadingRate(liosam_odom.twist.twist.angular);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during setting heading (hdg_liosam)");
    return;
  }

  hdg_liosam = radians::unwrap(hdg_liosam, liosam_hdg_previous_);
  hdg_liosam += liosam_offset_hdg_;
  liosam_hdg_previous_ = hdg_liosam;

  // Apply correction step to heading estimator
  headingEstimatorsCorrection(hdg_liosam, "hdg_liosam");

  if (std::isfinite(hdg_rate)) {
    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(hdg_rate, "rate_liosam");
    ROS_INFO_ONCE("[Odometry]: Fusing hdg rate from LIOSAM SLAM.");

  } else {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in LIOSAM angular velocity variable \"hdg_rate\", not fusing!!!");
  }

  if (_debug_publish_corrections_) {
    hdg_liosam = radians::wrap(hdg_liosam);

    mrs_msgs::Float64Stamped liosam_hdg_out;
    liosam_hdg_out.header.stamp    = ros::Time::now();
    liosam_hdg_out.header.frame_id = local_origin_frame_id_;
    liosam_hdg_out.value           = hdg_liosam;
    try {
      pub_liosam_hdg_corr_.publish(liosam_hdg_out);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_liosam_hdg_corr_.getTopic().c_str());
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing hdg from LIOSAM SLAM");

  /*//}*/

  /*//{ fuse liosam xy */

  {
    std::scoped_lock lock(mutex_liosam_);

    pos_liosam_x_ = liosam_odom.pose.pose.position.x + liosam_offset_(0);
    pos_liosam_y_ = liosam_odom.pose.pose.position.y + liosam_offset_(1);

    liosam_corr_ready_ = true;
  }

  // Current orientation
  double hdg;

  if (current_hdg_estimator_->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk_);
    hdg = orientation_mavros_.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator_);

    current_hdg_estimator_->getState(0, hdg);
  }

  // Set innoation variable if current estimator is LIOSAM
  if (current_lat_estimator_->getName() == "LIOSAM") {
    Vec2 pos_vec, innovation;
    current_lat_estimator_->getState(0, pos_vec);

    innovation(0) = pos_liosam_x_ - pos_vec(0);
    innovation(1) = pos_liosam_y_ - pos_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno_);
      odom_main_inno_.pose.pose.position.x = innovation(0);
      odom_main_inno_.pose.pose.position.y = innovation(1);
      odom_main_inno_.pose.pose.position.z = 0;
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing LIOSAM position");
  liosam_timestamp_          = liosam_odom.header.stamp;
  liosam_updated_mapping_tf_ = true;

  /*//}*/

  /* //{ fuse liosam lateral velocity */

  const double vel_liosam_x = liosam_odom.twist.twist.linear.x;
  const double vel_liosam_y = liosam_odom.twist.twist.linear.y;

  // Set innovation variable if current estimator is LIOSAM
  if (current_lat_estimator_->getName() == "LIOSAM") {
    Vec2 vel_vec, innovation;
    current_lat_estimator_->getState(1, vel_vec);

    innovation(0) = vel_liosam_x - vel_vec(0);
    innovation(1) = vel_liosam_y - vel_vec(1);

    mrs_lib::set_mutexed(mutex_odom_main_inno_, innovation(0), odom_main_inno_.twist.twist.linear.x);
    mrs_lib::set_mutexed(mutex_odom_main_inno_, innovation(1), odom_main_inno_.twist.twist.linear.y);
  }

  if (liosam_reliable_ && (vel_liosam_x > 10.0 || vel_liosam_y > 10.0)) {

    ROS_WARN("[Odometry]: Estimated liosam velocity > 10. LIOSAM is not reliable.");
    liosam_reliable_ = false;
  } else {
    // Apply correction step to all state estimators
    stateEstimatorsCorrection(vel_liosam_x, vel_liosam_y, "vel_liosam");
  }

  ROS_INFO_ONCE("[Odometry]: Fusing LIOSAM velocity");
  //}
}
//}

/* //{ callbackGarmin() */

void Odometry::callbackGarmin(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  if (!garmin_enabled_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGarmin");

  if (got_range_) {
    {
      std::scoped_lock lock(mutex_range_garmin_);
      range_garmin_previous_ = range_garmin_;
      range_garmin_          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_garmin_);
    {
      range_garmin_previous_ = *msg;
      range_garmin_          = *msg;
    }
    got_range_ = true;
  }

  auto range_garmin_tmp = mrs_lib::get_mutexed(mutex_range_garmin_, range_garmin_);

  height_active_      = true;
  garmin_last_update_ = ros::Time::now();

  if (!isTimestampOK(range_garmin_.header.stamp.toSec(), range_garmin_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Garmin range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: callbackGarmin(): No odom_pixhawk_ -> cannot untilt range measurement. Returning.");
    return;
  }

  if (!garmin_enabled_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin disabled. Returning.");
    return;
  }

  got_range_ = true;

  if (!std::isfinite(range_garmin_tmp.range)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Garmin variable \"measurement\" (garmin)!!!");

    return;
  }

  if (!isValidGate(range_garmin_tmp.range, _garmin_min_valid_alt_, _garmin_max_valid_alt_, "garmin range")) {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: garmin measurement not passed through gate: %f", range_garmin_tmp.range);
    return;
  }

  // innovation gate check
  if (_use_garmin_inno_gate_) {
    for (auto &alt_estimator : _altitude_estimators_) {
      if (alt_estimator.first == "HEIGHT") {
        double alt;
        if (!alt_estimator.second->getState(0, alt)) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
          return;
        }
        if (std::pow(range_garmin_tmp.range - alt, 2) > _garmin_inno_gate_value_sq_) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f declined by innovation gate. State value: %f. Not fusing.", range_garmin_tmp.range, alt);
          return;
        }
      }
    }
  }

  auto            odom_pixhawk_local = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_);
  Eigen::Matrix3d odom_pixhawk_R     = mrs_lib::AttitudeConverter(odom_pixhawk_local.pose.pose.orientation);

  double tilt = mrs_lib::geometry::angleBetween(Eigen::Vector3d(0, 0, 1), odom_pixhawk_R.col(2));

  // Check for excessive tilts
  // we do not want to fuse garmin with large tilts as the range will be too unreliable
  // barometer will do a better job in this situation
  bool excessive_tilt = false;
  if (std::pow(tilt, 2) > _excessive_tilt_sq_) {
    excessive_tilt = true;
  } else {
    excessive_tilt = false;
  }

  if (excessive_tilt) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Excessive tilt detected: %.2f rad. Not fusing.", tilt);
    return;
  }

  // the new way of converting "garmin's range" to "uav height"
  // Create a point in the garmin frame that correspond to the measured place
  // and then normally transform it to fcu_untilted. Later, extract the
  // negative z-component out of it.
  geometry_msgs::PoseStamped garmin_point;

  garmin_point.header           = msg->header;
  garmin_point.pose.position.x  = msg->range;
  garmin_point.pose.position.y  = 0;
  garmin_point.pose.position.z  = 0;
  garmin_point.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  auto res = transformer_.transformSingle("fcu_untilted", garmin_point);

  double measurement;

  if (res) {
    measurement = -res.value().pose.position.z;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: could not transform garmin measurement to the fcu_untilted, using fallback");
    measurement = msg->range - _garmin_z_offset_;
  }

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Garmin variable \"measurement\" (garmin)!!!");

    return;
  }


  // fuse height estimate
  lkf_height_t::z_t z;
  z << measurement;
  {
    std::scoped_lock lock(mutex_estimator_height_);

    sc_height_ = estimator_height_->correct(sc_height_, z, _R_height_);
  }

  //////////////////// Filter out garmin measurement ////////////////////
  // do not fuse garmin measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!alt_mf_garmin_->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (!garmin_enabled_) {
    ROS_WARN_ONCE("[Odometry]: Garmin not enabled. Not fusing range corrections.");
    return;
  }

  // Fuse garmin measurement for each altitude estimator
  for (auto &estimator : _altitude_estimators_) {
    alt_x_t alt_x = alt_x.Zero();
    if (!estimator.second->getStates(alt_x)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // create a correction value
    double correction;
    correction = measurement - alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);

    // saturate the correction only after switching garmin back on
    if (!std::isfinite(correction)) {
      correction = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in Garmin variable \"correction\", setting it to 0!!!");
    } else if (correction > _max_altitude_correction_) {
      correction = _max_altitude_correction_;
    } else if (correction < -_max_altitude_correction_) {
      correction = -_max_altitude_correction_;
    } else if (saturate_garmin_corrections_) {
      saturate_garmin_corrections_ = false;
      ROS_INFO("[Odometry]: Saturating garmin corrections: false");
    }

    // set the measurement vector
    double height_range;
    if (saturate_garmin_corrections_) {
      height_range = alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) + correction;
    } else {
      height_range = measurement;
    }

    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      ros::Time        time_now  = ros::Time::now();
      ros::Time        time_meas = msg->header.stamp;
      altitudeEstimatorCorrection(height_range, "height_range", estimator.second, time_meas, time_now);
      if (std::pow(height_range, 2) > 10000) {
        ROS_WARN("[Odometry]: Garmin height correction: %f", height_range);
      }
      estimator.second->getStates(alt_x);
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing Garmin rangefinder");
}

//}

/* //{ callbackSonar() */

void Odometry::callbackSonar(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackSonar");

  if (got_range_) {
    {
      std::scoped_lock lock(mutex_range_sonar_);
      range_sonar_previous_ = range_sonar_;
      range_sonar_          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_sonar_);
    {
      range_sonar_previous_ = *msg;
      range_sonar_          = *msg;
    }
    got_range_ = true;
  }

  if (!isTimestampOK(range_sonar_.header.stamp.toSec(), range_sonar_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: sonar range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk_) {
    return;
  }

  if (!sonar_enabled_) {
    return;
  }

  auto            odom_pixhawk_local = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_);
  Eigen::Matrix3d odom_pixhawk_R     = mrs_lib::AttitudeConverter(odom_pixhawk_local.pose.pose.orientation);

  double tilt = mrs_lib::geometry::angleBetween(Eigen::Vector3d(0, 0, 1), odom_pixhawk_R.col(2));

  // Check for excessive tilts
  // we do not want to fuse sonar with large tilts as the range will be too unreliable
  // barometer will do a better job in this situation
  bool excessive_tilt = false;
  if (std::pow(tilt, 2) > _excessive_tilt_sq_) {
    excessive_tilt = true;
  } else {
    excessive_tilt = false;
  }

  // the new way of converting "sonar's range" to "uav height"
  // Create a point in the sonar frame that correspond to the measured place
  // and then normally transform it to fcu_untilted. Later, extract the
  // negative z-component out of it.
  geometry_msgs::PoseStamped sonar_point;

  sonar_point.header           = range_sonar_.header;
  sonar_point.pose.position.x  = range_sonar_.range;
  sonar_point.pose.position.y  = 0;
  sonar_point.pose.position.z  = 0;
  sonar_point.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  auto res = transformer_.transformSingle("fcu_untilted", sonar_point);

  double measurement;

  if (res) {
    measurement = -res.value().pose.position.z;
  } else {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: could not transform sonar measurement to the fcu_untilted, using fallback");
    measurement = range_sonar_.range - _sonar_z_offset_;
  }

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in sonar variable \"measurement\" (sonar)!!!");

    return;
  }

  got_range_ = true;

  //////////////////// Filter out sonar measurement ////////////////////
  // do not fuse sonar measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!alt_mf_sonar_->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: sonar measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (!sonar_enabled_) {
    ROS_WARN_ONCE("[Odometry]: sonar not enabled. Not fusing range corrections.");
    return;
  }

  if (!isValidGate(measurement, _sonar_min_valid_alt_, _sonar_max_valid_alt_, "sonar range")) {
    return;
  }

  if (excessive_tilt) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Excessive tilt detected: %.2f. Not fusing sonar.", tilt);
    return;
  }

  // Fuse sonar measurement for each altitude estimator
  for (auto &estimator : _altitude_estimators_) {
    alt_x_t alt_x = alt_x.Zero();
    if (!estimator.second->getStates(alt_x)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // create a correction value
    double correction;
    correction = measurement - alt_x(mrs_msgs::AltitudeStateNames::HEIGHT);

    // saturate the correction
    if (!std::isfinite(correction)) {
      correction = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in sonar variable \"correction\", setting it to 0!!!");
    } else if (correction > _max_altitude_correction_) {
      correction = _max_altitude_correction_;
    } else if (correction < -_max_altitude_correction_) {
      correction = -_max_altitude_correction_;
    }

    // set the measurement vector
    double height_range = measurement;

    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      altitudeEstimatorCorrection(height_range, "height_sonar", estimator.second);
      if (fabs(height_range) > 100) {
        ROS_WARN("[Odometry]: sonar height correction: %f", height_range);
      }
      estimator.second->getStates(alt_x);
      if (estimator.second->getName() == "HEIGHT") {
      }
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing sonar rangefinder");
}

//}

/* //{ callbackPlane() */

void Odometry::callbackPlane(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPlane");

  if (got_plane_) {
    {
      std::scoped_lock lock(mutex_range_plane_);
      range_plane_previous_ = range_plane_;
      range_plane_          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_plane_);
    {
      range_plane_previous_ = *msg;
      range_plane_          = *msg;
    }
    got_plane_      = true;
    plane_reliable_ = true;
  }

  plane_last_update_ = ros::Time::now();

  if (!isTimestampOK(range_plane_.header.stamp.toSec(), range_plane_previous_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Plane range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk_) {
    return;
  }

  double measurement = range_plane_.range;

  //////////////////// Filter out plane measurement ////////////////////
  // do not fuse plane measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!alt_mf_plane_->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Plane measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////

  if (!isValidGate(measurement, _plane_min_valid_alt_, _plane_max_valid_alt_, "plane height")) {
    return;
  }

  // Fuse plane measurement for each altitude estimator
  for (auto &estimator : _altitude_estimators_) {
    alt_x_t alt_x = alt_x.Zero();
    if (!estimator.second->getStates(alt_x)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // set the measurement vector
    double height_range = measurement;
    {
      std::scoped_lock lock(mutex_altitude_estimator_);
      altitudeEstimatorCorrection(height_range, "height_plane", estimator.second);
      if (fabs(height_range) > 100) {
        ROS_WARN("[Odometry]: Plane height correction: %f", height_range);
        return;
      }
      estimator.second->getStates(alt_x);
      if (estimator.second->getName() == "HEIGHT") {
      }
    }
  }

  ROS_INFO_ONCE("[Odometry]: Fusing Plane rangefinder");
}

//}

/* //{ callbackPixhawkUtm() */

void Odometry::callbackPixhawkUtm(const sensor_msgs::NavSatFixConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPixhawkUtm");

  double out_x;
  double out_y;

  mrs_lib::UTM(msg->latitude, msg->longitude, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_x\"!!!");
    return;
  }

  if (!std::isfinite(out_y)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in UTM variable \"out_y\"!!!");
    return;
  }

  {
    std::scoped_lock lock(mutex_pixhawk_utm_position_);

    pixhawk_utm_position_x_ = out_x;
    pixhawk_utm_position_y_ = out_y;
  }

  got_pixhawk_utm_ = true;
  ROS_INFO_ONCE("[Odometry]: Got Pixhawk UTM.");

  nav_msgs::Odometry gps_local_odom;
  gps_local_odom.header          = msg->header;
  gps_local_odom.header.frame_id = _uav_name_ + "/gps_origin";

  // | ------------- offset the gps to local_origin ------------- |
  gps_local_odom.pose.pose.position.x = pixhawk_utm_position_x_ - _utm_origin_x_;
  gps_local_odom.pose.pose.position.y = pixhawk_utm_position_y_ - _utm_origin_y_;

  {
    std::scoped_lock lock(mutex_odom_pixhawk_);

    gps_local_odom.pose.pose.position.z  = odom_pixhawk_.pose.pose.position.z;
    gps_local_odom.pose.pose.orientation = odom_pixhawk_.pose.pose.orientation;
    gps_local_odom.twist                 = odom_pixhawk_.twist;
  }

  // | ------------- publish the gps local odometry ------------- |
  {
    std::scoped_lock lock(mutex_gps_local_odom_);

    try {
      pub_gps_local_odom_.publish(gps_local_odom);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_gps_local_odom_.getTopic().c_str());
    }
  }
}

//}

/* //{ callbackControlManagerDiag() */

void Odometry::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackControlManagerDiag");

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  control_manager_diag_     = *msg;
  got_control_manager_diag_ = true;

  if (uav_in_the_air_ && control_manager_diag.active_tracker == null_tracker_) {

    // save the current position
    Vec2 pose;
    current_lat_estimator_->getState(0, pose);
    land_position_x_   = pose[0];
    land_position_y_   = pose[1];
    land_position_set_ = true;

    uav_in_the_air_ = false;
  } else if (!uav_in_the_air_ && control_manager_diag.active_tracker != null_tracker_) {
    uav_in_the_air_ = true;
  }
}
//}

/* //{ callbackUavMassEstimate() */
void Odometry::callbackUavMassEstimate(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized_)
    return;


  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackUavMassEstimate");

  {
    std::scoped_lock lock(mutex_uav_mass_estimate_);

    _uav_mass_estimate_ = msg->data;
  }
}
//}

/* //{ callbackGPSCovariance() */
void Odometry::callbackGPSCovariance(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  if (!_gps_fallback_allowed_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGPSCovariance");

  double cov_tmp = msg->pose.covariance.at(0);

  mrs_lib::set_mutexed(mutex_gps_covariance_, cov_tmp, gps_covariance_);

  // Good/bad samples count
  if (cov_tmp > _gps_fallback_covariance_limit_ && c_gps_cov_over_lim_ < _gps_fallback_bad_samples_ + 1) {
    c_gps_cov_over_lim_++;
    c_gps_cov_ok_ = 0;
  } else if (cov_tmp < _gps_fallback_covariance_ok_ && c_gps_cov_ok_ < _gps_fallback_good_samples_ + 1) {
    c_gps_cov_ok_++;
    c_gps_cov_over_lim_ = 0;
  }

  // Fallback when GPS covariance over threshold
  if (!gps_in_fallback_ && toUppercase(current_lat_estimator_name_) == "GPS" && c_gps_cov_over_lim_ > _gps_fallback_bad_samples_) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: GPS covariance %f > %f", cov_tmp, _gps_fallback_covariance_limit_);

    std::transform(_gps_fallback_estimator_.begin(), _gps_fallback_estimator_.end(), _gps_fallback_estimator_.begin(), ::toupper);

    // Fallback to optflow
    if (_gps_fallback_estimator_ == "OPTFLOW") {

      if (optflow_active_ && got_optflow_) {
        ROS_WARN_THROTTLE(1.0, "Fallback to %s initiated.", _gps_fallback_estimator_.c_str());

        // Disable odometry service callbacks
        callbacks_enabled_ = false;

        // Call hover service
        callHover();

        // Call MpcController service
        callMpcController();

        // Disable control callbacks
        callDisableControlCallbacks();

        // Get current altitude
        alt_x_t alt_x    = alt_x.Zero();
        bool    have_alt = false;
        {
          std::scoped_lock lock(mutex_altitude_estimator_);
          if (current_alt_estimator_->getStates(alt_x)) {
            have_alt = true;
          } else {
            ROS_WARN("[Odometry]: Altitude estimator not initialized.");
            have_alt = false;
          }
        }

        if (have_alt) {
          double target_altitude = _gps_fallback_altitude_ - alt_x(0);

          // Got to optflow altitude (check if bumper enabled?)
          ROS_INFO_THROTTLE(1.0, "[Odometry]: Going %f m in z axis of fcu_untilted frame.", target_altitude);

          ROS_INFO("[Odometry]: Calling set emergency reference service.");
          mrs_msgs::ReferenceStampedSrv reference_srv;
          reference_srv.request.header.frame_id      = "fcu_untilted";
          reference_srv.request.reference.position.x = 0.0;
          reference_srv.request.reference.position.y = 0.0;
          reference_srv.request.reference.position.z = target_altitude;
          reference_srv.request.reference.heading    = 0.0;
          ser_client_reference_.call(reference_srv);
          if (reference_srv.response.success) {
            ROS_INFO("[Odometry]: Set emergency reference service called successfully: %s", reference_srv.response.message.c_str());
          } else {
            ROS_INFO("[Odometry]: Set emergency reference service call failed: %s", reference_srv.response.message.c_str());
          }

          // wait for altitude
          double t      = 0;
          double t_step = 1.0;
          ROS_INFO("[Odometry]: Waiting for reaching the target altitude.");
          while (t < _gps_fallback_wait_for_altitude_time_) {
            {
              std::scoped_lock lock(mutex_altitude_estimator_);
              current_alt_estimator_->getStates(alt_x);
            }

            ros::Duration(t_step).sleep();
            if (std::fabs(alt_x(0) - target_altitude) < 1.0) {
              break;
            }

            t += t_step;
          }
          ROS_INFO("[Odometry]: Waited %f seonds to reach the target altitude.", t);


          {
            std::scoped_lock lock(mutex_altitude_estimator_);
            if (current_alt_estimator_->getStates(alt_x)) {
              ROS_INFO_THROTTLE(1.0, "[Odometry]: Descended to %f m altitude", alt_x(0));
            } else {
              ROS_WARN("[Odometry]: Could not descend. Altitude estimator not initialized.");
            }
          }
        }

        // Change heading estimator
        bool                  hdg_switch_success;
        mrs_msgs::HeadingType desired_hdg_estimator;
        desired_hdg_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        if (changeCurrentHeadingEstimator(desired_hdg_estimator)) {
          hdg_switch_success = true;
          ROS_INFO_THROTTLE(1.0, "[Odometry]: Fallback from GPS to OPTFLOW heading estimator successful.");
        } else {
          hdg_switch_success = false;
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Fallback from GPS to OPTFLOW heading estimator failed.");
        }

        if (hdg_switch_success) {
          // Change lateral estimator
          mrs_msgs::EstimatorType desired_estimator;
          desired_estimator.type = mrs_msgs::EstimatorType::OPTFLOW;
          if (changeCurrentEstimator(desired_estimator)) {
            gps_in_fallback_ = true;
            gps_reliable_    = false;
            ROS_INFO_THROTTLE(1.0, "[Odometry]: Fallback from GPS to OPTFLOW lateral estimator successful.");
          } else {
            ROS_WARN_THROTTLE(1.0, "[Odometry]: Fallback from GPS to OPTFLOW lateral estimator failed.");
          }
        } else {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Fallback heading switch failed. Not attempting lateral switch.");
        }

        // Enable control callbacks
        callEnableControlCallbacks();

        // Enable odometry callbacks
        callbacks_enabled_ = true;
      } else {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Fallback to OPTFLOW not available.");
      }

    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Fallback from GPS allowed only to OPTFLOW.");
      return;
    }
  }


  // Back to GPS from fallback when covariance is ok
  if (gps_in_fallback_ && c_gps_cov_ok_ > _gps_fallback_good_samples_) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: GPS covariance returned to acceptable values %f < %f. Switching from fallback back to GPS.", cov_tmp,
                      _gps_fallback_covariance_ok_);

    mrs_msgs::HeadingType desired_hdg_estimator;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
    if (changeCurrentHeadingEstimator(desired_hdg_estimator)) {
      ROS_INFO_THROTTLE(1.0, "[Odometry]: Switching from fallback OPTFLOW to PIXHAWK heading successful.");
    } else {
      ROS_INFO_THROTTLE(1.0, "[Odometry]: Switching from fallback OPTFLOW to PIXHAWK heading failed.");
    }

    mrs_msgs::EstimatorType desired_estimator;
    desired_estimator.type = mrs_msgs::EstimatorType::GPS;

    if (changeCurrentEstimator(desired_estimator)) {
      gps_in_fallback_ = false;
      ROS_INFO_THROTTLE(1.0, "[Odometry]: Switching from fallback OPTFLOW to GPS successful.");
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Switching from fallback OPTFLOW to GPS failed.");
    }
    gps_reliable_ = true;
  }
}

//}

/* //{ callbackGroundTruth() */
void Odometry::callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGroundTruth");

  {
    std::scoped_lock lock(mutex_ground_truth_);

    ground_truth_ = *msg;
  }

  double rot_x, rot_y, rot_z;
  getGlobalRot(ground_truth_.pose.pose.orientation, rot_x, rot_y, rot_z);

  orientation_gt_.header   = odom_pixhawk_.header;
  orientation_gt_.vector.x = rot_x;
  orientation_gt_.vector.y = rot_y;
  orientation_gt_.vector.z = rot_z;

  try {
    pub_orientation_gt_.publish(orientation_gt_);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_orientation_gt_.getTopic().c_str());
  }
}
//}

/* //{ callbackT265Odometry() */
void Odometry::callbackT265Odometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackT265Odometry");

  if (got_odom_t265_) {

    {
      std::scoped_lock lock(mutex_odom_t265_);

      odom_t265_previous_ = odom_t265_;
      odom_t265_          = *msg;
    }

  } else {

    {
      std::scoped_lock lock(mutex_odom_t265_);

      odom_t265_previous_ = *msg;
      odom_t265_          = *msg;
    }

    got_odom_t265_         = true;
    t265_reliable_         = true;
    odom_t265_last_update_ = ros::Time::now();
    return;
  }

  odom_t265_last_update_ = ros::Time::now();

  if (!isTimestampOK(odom_t265_.header.stamp.toSec(), odom_t265_.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: T265 odom timestamp not OK.");
  }

  double dt;
  {
    std::scoped_lock lock(mutex_odom_t265_);

    dt = (odom_t265_.header.stamp - odom_pixhawk_previous_.header.stamp).toSec();
  }

  if (!got_init_heading_) {

    auto odom_t265_local = mrs_lib::get_mutexed(mutex_odom_t265_, odom_t265_);
    try {
      init_heading_ = mrs_lib::AttitudeConverter(odom_t265_local.pose.pose.orientation).getHeading();
    }
    catch (...) {
      ROS_WARN("[Odometry]: failed to getHeading() from odom_t265_local");
    }

    got_init_heading_ = true;
  }

  if (!got_range_ && garmin_enabled_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for rangefinder.");
    return;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  /* check NaNs //{ */
  if (t265_reliable_ && !std::isfinite(odom_t265_.pose.pose.position.x)) {
    odom_t265_.pose.pose.position.x = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265_.pose.pose.position.x\", T265 odom is now unreliable!!!");
    t265_reliable_ = false;
    return;
  }

  if (t265_reliable_ && !std::isfinite(odom_t265_.pose.pose.position.y)) {
    odom_t265_.pose.pose.position.y = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265_.pose.pose.position.y\", T265 odom is now unreliable!!!");
    t265_reliable_ = false;
    return;
  }

  if (t265_reliable_ && !std::isfinite(odom_t265_.pose.pose.position.z)) {
    odom_t265_.pose.pose.position.z = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265_.pose.pose.position.z\", T265 odom is now unreliable!!!");
    t265_reliable_ = false;
    return;
  }
  //}

  /* check maximum velocity //{ */

  double vel_x, vel_y, vel_z;
  {
    std::scoped_lock lock(mutex_odom_t265_);
    vel_x = (odom_t265_.pose.pose.position.x - odom_t265_previous_.pose.pose.position.x) / dt;
    vel_y = (odom_t265_.pose.pose.position.y - odom_t265_previous_.pose.pose.position.y) / dt;
    vel_z = (odom_t265_.pose.pose.position.z - odom_t265_previous_.pose.pose.position.z) / dt;
  }

  if (vel_x > _max_t265_vel_ || vel_y > _max_t265_vel_ || vel_z > _max_t265_vel_) {
    t265_reliable_ = false;
    ROS_WARN_THROTTLE(1.0, "[Odometry]: T265 velocity: x: %f, y: %f, z: %f, exceeded %f m/s", vel_x, vel_y, vel_z, _max_t265_vel_);
    return;
  }

  //}

  /* republish t265 odometry //{ */

  if (toUppercase(current_lat_estimator_name_) == "T265") {

    /* publish t265 altitude //{ */
    mrs_msgs::Float64Stamped new_altitude;
    {
      std::scoped_lock lock(mutex_odom_pixhawk_);

      new_altitude.header = odom_t265_.header;
      new_altitude.value  = odom_t265_.pose.pose.position.z;
    }

    new_altitude.header.frame_id = local_origin_frame_id_;
    new_altitude.header.stamp    = ros::Time::now();

    try {
      pub_altitude_.publish(new_altitude);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_altitude_.getTopic().c_str());
    }
    //}

    /* publish t265 orientation  //{ */

    nav_msgs::Odometry orientation;
    {
      std::scoped_lock lock(mutex_odom_t265_);
      orientation.header                = odom_t265_.header;
      orientation.pose.pose.orientation = odom_t265_.pose.pose.orientation;
    }
    orientation.header.frame_id = local_origin_frame_id_;

    try {
      pub_orientation_.publish(orientation);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_orientation_.getTopic().c_str());
    }

    //}

    /* publish t265 odometry //{ */
    nav_msgs::Odometry odom_main;
    {
      std::scoped_lock lock(mutex_odom_t265_);
      odom_main = odom_t265_;
    }

    odom_main.header.frame_id = _uav_name_ + "/" + toLowercase(current_lat_estimator_name_) + "_origin";
    odom_main.child_frame_id  = fcu_frame_id_;
    odom_main.header.stamp    = ros::Time::now();

    if (!odometry_published_) {
      std::scoped_lock lock(mutex_odom_local_);
      odom_local_                         = odom_main;
      odom_local_.pose.pose.orientation.x = 0.0;
      odom_local_.pose.pose.orientation.y = 0.0;
      odom_local_.pose.pose.orientation.z = 0.0;
      odom_local_.pose.pose.orientation.w = 1.0;
      pos_odom_offset_.setZero();
      rot_odom_offset_ = mrs_lib::AttitudeConverter(0, 0, 0);
      rot_odom_offset_.normalize();
      last_stable_name_ = odom_main.header.frame_id;
      last_local_name_  = odom_main.header.frame_id;
    }
    {
      std::scoped_lock lock(mutex_odom_local_);
      if (odom_main.header.frame_id != last_local_name_) {

        last_local_name_ = odom_main.header.frame_id;

        tf2::Vector3 v1, v2;
        tf2::fromMsg(odom_main.pose.pose.position, v1);
        tf2::fromMsg(odom_local_.pose.pose.position, v2);
        tf2::Vector3 pos_diff = v1 - v2;
        pos_odom_offset_      = pos_diff;

        if (odom_local_.pose.pose.orientation.w == 0.0) {
          odom_local_.pose.pose.orientation = odom_pixhawk_.pose.pose.orientation;
        }
        tf2::Quaternion q1       = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation);
        tf2::Quaternion q2       = mrs_lib::AttitudeConverter(odom_local_.pose.pose.orientation);
        tf2::Quaternion rot_diff = q2 * q1.inverse();
        rot_odom_offset_         = rot_diff;
        rot_odom_offset_.normalize();
        ROS_WARN("[Odometry]: Changed odometry estimator. Updating offset for stable odometry.");
      }

      odom_local_                 = applyOdomOffset(odom_main, pos_odom_offset_, rot_odom_offset_);
      odom_local_.header.frame_id = local_origin_frame_id_;

      try {
        pub_odom_local_.publish(odom_local_);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_local_.getTopic().c_str());
      }
    }

    {
      std::scoped_lock lock(mutex_shared_odometry_);

      shared_odom_ = odom_main;
    }

    try {
      pub_odom_main_.publish(odom_main);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_main_.getTopic().c_str());
    }
    ROS_INFO_ONCE("[Odometry]: Publishing odometry");


    //}
  }
  //}
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackOdometrySource() */

bool Odometry::callbackChangeOdometrySource(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  mrs_msgs::EstimatorType desired_estimator;
  mrs_msgs::HeadingType   desired_hdg_estimator;
  mrs_msgs::AltitudeType  desired_alt_estimator;


  std::string type = req.value;
  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  if (type == "OPTFLOW") {
    desired_estimator.type     = mrs_msgs::EstimatorType::OPTFLOW;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "GPS") {
    desired_estimator.type     = mrs_msgs::EstimatorType::GPS;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "OPTFLOWGPS") {
    desired_estimator.type     = mrs_msgs::EstimatorType::OPTFLOWGPS;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "RTK") {
    desired_estimator.type     = mrs_msgs::EstimatorType::RTK;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
    if (_use_rtk_altitude_) {
      desired_alt_estimator.type = mrs_msgs::AltitudeType::RTK;
    } else {
      desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
    }
  } else if (type == "VIO") {
    desired_estimator.type     = mrs_msgs::EstimatorType::VIO;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::VIO;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::VIO;
  } else if (type == "VSLAM") {
    desired_estimator.type     = mrs_msgs::EstimatorType::VSLAM;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::VSLAM;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "BRICK") {
    desired_estimator.type     = mrs_msgs::EstimatorType::BRICK;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::BRICK;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::PLANE;
  } else if (type == "T265") {
    desired_estimator.type     = mrs_msgs::EstimatorType::T265;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "HECTOR") {
    desired_estimator.type     = mrs_msgs::EstimatorType::HECTOR;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::HECTOR;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "ALOAM") {
    desired_estimator.type     = mrs_msgs::EstimatorType::ALOAM;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::ALOAM;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::ALOAM;
  } else if (type == "BRICKFLOW") {
    desired_estimator.type     = mrs_msgs::EstimatorType::BRICKFLOW;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::BRICK;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::BRICK;
  } else if (type == "ICP") {
    desired_estimator.type     = mrs_msgs::EstimatorType::ICP;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::ICP;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "ALOAMGARM") {
    desired_estimator.type     = mrs_msgs::EstimatorType::ALOAMGARM;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::ALOAMREP;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::ALOAMGARM;
  } else if (type == "ALOAMREP") {
    desired_estimator.type     = mrs_msgs::EstimatorType::ALOAMREP;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::ALOAMREP;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::ALOAMREP;
  } else if (type == "LIOSAM") {
    desired_estimator.type     = mrs_msgs::EstimatorType::LIOSAM;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::LIOSAM;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::LIOSAM;
  } else {
    ROS_WARN("[Odometry]: Invalid type %s requested", type.c_str());
    res.success = false;
    res.message = ("Not a valid odometry type");
    return true;
  }

  // Check whether a valid hdg type was requested
  if (!isValidType(desired_hdg_estimator)) {
    ROS_ERROR("[Odometry]: %d is not a valid heading estimator type", desired_hdg_estimator.type);
    res.success = false;
    res.message = ("Not a valid heading estimator type");
    return true;
  }

  desired_hdg_estimator.name = _heading_estimators_names_[desired_hdg_estimator.type];

  bool success_hdg = false;

  if (toUppercase(current_hdg_estimator_name_) == toUppercase(desired_hdg_estimator.name)) {

    success_hdg = true;
    ROS_INFO("[Odometry]: Heading estimator %s already active.", desired_hdg_estimator.name.c_str());

  } else {

    std::scoped_lock lock(mutex_hdg_estimator_type_);

    success_hdg = changeCurrentHeadingEstimator(desired_hdg_estimator);
  }

  desired_estimator.name = _state_estimators_names_[desired_estimator.type];

  bool success = false;

  if (toUppercase(current_lat_estimator_name_) == toUppercase(desired_estimator.name)) {

    success = true;
    ROS_INFO("[Odometry]: Lateral estimator %s already active.", desired_estimator.name.c_str());

  } else {

    std::scoped_lock lock(mutex_estimator_type_);

    success = changeCurrentEstimator(desired_estimator);
  }

  // Check whether a valid altitude type was requested
  if (!isValidType(desired_alt_estimator)) {
    ROS_ERROR("[Odometry]: %d is not a valid altitude estimator type", desired_alt_estimator.type);
    res.success = false;
    res.message = ("Not a valid altitude estimator type");
    return true;
  }

  desired_alt_estimator.name = _altitude_estimators_names_[desired_alt_estimator.type];

  bool success_alt = false;
  if (toUppercase(current_alt_estimator_name_) == toUppercase(desired_alt_estimator.name)) {

    success_alt = true;
    ROS_INFO("[Odometry]: Altitude estimator %s already active.", desired_alt_estimator.name.c_str());

  } else {
    std::scoped_lock lock(mutex_alt_estimator_type_);

    success_alt = changeCurrentAltitudeEstimator(desired_alt_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success_alt && success_hdg && success;
  res.message = (printOdometryDiag().c_str());

  return true;
}

//}

/* //{ callbackChangeEstimator() */

bool Odometry::callbackChangeEstimator(mrs_msgs::ChangeEstimator::Request &req, mrs_msgs::ChangeEstimator::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(req.estimator_type)) {
    ROS_ERROR("[Odometry]: %d is not a valid odometry type", req.estimator_type.type);
    res.success = false;
    res.message = ("Not a valid odometry type");
    {
      std::scoped_lock lock(mutex_estimator_type_);

      res.estimator_type.type = estimator_type_.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_estimator_type_);

    mrs_msgs::EstimatorType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _state_estimators_names_[desired_estimator.type];
    success                = changeCurrentEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_estimator_type_);

    res.estimator_type.type = estimator_type_.type;
  }

  return true;
}

//}

/* //{ callbackChangeEstimatorString() */

bool Odometry::callbackChangeEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  mrs_msgs::EstimatorType desired_estimator;

  std::string type = req.value;
  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  if (type == "OPTFLOW") {
    desired_estimator.type = mrs_msgs::EstimatorType::OPTFLOW;
  } else if (type == "GPS") {
    desired_estimator.type = mrs_msgs::EstimatorType::GPS;
  } else if (type == "OPTFLOWGPS") {
    desired_estimator.type = mrs_msgs::EstimatorType::OPTFLOWGPS;
  } else if (type == "RTK") {
    desired_estimator.type = mrs_msgs::EstimatorType::RTK;
  } else if (type == "VIO") {
    desired_estimator.type = mrs_msgs::EstimatorType::VIO;
  } else if (type == "VSLAM") {
    desired_estimator.type = mrs_msgs::EstimatorType::VSLAM;
  } else if (type == "BRICK") {
    desired_estimator.type = mrs_msgs::EstimatorType::BRICK;
  } else if (type == "T265") {
    desired_estimator.type = mrs_msgs::EstimatorType::T265;
  } else if (type == "HECTOR") {
    desired_estimator.type = mrs_msgs::EstimatorType::HECTOR;
  } else if (type == "ALOAM") {
    desired_estimator.type = mrs_msgs::EstimatorType::ALOAM;
  } else if (type == "ALOAMGARM") {
    desired_estimator.type = mrs_msgs::EstimatorType::ALOAMGARM;
  } else if (type == "ALOAMREP") {
    desired_estimator.type = mrs_msgs::EstimatorType::ALOAMREP;
  } else if (type == "BRICKFLOW") {
    desired_estimator.type = mrs_msgs::EstimatorType::BRICKFLOW;
  } else if (type == "ICP") {
    desired_estimator.type = mrs_msgs::EstimatorType::ICP;
  } else if (type == "LIOSAM") {
    desired_estimator.type = mrs_msgs::EstimatorType::LIOSAM;
  } else {
    ROS_WARN("[Odometry]: Invalid type %s requested", type.c_str());
    res.success = false;
    res.message = ("Not a valid odometry type");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(desired_estimator)) {
    ROS_ERROR("[Odometry]: %d is not a valid odometry type", desired_estimator.type);
    res.success = false;
    res.message = ("Not a valid odometry type");
    return true;
  }

  bool success           = false;
  desired_estimator.name = _state_estimators_names_[desired_estimator.type];

  if (toUppercase(current_lat_estimator_name_) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Lateral estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  {
    std::scoped_lock lock(mutex_estimator_type_);

    success = changeCurrentEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());

  return true;
}  // namespace mrs_uav_odometry

//}

/* //{ callbackChangeHdgEstimator() */

bool Odometry::callbackChangeHdgEstimator(mrs_msgs::ChangeHdgEstimator::Request &req, mrs_msgs::ChangeHdgEstimator::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(req.estimator_type)) {
    ROS_ERROR("[Odometry]: %d is not a valid heading estimator type", req.estimator_type.type);
    res.success = false;
    res.message = ("Not a valid heading estimator type");
    {
      std::scoped_lock lock(mutex_hdg_estimator_type_);

      res.estimator_type.type = hdg_estimator_type_.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_hdg_estimator_type_);

    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    success                = changeCurrentHeadingEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_hdg_estimator_type_);

    res.estimator_type.type = hdg_estimator_type_.type;
  }

  return true;
}

//}

/* //{ callbackChangeHdgEstimatorString() */

bool Odometry::callbackChangeHdgEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  mrs_msgs::HeadingType desired_estimator;

  std::string type = req.value;
  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  if (type == "PIXHAWK") {
    desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
  } else if (type == "GYRO") {
    desired_estimator.type = mrs_msgs::HeadingType::GYRO;
  } else if (type == "COMPASS") {
    desired_estimator.type = mrs_msgs::HeadingType::COMPASS;
  } else if (type == "OPTFLOW") {
    desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
  } else if (type == "HECTOR") {
    desired_estimator.type = mrs_msgs::HeadingType::HECTOR;
  } else if (type == "ALOAM") {
    desired_estimator.type = mrs_msgs::HeadingType::ALOAM;
  } else if (type == "ALOAMREP") {
    desired_estimator.type = mrs_msgs::HeadingType::ALOAMREP;
  } else if (type == "BRICK") {
    desired_estimator.type = mrs_msgs::HeadingType::BRICK;
  } else if (type == "VIO") {
    desired_estimator.type = mrs_msgs::HeadingType::VIO;
  } else if (type == "VSLAM") {
    desired_estimator.type = mrs_msgs::HeadingType::VSLAM;
  } else if (type == "ICP") {
    desired_estimator.type = mrs_msgs::HeadingType::ICP;
  } else if (type == "BRICKFLOW") {
    desired_estimator.type = mrs_msgs::HeadingType::BRICKFLOW;
  } else if (type == "LIOSAM") {
    desired_estimator.type = mrs_msgs::HeadingType::LIOSAM;
  } else {
    ROS_WARN("[Odometry]: Invalid type %s requested", type.c_str());
    res.success = false;
    res.message = ("Not a valid heading estimator type");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(desired_estimator)) {
    ROS_ERROR("[Odometry]: %d is not a valid heading estimator type", desired_estimator.type);
    res.success = false;
    res.message = ("Not a valid heading estimator type");
    return true;
  }

  desired_estimator.name = _heading_estimators_names_[desired_estimator.type];

  if (toUppercase(current_hdg_estimator_name_) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Heading estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_hdg_estimator_type_);

    success = changeCurrentHeadingEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());

  return true;
}  // namespace mrs_uav_odometry

//}

/* //{ callbackChangeAltEstimator() */

bool Odometry::callbackChangeAltEstimator(mrs_msgs::ChangeAltEstimator::Request &req, mrs_msgs::ChangeAltEstimator::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(req.estimator_type)) {
    ROS_ERROR("[Odometry]: %d is not a valid altitude estimator type", req.estimator_type.type);
    res.success = false;
    res.message = ("Not a valid altitude estimator type");
    {
      std::scoped_lock lock(mutex_alt_estimator_type_);

      res.estimator_type.type = alt_estimator_type_.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_alt_estimator_type_);

    mrs_msgs::AltitudeType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _altitude_estimators_names_[desired_estimator.type];
    success                = changeCurrentAltitudeEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_alt_estimator_type_);

    res.estimator_type.type = alt_estimator_type_.type;
  }

  return true;
}

//}

/* //{ callbackChangeAltEstimatorString() */

bool Odometry::callbackChangeAltEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  mrs_msgs::AltitudeType desired_estimator;

  std::string type = req.value;
  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  if (type == "HEIGHT") {
    desired_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "PLANE") {
    desired_estimator.type = mrs_msgs::AltitudeType::PLANE;
  } else if (type == "BRICK") {
    desired_estimator.type = mrs_msgs::AltitudeType::BRICK;
  } else if (type == "VIO") {
    desired_estimator.type = mrs_msgs::AltitudeType::VIO;
  } else if (type == "ALOAM") {
    desired_estimator.type = mrs_msgs::AltitudeType::ALOAM;
  } else if (type == "ALOAMGARM") {
    desired_estimator.type = mrs_msgs::AltitudeType::ALOAMGARM;
  } else if (type == "ALOAMREP") {
    desired_estimator.type = mrs_msgs::AltitudeType::ALOAMREP;
  } else if (type == "BARO") {
    desired_estimator.type = mrs_msgs::AltitudeType::BARO;
  } else if (type == "RTK") {
    desired_estimator.type = mrs_msgs::AltitudeType::RTK;
  } else if (type == "LIOSAM") {
    desired_estimator.type = mrs_msgs::AltitudeType::LIOSAM;
  } else {
    ROS_WARN("[Odometry]: Invalid type %s requested", type.c_str());
    res.success = false;
    res.message = ("Not a valid altitude estimator type");
    return true;
  }

  // Check whether a valid type was requested
  if (!isValidType(desired_estimator)) {
    ROS_ERROR("[Odometry]: %d is not a valid altitude estimator type", desired_estimator.type);
    res.success = false;
    res.message = ("Not a valid altitude estimator type");
    return true;
  }

  desired_estimator.name = _altitude_estimators_names_[desired_estimator.type];

  if (toUppercase(current_alt_estimator_name_) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Altitude estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_alt_estimator_type_);

    success = changeCurrentAltitudeEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());

  return true;
}

//}

/* //{ callbackToggleGarmin() */

bool Odometry::callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  garmin_enabled_ = req.data;

  // after enabling garmin we want to start correcting the altitude slowly
  if (garmin_enabled_) {
    saturate_garmin_corrections_ = true;
    ROS_INFO("[Odometry]: Saturating garmin corrections: true");
  }

  res.success = true;
  res.message = (garmin_enabled_ ? "Garmin enabled" : "Garmin disabled");

  if (garmin_enabled_) {

    ROS_INFO("[Odometry]: Garmin enabled.");
    rtk_altitude_enabled_ = false;

  } else {

    ROS_INFO("[Odometry]: Garmin disabled");
  }

  return true;
}

//}

/* //{ callbackResetEstimator() */

bool Odometry::callbackResetEstimator([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  LatState2D states;
  bool       success = false;

  // reset lateral kalman x
  {
    std::scoped_lock lock(mutex_current_lat_estimator_);
    success = current_lat_estimator_->getStates(states);
  }

  if (estimator_type_.type == mrs_msgs::EstimatorType::GPS || estimator_type_.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
      estimator_type_.type == mrs_msgs::EstimatorType::RTK) {

    states(0, 0) = odom_pixhawk_shifted_.pose.pose.position.x;
    states(0, 1) = odom_pixhawk_shifted_.pose.pose.position.y;

  } else {
    // TODO there is a bug: when taking off, the position is set to local_origin instead of current pixhawk odom
    if (!land_position_set_) {  // if taking off for the first time

      if (estimator_type_.type == mrs_msgs::EstimatorType::GPS || estimator_type_.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
          estimator_type_.type == mrs_msgs::EstimatorType::RTK) {
        if (!calculatePixhawkOdomOffset()) {
          ROS_ERROR("[Odometry]: Calculating pixhawk odom offset failed");
        }
        states(0, 0) = odom_pixhawk_shifted_.pose.pose.position.x;
        states(0, 1) = odom_pixhawk_shifted_.pose.pose.position.y;
        ROS_INFO("[Odometry]: Resetting estimators to pijhawk shifted odom x: %f y: %f", states(0, 0), states(0, 1));
      } else {
        states(0, 0) = _init_gps_offset_x_;
        states(0, 1) = _init_gps_offset_y_;
        ROS_INFO("[Odometry]: Resetting estimators to local_origin x: %f y: %f", states(0, 0), states(0, 1));
      }

    } else {  // taking off again
      if (estimator_type_.type == mrs_msgs::EstimatorType::GPS || estimator_type_.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
          estimator_type_.type == mrs_msgs::EstimatorType::RTK) {
        if (!calculatePixhawkOdomOffset()) {
          ROS_ERROR("[Odometry]: Calculating pixhawk odom offset failed");
        }
        states(0, 0) = odom_pixhawk_shifted_.pose.pose.position.x;
        states(0, 1) = odom_pixhawk_shifted_.pose.pose.position.y;
        ROS_INFO("[Odometry]: Resetting estimators to pixhawk shifted odom x: %f y: %f", states(0, 0), states(0, 1));
      } else {
        states(0, 0) = land_position_x_;
        states(0, 1) = land_position_y_;
        ROS_INFO("[Odometry]: Resetting estimators to land position x: %f y: %f", states(0, 0), states(0, 1));
      }
    }
  }

  if (!success) {

    ROS_ERROR("[Odometry]: Lateral kalman states and covariance reset failed #1.");

    res.success = false;
    res.message = "Reset of lateral kalman failed";

    return true;
  }

  states(1, 0) = 0.0;
  states(2, 0) = 0.0;
  states(1, 1) = 0.0;
  states(2, 1) = 0.0;

  {
    std::scoped_lock lock(mutex_current_lat_estimator_);
    success = current_lat_estimator_->reset(states);
  }

  if (!success) {

    ROS_ERROR("[Odometry]: Lateral kalman states and covariance reset failed #2.");

    res.success = false;
    res.message = "Reset of lateral kalman failed";

    return true;
  }

  ROS_WARN("[Odometry]: Lateral kalman states and covariance reset.");

  res.success = true;
  res.message = "Reset of lateral kalman successful";

  return true;
}
//}

/* //{ callbackResetHector() */

bool Odometry::callbackResetHector([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  if (toUppercase(current_lat_estimator_name_) == "HECTOR" || toUppercase(current_hdg_estimator_name_) == "HECTOR") {
    res.success = false;
    res.message = ("Cannot reset when HECTOR in feedback");
    ROS_WARN("[Odometry]: Cannot switch to HECTOR, when HECTOR is in feedback.");
    return true;
  }

  if (!got_hector_pose_) {
    res.success = false;
    res.message = ("Cannot reset when HECTOR is not running");
    ROS_WARN("[Odometry]: Cannot switch to HECTOR when HECTOR is not running.");
    return true;
  }

  // Reset HECTOR map
  ROS_INFO("[Odometry]: Calling Hector map reset.");
  std_msgs::String reset_msg;
  reset_msg.data = "reset";
  try {
    pub_hector_reset_.publish(reset_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_hector_reset_.getTopic().c_str());
  }
  hector_reset_called_ = true;
  ROS_INFO("[Odometry]: Hector map reset called.");

  // wait for reset - workaroung for non-blocking map reset by toopic
  double t      = 0;
  double t_step = 0.1;
  ROS_INFO("[Odometry]: Waiting for reset of hector map.");
  while (t < 1.0) {
    geometry_msgs::Pose hector_pose_tmp = mrs_lib::get_mutexed(mutex_hector_, hector_pose_.pose);
    ros::Duration(t_step).sleep();
    if (std::fabs(hector_pose_tmp.position.x) < 1.0 && std::fabs(hector_pose_tmp.position.y) < 1.0) {
      break;
    }

    t += t_step;
  }

  ROS_INFO("[Odometry]: Waited %f seconds for hector map reset.", t);

  // Reset HECTOR heading
  for (auto &estimator : heading_estimators_) {
    if (estimator.first == "HECTOR") {
      estimator.second->setState(0, 0);
      estimator.second->setState(1, 0);
      estimator.second->setState(2, 0);
    }
  }

  // Reset HECTOR position
  for (auto &estimator : _lateral_estimators_) {
    if (estimator.first == "HECTOR") {
      Vec2 pos_vec, vel_vec, acc_vec;
      pos_vec << 0, 0;
      vel_vec << 0, 0;
      acc_vec << 0, 0;
      estimator.second->setState(0, pos_vec);
      estimator.second->setState(1, vel_vec);
      estimator.second->setState(2, acc_vec);
    }
  }

  ROS_WARN("[Odometry]: Hector estimator states reset.");

  res.success = true;
  res.message = "Reset of Hector estimator successful";

  return true;
}
//}

/* //{ callbackReliableHector() */

bool Odometry::callbackReliableHector([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  hector_reliable_ = true;

  ROS_WARN("[Odometry]: Hector manually set to reliable.");

  res.success = true;
  res.message = "Hector manually set to reliable";

  return true;
}
//}

/* //{ callbackToggleCallbacks() */

bool Odometry::callbackToggleCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized_)
    return false;

  callbacks_enabled_ = req.data;

  res.success = true;
  res.message = (callbacks_enabled_ ? "Callbacks enabled" : "Callbacks disabled");

  if (callbacks_enabled_) {

    ROS_INFO("[Odometry]: Callbacks enabled.");

  } else {

    ROS_INFO("[Odometry]: Callbacks disabled");
  }

  return true;
}

//}

/* //{ callbackReconfigure() */
void Odometry::callbackReconfigure([[maybe_unused]] mrs_uav_odometry::odometry_dynparamConfig &config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  if (!callbacks_enabled_) {
    return;
  }

  ROS_INFO(
      "Reconfigure Request:\n"
      "Lateral measurement covariance:\n"
      "\nPosition:\n"
      "R_pos_mavros: %f\n"
      "R_pos_vio: %f\n"
      "R_pos_vslam: %f\n"
      "R_pos_rtk: %f\n"
      "R_pos_brick: %f\n"
      "R_pos_hector: %f\n"

      "\nVelocity:\n"
      "R_vel_mavros: %f\n"
      "R_vel_vio: %f\n"
      "R_vel_icp: %f\n"
      "R_vel_optflow: %f\n"
      "R_vel_rtk: %f\n"

      "\nAcceleration:\n"
      "R_acc_imu: %f\n",

      config.R_pos_mavros, config.R_pos_vio, config.R_pos_vslam, config.R_pos_rtk, config.R_pos_brick, config.R_pos_hector, config.R_vel_mavros,
      config.R_vel_vio, config.R_vel_icp, config.R_vel_optflow, config.R_vel_rtk, config.R_acc_imu_lat);

  for (auto &estimator : _lateral_estimators_) {
    estimator.second->setR(config.R_pos_mavros, map_measurement_name_id_.find("pos_mavros")->second);
    estimator.second->setR(config.R_pos_vio, map_measurement_name_id_.find("pos_vio")->second);
    estimator.second->setR(config.R_pos_vslam, map_measurement_name_id_.find("pos_vslam")->second);
    estimator.second->setR(config.R_pos_rtk, map_measurement_name_id_.find("pos_rtk")->second);
    estimator.second->setR(config.R_pos_brick, map_measurement_name_id_.find("pos_brick")->second);
    estimator.second->setR(config.R_pos_hector, map_measurement_name_id_.find("pos_hector")->second);
    estimator.second->setR(config.R_pos_aloam, map_measurement_name_id_.find("pos_aloam")->second);
    estimator.second->setR(config.R_pos_liosam, map_measurement_name_id_.find("pos_liosam")->second);

    estimator.second->setR(config.R_vel_mavros, map_measurement_name_id_.find("vel_mavros")->second);
    estimator.second->setR(config.R_vel_vio, map_measurement_name_id_.find("vel_vio")->second);
    estimator.second->setR(config.R_vel_icp, map_measurement_name_id_.find("vel_icp")->second);
    estimator.second->setR(config.R_vel_optflow, map_measurement_name_id_.find("vel_optflow")->second);
    estimator.second->setR(config.R_vel_rtk, map_measurement_name_id_.find("vel_rtk")->second);

    estimator.second->setR(config.R_acc_imu_lat, map_measurement_name_id_.find("acc_imu")->second);

    ROS_INFO(
        "Lateral process covariance:\n"
        "Position (0,0): %f\n"
        "Velocity (1,1): %f\n"
        "Acceleration (2,2): %f\n",
        config.Q_lat_pos, config.Q_lat_vel, config.Q_lat_acc);

    estimator.second->setQ(config.Q_lat_pos, Eigen::Vector2i(0, 0));
    estimator.second->setQ(config.Q_lat_vel, Eigen::Vector2i(1, 1));
    estimator.second->setQ(config.Q_lat_acc, Eigen::Vector2i(2, 2));
  }

  ROS_INFO(
      "Altitude measurement covariance:\n"
      "R_height_range: %f\n"
      "R_height_plane: %f\n"
      "R_height_brick: %f\n"
      "R_height_aloam: %f\n"
      "R_height_liosam: %f\n"
      "R_height_baro: %f\n"
      "R_vel_baro: %f\n"
      "R_acc_imu: %f\n",
      config.R_height_range, config.R_height_plane, config.R_height_brick, config.R_height_aloam, config.R_height_liosam, config.R_height_baro,
      config.R_vel_baro, config.R_acc_imu);

  for (auto &estimator : _altitude_estimators_) {
    estimator.second->setR(config.R_height_range, map_alt_measurement_name_id_.find("height_range")->second);
    estimator.second->setR(config.R_height_plane, map_alt_measurement_name_id_.find("height_plane")->second);
    estimator.second->setR(config.R_height_brick, map_alt_measurement_name_id_.find("height_brick")->second);
    estimator.second->setR(config.R_height_aloam, map_alt_measurement_name_id_.find("height_aloam")->second);
    estimator.second->setR(config.R_height_liosam, map_alt_measurement_name_id_.find("height_liosam")->second);
    estimator.second->setR(config.R_height_baro, map_alt_measurement_name_id_.find("height_baro")->second);
    estimator.second->setR(config.R_vel_baro, map_alt_measurement_name_id_.find("vel_baro")->second);
    estimator.second->setR(config.R_acc_imu, map_alt_measurement_name_id_.find("acc_imu")->second);

    ROS_INFO(
        "Altitude process covariance:\n"
        "Position (0,0): %f\n"
        "Velocity (1,1): %f\n"
        "Acceleration (2,2): %f\n",
        config.Q_alt_pos, config.Q_alt_vel, config.Q_alt_acc);

    estimator.second->setQ(config.Q_alt_pos, Eigen::Vector2i(0, 0));
    estimator.second->setQ(config.Q_alt_vel, Eigen::Vector2i(1, 1));
    estimator.second->setQ(config.Q_alt_acc, Eigen::Vector2i(2, 2));

    ROS_INFO("Altitude input coefficient: %f\n", config.alt_input_coeff);
    estimator.second->setInputCoeff(config.alt_input_coeff);
  }

  ROS_INFO(
      "Heading measurement covariance:\n"
      "R_hdg_compass: %f\n"
      "R_hdg_hector: %f\n"
      "R_hdg_aloam: %f\n"
      "R_hdg_liosam: %f\n"
      "R_hdg_brick: %f\n"
      "R_hdg_vio: %f\n"
      "R_hdg_vslam: %f\n"
      "R_rate_gyro: %f\n"
      "R_rate_optflow: %f\n"
      "R_rate_icp: %f\n",
      config.R_hdg_compass, config.R_hdg_hector, config.R_hdg_aloam, config.R_hdg_liosam, config.R_hdg_brick, config.R_hdg_vio, config.R_hdg_vslam,
      config.R_rate_gyro, config.R_rate_optflow, config.R_rate_icp);

  for (auto &estimator : heading_estimators_) {
    estimator.second->setR(config.R_hdg_compass, map_hdg_measurement_name_id_.find("hdg_compass")->second);
    estimator.second->setR(config.R_hdg_hector, map_hdg_measurement_name_id_.find("hdg_hector")->second);
    estimator.second->setR(config.R_hdg_aloam, map_hdg_measurement_name_id_.find("hdg_aloam")->second);
    estimator.second->setR(config.R_hdg_liosam, map_hdg_measurement_name_id_.find("hdg_liosam")->second);
    estimator.second->setR(config.R_hdg_brick, map_hdg_measurement_name_id_.find("hdg_brick")->second);
    estimator.second->setR(config.R_hdg_vio, map_hdg_measurement_name_id_.find("hdg_vio")->second);
    estimator.second->setR(config.R_hdg_vslam, map_hdg_measurement_name_id_.find("hdg_vslam")->second);
    estimator.second->setR(config.R_rate_gyro, map_hdg_measurement_name_id_.find("rate_gyro")->second);
    estimator.second->setR(config.R_rate_optflow, map_hdg_measurement_name_id_.find("rate_optflow")->second);
    estimator.second->setR(config.R_rate_icp, map_hdg_measurement_name_id_.find("rate_icp")->second);
  }
}
//}

// --------------------------------------------------------------
// |                      helper functions                      |
// --------------------------------------------------------------

/*  //{ stateEstimatorsPrediction() */

void Odometry::stateEstimatorsPrediction(const geometry_msgs::Vector3 &acc_in, double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  if (!is_initialized_)
    return;

  if (!got_fcu_untilted_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Lateral prediction not running. Waiting for fcu_untilted tf.");
    return;
  }


  if (dt <= 0.0) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Lateral estimator prediction dt=%f, skipping prediction.", dt);
    return;
  }

  Vec2 input;

  for (auto &estimator : _lateral_estimators_) {

    // Rotate body frame measurements into estimator frame
    double current_hdg;
    for (auto &hdg_estimator : heading_estimators_) {
      if (estimator.first == "GPS" || estimator.first == "RTK") {

        geometry_msgs::Quaternion q_pixhawk = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_.pose.pose.orientation);

        try {
          current_hdg = mrs_lib::AttitudeConverter(q_pixhawk).getHeading();
        }
        catch (...) {
          ROS_WARN("[Odometry]: failed to getHeading() from q_pixhawk");
        }

        break;
      } else {
        if (estimator.first == hdg_estimator.first) {
          hdg_estimator.second->getState(0, current_hdg);
          break;
        }
      }
    }

    if (estimator.first == "BRICK" && !brick_reliable_) {
      continue;
    }

    // transform control accelerations to untilted frame
    geometry_msgs::Vector3Stamped acc_untilted;
    acc_untilted.vector          = acc_in;
    acc_untilted.header.frame_id = fcu_frame_id_;
    acc_untilted.header.stamp    = ros::Time::now();
    auto response_acc            = transformer_.transformSingle(fcu_untilted_frame_id_, acc_untilted);
    if (response_acc) {
      acc_untilted = response_acc.value();
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", acc_untilted.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
    }

    geometry_msgs::Vector3 acc_global;
    getRotatedVector(acc_untilted.vector, current_hdg, acc_global);

    if (!std::isfinite(acc_global.x)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"acc_x\" (stateEstimatorsPrediction) !!!");
      return;
    }

    if (!std::isfinite(acc_global.y)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"acc_y\" (stateEstimatorsPrediction) !!!");
      return;
    }

    input(0) = acc_global.x;
    input(1) = acc_global.y;


    if (input_stamp.toSec() != 0) {
      estimator.second->doPrediction(input, dt, input_stamp, predict_stamp);
    } else {
      // input time is zero if no controller is active
      estimator.second->doPrediction(input, dt, predict_stamp, predict_stamp);
    }
  }
}

//}

/*  //{ stateEstimatorsCorrection() */

void Odometry::stateEstimatorsCorrection(double x, double y, const std::string &measurement_name, const ros::Time &meas_stamp, const ros::Time &predict_stamp) {

  std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id_.find(measurement_name);
  if (it_measurement_id == map_measurement_name_id_.end()) {
    ROS_ERROR("[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(x)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"x\" (stateEstimatorsCorrection) !!!");
    return;
  }

  if (!std::isfinite(y)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"y\" (stateEstimatorsCorrection) !!!");
    return;
  }

  Vec2 mes;

  for (auto &estimator : _lateral_estimators_) {

    mes(0) = x;
    mes(1) = y;

    // Rotate body frame measurements into estimator frame (vel_mavros) is missing, because it is calculated as difference of ENU positions
    if (measurement_name == "vel_optflow" || measurement_name == "vel_icp" || measurement_name == "acc_imu") {

      double current_hdg;
      for (auto &hdg_estimator : heading_estimators_) {

        if (estimator.first == "GPS" || estimator.first == "RTK") {

          geometry_msgs::Quaternion q_pixhawk = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_.pose.pose.orientation);
          try {
            current_hdg = mrs_lib::AttitudeConverter(q_pixhawk).getHeading();
          }
          catch (...) {
            ROS_WARN("[Odometry]: failed to getHeading() from q_pixhawk");
          }
          break;

        } else {
          if (estimator.first == hdg_estimator.first) {
            hdg_estimator.second->getState(0, current_hdg);
            break;
          }
        }
      }
      double mes_x, mes_y;
      mes_x  = mes(0) * cos(current_hdg) - mes(1) * sin(current_hdg);
      mes_y  = mes(0) * sin(current_hdg) + mes(1) * cos(current_hdg);
      mes(0) = mes_x;
      mes(1) = mes_y;
    }

    estimator.second->doCorrection(mes, it_measurement_id->second, meas_stamp, predict_stamp);
  }
}

//}

/* altitudeEstimatorsPrediction //{ */

void Odometry::altitudeEstimatorsPrediction(const double input, const double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  if (dt <= 0.0) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Altitude estimator prediction dt=%f, skipping prediction.", dt);
    return;
  }

  for (auto &estimator : _altitude_estimators_) {

    /* do not run repredictor estimators when missing aloam data //{*/
    if ((estimator.second->getName() == "ALOAMGARM" || estimator.second->getName() == "ALOAMREP") && !got_aloam_odom_) {
      ROS_WARN_ONCE("[Odometry]: ALOAMGARM/ALOAMREP active, but not received aloam odom yet. Skipping altitude prediction until aloam odom starts coming.");
      continue;
    } else if ((estimator.second->getName() == "ALOAMGARM" || estimator.second->getName() == "ALOAMREP") && got_aloam_odom_) {
      ROS_INFO_ONCE("[Odometry]: Received aloam odom. ALOAMGARM/ALOAMREP altitude prediction starting.");
    }
    /*//}*/

    if (input_stamp.toSec() > predict_stamp.toSec() - 0.5) {
      estimator.second->doPrediction(input, dt, input_stamp, predict_stamp);
    } else {
      // pass zero input if the input is too old (no controller is active)
      ROS_WARN_THROTTLE(2.0, "[Odometry] Last attitude command too old, passing zero input to altitude estimator.");
      estimator.second->doPrediction(0, dt, predict_stamp, predict_stamp);
    }
  }
}

/*//}*/


/*  //{ altitudeEstimatorCorrection() */

void Odometry::altitudeEstimatorCorrection(double value, const std::string &measurement_name, const ros::Time &meas_stamp, const ros::Time &predict_stamp) {

  std::map<std::string, int>::iterator it_measurement_id = map_alt_measurement_name_id_.find(measurement_name);
  if (it_measurement_id == map_alt_measurement_name_id_.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" of %s (altitudeEstimatorCorrection) !!!", measurement_name.c_str());
    return;
  }

  if (fabs(value) > 100) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator correction: %f", value);
  }

  for (auto &estimator : _altitude_estimators_) {

    /* do not run repredictor estimators when missing aloam data //{*/
    if ((estimator.second->getName() == "ALOAMGARM" || estimator.second->getName() == "ALOAMREP") && !got_aloam_odom_) {
      ROS_WARN_ONCE("[Odometry]: ALOAMGARM/ALOAMREP active, but not received aloam odom yet. Skipping altitude correction until aloam odom starts coming.");
      continue;
    } else if ((estimator.second->getName() == "ALOAMGARM" || estimator.second->getName() == "ALOAMREP") && got_aloam_odom_) {
      ROS_INFO_ONCE("[Odometry]: Received aloam odom. ALOAMGARM/ALOAMREP altitude correction starting.");
    }
    /*//}*/

    estimator.second->doCorrection(value, it_measurement_id->second, meas_stamp, predict_stamp, measurement_name);
  }
}

//}

/*  //{ altitudeEstimatorCorrection() */

void Odometry::altitudeEstimatorCorrection(double value, const std::string &measurement_name,
                                           const std::shared_ptr<mrs_uav_odometry::AltitudeEstimator> &estimator, const ros::Time &meas_stamp,
                                           const ros::Time &predict_stamp) {

  /* do not run repredictor estimators when missing aloam data //{*/
  if ((estimator->getName() == "ALOAMGARM" || estimator->getName() == "ALOAMREP") && !got_aloam_odom_) {
    ROS_WARN_ONCE("[Odometry]: ALOAMGARM/ALOAMREP active, but not received aloam odom yet. Skipping altitude correction until aloam odom starts coming.");
    return;
  } else if ((estimator->getName() == "ALOAMGARM" || estimator->getName() == "ALOAMREP") && got_aloam_odom_) {
    ROS_INFO_ONCE("[Odometry]: Received aloam odom. ALOAMGARM/ALOAMREP altitude correction starting.");
  }
  /*//}*/

  std::map<std::string, int>::iterator it_measurement_id = map_alt_measurement_name_id_.find(measurement_name);
  if (it_measurement_id == map_alt_measurement_name_id_.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" of %s (altitudeEstimatorCorrection) !!!", measurement_name.c_str());
    return;
  }

  estimator->doCorrection(value, it_measurement_id->second, meas_stamp, predict_stamp, measurement_name);

  // Publisher of ALOAM computational delay
  if (estimator->getName() == "ALOAMREP" && measurement_name == "height_aloam") {
    mrs_msgs::Float64Stamped msg;
    msg.header.stamp = predict_stamp;
    msg.value        = predict_stamp.toSec() - meas_stamp.toSec();
    pub_debug_aloam_delay_.publish(msg);
  }
}

//}

/*  //{ headingEstimatorsPrediction() */

void Odometry::headingEstimatorsPrediction(const double hdg, const double hdg_rate, const double dt, const ros::Time &input_stamp,
                                           const ros::Time &predict_stamp) {

  if (dt <= 0.0) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Lateral estimator prediction dt=%f, skipping prediction.", dt);
    return;
  }

  if (!std::isfinite(hdg)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"hdg\" (headingEstimatorsPrediction) !!!");
    return;
  }

  if (!std::isfinite(hdg_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"hdg rate\" (headingEstimatorsPrediction) !!!");
    return;
  }


  for (auto &estimator : heading_estimators_) {

    hdg_u_t input = input.Zero();
    input << hdg, hdg_rate;

    double current_hdg;
    estimator.second->getState(0, current_hdg);
    input(0) = radians::unwrap(input(0), current_hdg);
    if (input_stamp.toSec() > predict_stamp.toSec() - 0.5) {
      estimator.second->doPrediction(input, dt, input_stamp, predict_stamp);
    } else {
      // pass zero input if the input is too old (no controller active)
      input(0) = 0;
      input(1) = 0;
      ROS_WARN_THROTTLE(2.0, "[Odometry] Last attitude command too old, passing zero input to heading estimator.");
      estimator.second->doPrediction(input, dt, predict_stamp, predict_stamp);
    }
  }
}

//}

/*  //{ headingEstimatorsCorrection() */

void Odometry::headingEstimatorsCorrection(const double value, const std::string &measurement_name, const ros::Time &meas_stamp,
                                           const ros::Time &predict_stamp) {

  std::map<std::string, int>::iterator it_measurement_id = map_hdg_measurement_name_id_.find(measurement_name);
  if (it_measurement_id == map_hdg_measurement_name_id_.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" (headingEstimatorsCorrection) !!!");
    return;
  }

  double z = value;

  for (auto &estimator : heading_estimators_) {


    if (measurement_name == "hdg_compass") {
      double current_hdg;
      estimator.second->getState(0, current_hdg);

      z = radians::unwrap(z, current_hdg);
    }

    estimator.second->doCorrection(z, it_measurement_id->second, meas_stamp, predict_stamp);
  }
}

//}

/* //{ getGlobalRot() */
void Odometry::getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz) {

  double heading;
  try {
    heading = mrs_lib::AttitudeConverter(q_msg).getHeading();
  }
  catch (...) {
    ROS_ERROR("exception caught, could not calculate heading.");
    return;
  }

  Eigen::Matrix3d original_R     = mrs_lib::AttitudeConverter(q_msg);
  Eigen::Matrix3d undo_heading_R = mrs_lib::AttitudeConverter(Eigen::AngleAxis(-heading, Eigen::Vector3d(0, 0, 1)));

  // undo the heading
  Eigen::Matrix3d R_no_heading = mrs_lib::AttitudeConverter(undo_heading_R * original_R);

  rx = mrs_lib::geometry::angleBetween(Eigen::Vector3d(1, 0, 0), R_no_heading.col(0));
  ry = mrs_lib::geometry::angleBetween(Eigen::Vector3d(0, 1, 0), R_no_heading.col(1));
  rz = heading;
}
//}

/* //{ getRotatedVector() */
void Odometry::getRotatedVector(const geometry_msgs::Vector3 &acc_in, double hdg_in, geometry_msgs::Vector3 &acc_out) {

  tf2::Quaternion q_hdg = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(hdg_in);

  tf2::Vector3 acc_tf2(acc_in.x, acc_in.y, acc_in.z);

  acc_tf2   = quatRotate(q_hdg, acc_tf2);
  acc_out.x = acc_tf2.getX();
  acc_out.y = acc_tf2.getY();
  acc_out.z = acc_tf2.getZ();
}
//}

/* rotateLateralStates() //{ */

void Odometry::rotateLateralStates(const double hdg_new, const double hdg_old) {

  hdg_diff_ = hdg_new - hdg_old;
  double cy = cos(hdg_diff_);
  double sy = sin(hdg_diff_);

  for (auto &estimator : _lateral_estimators_) {
    LatState2D old_state;
    if (!estimator.second->getStates(old_state)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Lateral estimator not initialized.");
      return;
    }
    if (estimator.first == "GPS") {
      ROS_INFO("[Odometry]: Rotating lateral state after hdg estimator switch.");
      ROS_INFO_STREAM("[Odometry]: old_state:" << old_state);
    }

    LatState2D new_state;
    for (int i = 0; i < new_state.size(); i++) {
      new_state(i, 0) = old_state(i, 0) * cy - old_state(i, 1) * sy;
      new_state(i, 1) = old_state(i, 0) * sy + old_state(i, 1) * cy;
    }
    if (estimator.first == "GPS") {
      ROS_INFO_STREAM("[Odometry]: new_state:" << new_state);
    }
    estimator.second->setStates(new_state);
  }
}

//}

/* getCurrentHeading() //{ */

double Odometry::getCurrentHeading() {

  double hdg;
  if (current_hdg_estimator_->getName() == "PIXHAWK") {

    geometry_msgs::Quaternion q_pixhawk = mrs_lib::get_mutexed(mutex_odom_pixhawk_, odom_pixhawk_.pose.pose.orientation);
    try {
      hdg = mrs_lib::AttitudeConverter(q_pixhawk).getHeading();
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during getting heading (getCurrentHeading)");
    }

  } else {

    {
      std::scoped_lock lock(mutex_current_hdg_estimator_);
      current_hdg_estimator_->getState(0, hdg);
    }
  }

  return hdg;
}

//}

/* //{ getGlobalZAcceleration() */
double Odometry::getGlobalZAcceleration(const geometry_msgs::Quaternion &q_msg, const double &acc_z_in) {

  tf2::Quaternion q_body = mrs_lib::AttitudeConverter(q_msg);

  // Get acceleration pointing upward from the body frame
  tf2::Vector3 acc_z(0, 0, acc_z_in);

  // Transform upward axis to the world frame
  tf2::Vector3 acc_world;
  acc_world = quatRotate(q_body.inverse(), acc_z);

  // Return Z component of acceleration in world frame
  return acc_world.getZ();
}
//}

/* //{ changeCurrentEstimator() */
bool Odometry::changeCurrentEstimator(const mrs_msgs::EstimatorType &desired_estimator) {

  mrs_msgs::EstimatorType target_estimator = desired_estimator;
  target_estimator.name                    = _estimator_type_names_[target_estimator.type];

  alt_x_t alt_x = alt_x.Zero();
  {
    std::scoped_lock lock(mutex_altitude_estimator_);
    if (isUavFlying() && !current_alt_estimator_->getStates(alt_x)) {
      ROS_WARN("[Odometry]: Altitude estimator not initialized.");
      return false;
    }
  }

  // Return if already active
  if (toUppercase(current_lat_estimator_name_) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired lateral estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  /* OPTFLOW //{ */
  if (target_estimator.type == mrs_msgs::EstimatorType::OPTFLOW) {

    if (!optflow_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. OPTFLOW estimator not active.");
      return false;
    }

    if (!got_optflow_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. No new optic flow msgs received.");
      return false;
    }

    if (alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. Current altitude %f. Must descend to %f.", alt_x(mrs_msgs::AltitudeStateNames::HEIGHT),
                _max_optflow_altitude_);
      return false;
    }


    mrs_lib::set_mutexed(mutex_max_altitude_, _max_optflow_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_optflow_altitude_);

    //}

    /* GPS //{ */

  } else if (target_estimator.type == mrs_msgs::EstimatorType::GPS) {

    if (!gps_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. GPS estimator not active.");
      return false;
    }

    if (!gps_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. GPS not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);


    //}

    /* OPTFLOWGPS //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {

    if (!optflow_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. OPTFLOW estimator not active.");
      return false;
    }

    if (!got_optflow_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. No new optic flow msgs received.");
      return false;
    }

    if (alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Current altitude %f. Must descend to %f.", alt_x(mrs_msgs::AltitudeStateNames::HEIGHT),
                _max_optflow_altitude_);
      return false;
    }

    if (!gps_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. GPS estimator not active.");
      return false;
    }

    if (!gps_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* RTK //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::RTK) {

    if (!rtk_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. RTK estimator not active.");
      return false;
    }

    if (!got_rtk_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. No new rtk msgs received.");
      return false;
    }

    if (!gps_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. GPS not active.");
      return false;
    }

    if (!gps_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* T265 //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::T265) {

    if (!t265_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. T265 not available in this world.");
      return false;
    }

    if (!got_odom_t265_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. No new T265 odom msgs received.");
      return false;
    }

    if (!t265_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. T265 odometry is not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* HECTOR //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::HECTOR) {

    if (!hector_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. HECTOR estimator not active.");
      return false;
    }

    if (!got_hector_pose_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. No new hector msgs received.");
      return false;
    }

    hector_reliable_ = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* ALOAM //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ALOAM) {

    if (!aloam_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAM type. ALOAM estimator not active.");
      return false;
    }

    if (!got_aloam_odom_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAM type. No new aloam msgs received.");
      return false;
    }

    aloam_reliable_ = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);


    //}

    /* ALOAMGARM //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ALOAMGARM) {

    if (!aloamgarm_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAMGARM type. ALOAMGARM estimator not active.");
      return false;
    }

    if (!got_aloam_odom_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAMGARM type. No new aloam msgs received.");
      return false;
    }

    aloam_reliable_ = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);


    //}

    /* ALOAMREP //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ALOAMREP) {

    if (!aloamrep_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAMREP type. ALOAMREP estimator not active.");
      return false;
    }

    if (!got_aloam_odom_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAMREP type. No new aloam msgs received.");
      return false;
    }

    aloam_reliable_ = true;
    /* LIOSAM //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::LIOSAM) {

    if (!liosam_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to LIOSAM type. LIOSAM estimator not active.");
      return false;
    }

    if (!got_liosam_odom_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to LIOSAM type. No new LIOSAM msgs received.");
      return false;
    }

    liosam_reliable_ = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* ICP //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ICP) {

    if (!icp_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP type. ICP estimator not active.");
      return false;
    }

    if (!got_icp_twist_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP type. No new ICP stmsgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* VIO //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::VIO) {

    if (!vio_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO type. VIO estimator not active.");
      return false;
    }

    if (!got_vio_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO type. No new vio msgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* VSLAM //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::VSLAM) {

    if (!vslam_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to VSLAM type. VSLAM estimator not active.");
      return false;
    }

    if (!got_vslam_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to VSLAM type. No new VSLAM msgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* BRICK //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::BRICK) {

    if (!brick_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. BRICK estimator not active.");
      return false;
    }

    if (!got_brick_pose_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. No new brick msgs received.");
      return false;
    }

    if (!brick_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. brick detection is not reliable");
      return false;
    }

    if (estimator_type_.type != mrs_msgs::EstimatorType::BRICK) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Already in BRICK state estimator.");
      fallback_brick_estimator_type_ = estimator_type_;
      ROS_INFO("[Odometry]: Fallback from BRICK estimator: %s", estimator_type_.name.c_str());
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

    /* BRICKFLOW //{ */
  } else if (target_estimator.type == mrs_msgs::EstimatorType::BRICKFLOW) {

    if (!optflow_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. OPTFLOW estimator not active.");
      return false;
    }

    if (!got_optflow_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. No new optic flow msgs received.");
      return false;
    }

    if (alt_x(2) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. Current altitude %f. Must descend to %f.", alt_x(2), _max_optflow_altitude_);
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    //}

  } else {

    ROS_ERROR("[Odometry]: Rejected transition to invalid type %s.", target_estimator.name.c_str());
    return false;
  }

  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _active_state_estimators_names_)) {

    {
      std::scoped_lock lock(mutex_current_lat_estimator_);

      current_lat_estimator_      = _lateral_estimators_.find(target_estimator.name)->second;
      current_lat_estimator_name_ = current_lat_estimator_->getName();
    }

    ROS_WARN("[Odometry]: Transition to %s state estimator successful", toUppercase(current_lat_estimator_name_).c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to non-active state estimator %s", target_estimator.name.c_str());
    is_updating_state_ = false;
    return false;
  }

  estimator_type_      = target_estimator;
  estimator_type_.name = _estimator_type_names_[estimator_type_.type];
  estimator_iteration_++;

  is_updating_state_ = false;
  return true;
}

//}

/* //{ changeCurrentAltitudeEstimator() */
bool Odometry::changeCurrentAltitudeEstimator(const mrs_msgs::AltitudeType &desired_estimator) {

  mrs_msgs::AltitudeType target_estimator = desired_estimator;
  target_estimator.name                   = _altitude_type_names_[target_estimator.type];

  ROS_INFO("[Odometry]: Changing altitude estimator to %s. (desired_estimator: %s)", target_estimator.name.c_str(), desired_estimator.name.c_str());

  if (target_estimator.type != mrs_msgs::AltitudeType::HEIGHT && target_estimator.type != mrs_msgs::AltitudeType::PLANE &&
      target_estimator.type != mrs_msgs::AltitudeType::BRICK && target_estimator.type != mrs_msgs::AltitudeType::VIO &&
      target_estimator.type != mrs_msgs::AltitudeType::ALOAM && target_estimator.type != mrs_msgs::AltitudeType::LIOSAM &&
      target_estimator.type != mrs_msgs::AltitudeType::ALOAMGARM && target_estimator.type != mrs_msgs::AltitudeType::ALOAMREP &&
      target_estimator.type != mrs_msgs::AltitudeType::BARO && target_estimator.type != mrs_msgs::AltitudeType::RTK) {
    ROS_ERROR("[Odometry]: Rejected transition to invalid altitude type %d: %s.", target_estimator.type, target_estimator.name.c_str());
    return false;
  }

  if (target_estimator.type == mrs_msgs::AltitudeType::HEIGHT && !garmin_enabled_) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: Tried to switch altitude estimator to HEIGHT without garmin_enabled_. Do you have garmin_down in SENSORS in .bashrc?");
    return false;
  }

  // Return if already active
  if (toUppercase(current_alt_estimator_name_) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired altitue estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  /* brick type //{ */

  if (target_estimator.type == mrs_msgs::AltitudeType::BRICK) {

    if (!brick_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. BRICK not active.");
      return false;
    }

    if (!got_brick_pose_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. No new brick msgs received.");
      return false;
    }

    if (isUavFlying()) {

      // update the altitude state
      alt_x_t alt_x = alt_x.Zero();
      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        if (!current_alt_estimator_->getStates(alt_x)) {
          ROS_WARN("[Odometry]: Altitude estimator not initialized.");
          return false;
        }
      }
      if (alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) > _brick_max_valid_alt_) {
        ROS_ERROR("[Odometry]: Cannot transition to BRICK type. Current altitude %f. Must descend to %f.", alt_x(mrs_msgs::AltitudeStateNames::HEIGHT),
                  _brick_max_valid_alt_);
        return false;
      }
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _brick_max_valid_alt_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _brick_max_valid_alt_);
  }

  //}

  /* plane type //{ */

  if (target_estimator.type == mrs_msgs::AltitudeType::PLANE) {

    if (!got_plane_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to PLANE type. No new plane msgs received.");
      return false;
    }

    if (isUavFlying()) {
      // update the altitude state
      alt_x_t alt_x = alt_x.Zero();
      {
        std::scoped_lock lock(mutex_altitude_estimator_);
        if (!current_alt_estimator_->getStates(alt_x)) {
          ROS_WARN("[Odometry]: Altitude estimator not initialized.");
          return false;
        }
      }
      if (alt_x(mrs_msgs::AltitudeStateNames::HEIGHT) > _plane_max_valid_alt_) {
        ROS_ERROR("[Odometry]: Cannot transition to PLANE type. Current altitude %f. Must descend to %f.", alt_x(mrs_msgs::AltitudeStateNames::HEIGHT),
                  _plane_max_valid_alt_);
        return false;
      }
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _plane_max_valid_alt_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _plane_max_valid_alt_);
  }

  //}

  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _altitude_estimators_names_)) {
    {
      std::scoped_lock lock(mutex_current_alt_estimator_);

      current_alt_estimator_      = _altitude_estimators_.find(target_estimator.name)->second;
      current_alt_estimator_name_ = current_alt_estimator_->getName();
    }

    ROS_WARN("[Odometry]: Transition to %s altitude estimator successful", current_alt_estimator_name_.c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to nonexistent altitude estimator %s", target_estimator.name.c_str());
    is_updating_state_ = false;
    return false;
  }

  alt_estimator_type_      = target_estimator;
  alt_estimator_type_.name = _altitude_type_names_[alt_estimator_type_.type];
  estimator_iteration_++;
  is_updating_state_ = false;
  return true;
}

//}

/* //{ changeCurrentHeadingEstimator() */
bool Odometry::changeCurrentHeadingEstimator(const mrs_msgs::HeadingType &desired_estimator) {

  mrs_msgs::HeadingType target_estimator = desired_estimator;
  target_estimator.name                  = _heading_type_names_[target_estimator.type];

  if (target_estimator.type != mrs_msgs::HeadingType::PIXHAWK && target_estimator.type != mrs_msgs::HeadingType::GYRO &&
      target_estimator.type != mrs_msgs::HeadingType::COMPASS && target_estimator.type != mrs_msgs::HeadingType::OPTFLOW &&
      target_estimator.type != mrs_msgs::HeadingType::HECTOR && target_estimator.type != mrs_msgs::HeadingType::BRICK &&
      target_estimator.type != mrs_msgs::HeadingType::VIO && target_estimator.type != mrs_msgs::HeadingType::VSLAM &&
      target_estimator.type != mrs_msgs::HeadingType::ICP && target_estimator.type != mrs_msgs::HeadingType::BRICKFLOW &&
      target_estimator.type != mrs_msgs::HeadingType::ALOAM && target_estimator.type != mrs_msgs::HeadingType::ALOAMREP &&
      target_estimator.type != mrs_msgs::HeadingType::LIOSAM) {
    ROS_ERROR("[Odometry]: Rejected transition to invalid type %s.", target_estimator.name.c_str());
    return false;
  }

  // Return if already active
  if (toUppercase(current_hdg_estimator_name_) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired heading estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  /* BRICK //{ */
  if (target_estimator.type == mrs_msgs::HeadingType::BRICK && hdg_estimator_type_.type != mrs_msgs::HeadingType::BRICK) {

    if (!brick_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. BRICK estimator not active.");
      return false;
    }

    if (!got_brick_pose_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. No new brick msgs received.");
      return false;
    }

    if (!brick_reliable_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. brick detection is not reliable");
      return false;
    }

    fallback_brick_hdg_estimator_type_ = hdg_estimator_type_;
    ROS_INFO("[Odometry]: Fallback from BRICK heading estimator: %s", hdg_estimator_type_.name.c_str());


    //}

    /* HECTOR //{ */
  } else if (target_estimator.type == mrs_msgs::HeadingType::HECTOR) {

    if (!hector_active_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. HECTOR estimator not active.");
      return false;
    }

    if (!got_hector_pose_ && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. No new hector msgs received.");
      return false;
    }
  }

  //}

  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _active_heading_estimators_names_)) {
    if (is_initialized_) {

      {
        std::scoped_lock lock(mutex_current_hdg_estimator_);
        current_hdg_estimator_      = heading_estimators_.find(target_estimator.name)->second;
        current_hdg_estimator_name_ = current_hdg_estimator_->getName();
      }

    } else {

      {
        std::scoped_lock lock(mutex_current_hdg_estimator_);

        current_hdg_estimator_      = heading_estimators_.find(target_estimator.name)->second;
        current_hdg_estimator_name_ = current_hdg_estimator_->getName();
      }
    }


    ROS_WARN("[Odometry]: Transition to %s heading estimator successful", toUppercase(current_hdg_estimator_name_).c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to nonexistent heading estimator %s", toUppercase(target_estimator.name).c_str());
    is_updating_state_ = false;
    return false;
  }

  hdg_estimator_type_      = target_estimator;
  hdg_estimator_type_.name = _heading_type_names_[hdg_estimator_type_.type];
  is_updating_state_       = false;
  finished_state_update_   = true;
  estimator_iteration_++;
  ROS_INFO("[Odometry]: finished hdg switch");
  return true;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::EstimatorType &type) {

  if (type.type == mrs_msgs::EstimatorType::OPTFLOW || type.type == mrs_msgs::EstimatorType::GPS || type.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
      type.type == mrs_msgs::EstimatorType::RTK || type.type == mrs_msgs::EstimatorType::VIO || type.type == mrs_msgs::EstimatorType::VSLAM ||
      type.type == mrs_msgs::EstimatorType::BRICK || type.type == mrs_msgs::EstimatorType::T265 || type.type == mrs_msgs::EstimatorType::HECTOR ||
      type.type == mrs_msgs::EstimatorType::BRICKFLOW || type.type == mrs_msgs::EstimatorType::ICP || type.type == mrs_msgs::EstimatorType::ALOAM ||
      type.type == mrs_msgs::EstimatorType::ALOAMGARM || type.type == mrs_msgs::EstimatorType::ALOAMREP || type.type == mrs_msgs::EstimatorType::LIOSAM) {
    return true;
  }

  return false;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::HeadingType &type) {

  if (type.type == mrs_msgs::HeadingType::PIXHAWK || type.type == mrs_msgs::HeadingType::GYRO || type.type == mrs_msgs::HeadingType::COMPASS ||
      type.type == mrs_msgs::HeadingType::OPTFLOW || type.type == mrs_msgs::HeadingType::HECTOR || type.type == mrs_msgs::HeadingType::BRICK ||
      type.type == mrs_msgs::HeadingType::VIO || type.type == mrs_msgs::HeadingType::VSLAM || type.type == mrs_msgs::HeadingType::ICP ||
      type.type == mrs_msgs::HeadingType::BRICKFLOW || type.type == mrs_msgs::HeadingType::ALOAM || type.type == mrs_msgs::HeadingType::ALOAMREP ||
      type.type == mrs_msgs::HeadingType::LIOSAM) {
    return true;
  }

  return false;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::AltitudeType &type) {

  if (type.type == mrs_msgs::AltitudeType::HEIGHT || type.type == mrs_msgs::AltitudeType::PLANE || type.type == mrs_msgs::AltitudeType::BRICK ||
      type.type == mrs_msgs::AltitudeType::VIO || type.type == mrs_msgs::AltitudeType::ALOAM || type.type == mrs_msgs::AltitudeType::BARO ||
      type.type == mrs_msgs::AltitudeType::ALOAMGARM || type.type == mrs_msgs::AltitudeType::ALOAMREP || type.type == mrs_msgs::AltitudeType::RTK ||
      type.type == mrs_msgs::AltitudeType::LIOSAM) {
    return true;
  }

  return false;
}

//}

/* isTimestampOK() //{ */
bool Odometry::isTimestampOK(const double curr_sec, const double prev_sec) {

  double delta_tol = 100;

  double delta = curr_sec - prev_sec;

  if (curr_sec < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: current timestamp negative: %f", curr_sec);
    return false;
  }

  if (prev_sec < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: current timestamp negative: %f", prev_sec);
    return false;
  }

  if (delta < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: time delta negative: %f", delta);
    return false;
  }

  if (fabs(delta) < 0.001) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: time delta too small: %f", delta);
    return false;
  }

  if (delta > delta_tol) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: time delta %f > %f", delta, delta_tol);
    return false;
  }

  return true;
}
//}

/* //{ printOdometryDiag() */
std::string Odometry::printOdometryDiag() {

  std::string s_diag;

  mrs_msgs::EstimatorType type;

  /* lateral //{ */

  {
    std::scoped_lock lock(mutex_estimator_type_);

    type.type = estimator_type_.type;
  }

  s_diag += "Current lateral estimator type: ";
  s_diag += std::to_string(type.type);
  s_diag += " - ";

  if (type.type == mrs_msgs::EstimatorType::OPTFLOW) {
    s_diag += "OPTFLOW";
  } else if (type.type == mrs_msgs::EstimatorType::GPS) {
    s_diag += "GPS";
  } else if (type.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    s_diag += "OPTFLOWGPS";
  } else if (type.type == mrs_msgs::EstimatorType::RTK) {
    s_diag += "RTK";
  } else if (type.type == mrs_msgs::EstimatorType::VIO) {
    s_diag += "VIO";
  } else if (type.type == mrs_msgs::EstimatorType::VSLAM) {
    s_diag += "VSLAM";
  } else if (type.type == mrs_msgs::EstimatorType::BRICK) {
    s_diag += "BRICK";
  } else if (type.type == mrs_msgs::EstimatorType::T265) {
    s_diag += "T265";
  } else if (type.type == mrs_msgs::EstimatorType::HECTOR) {
    s_diag += "HECTOR";
  } else if (type.type == mrs_msgs::EstimatorType::ALOAM) {
    s_diag += "ALOAM";
  } else if (type.type == mrs_msgs::EstimatorType::ALOAMGARM) {
    s_diag += "ALOAMGARM";
  } else if (type.type == mrs_msgs::EstimatorType::ALOAMREP) {
    s_diag += "ALOAMREP";
  } else if (type.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    s_diag += "BRICKFLOW";
  } else if (type.type == mrs_msgs::EstimatorType::ICP) {
    s_diag += "ICP";
  } else if (type.type == mrs_msgs::EstimatorType::LIOSAM) {
    s_diag += "LIOSAM";
  } else {
    s_diag += "UNKNOWN";
  }

  //}

  /* heading //{ */

  mrs_msgs::HeadingType hdg_type;

  {
    std::scoped_lock lock(mutex_hdg_estimator_type_);

    hdg_type.type = hdg_estimator_type_.type;
  }
  s_diag += ", Current heading estimator type: ";
  s_diag += std::to_string(hdg_type.type);
  s_diag += " - ";

  if (hdg_type.type == mrs_msgs::HeadingType::PIXHAWK) {
    s_diag += "PIXHAWK";
  } else if (hdg_type.type == mrs_msgs::HeadingType::GYRO) {
    s_diag += "GYRO";
  } else if (hdg_type.type == mrs_msgs::HeadingType::COMPASS) {
    s_diag += "COMPASS";
  } else if (hdg_type.type == mrs_msgs::HeadingType::OPTFLOW) {
    s_diag += "OPTFLOW";
  } else if (hdg_type.type == mrs_msgs::HeadingType::HECTOR) {
    s_diag += "HECTOR";
  } else if (hdg_type.type == mrs_msgs::HeadingType::ALOAM) {
    s_diag += "ALOAM";
  } else if (hdg_type.type == mrs_msgs::HeadingType::ALOAMREP) {
    s_diag += "ALOAMREP";
  } else if (hdg_type.type == mrs_msgs::HeadingType::BRICK) {
    s_diag += "BRICK";
  } else if (hdg_type.type == mrs_msgs::HeadingType::VIO) {
    s_diag += "VIO";
  } else if (hdg_type.type == mrs_msgs::HeadingType::VSLAM) {
    s_diag += "VSLAM";
  } else if (hdg_type.type == mrs_msgs::HeadingType::ICP) {
    s_diag += "ICP";
  } else if (hdg_type.type == mrs_msgs::HeadingType::BRICKFLOW) {
    s_diag += "BRICKFLOW";
  } else if (hdg_type.type == mrs_msgs::HeadingType::LIOSAM) {
    s_diag += "LIOSAM";
  } else {
    s_diag += "UNKNOWN";
  }

  //}

  // altitude
  mrs_msgs::AltitudeType alt_type;

  {
    std::scoped_lock lock(mutex_alt_estimator_type_);

    alt_type.type = alt_estimator_type_.type;
  }
  s_diag += ", Current altitude estimator type: ";
  s_diag += std::to_string(alt_type.type);
  s_diag += " - ";

  if (alt_type.type == mrs_msgs::AltitudeType::HEIGHT) {
    s_diag += "HEIGHT";
  } else if (alt_type.type == mrs_msgs::AltitudeType::PLANE) {
    s_diag += "PLANE";
  } else if (alt_type.type == mrs_msgs::AltitudeType::BRICK) {
    s_diag += "BRICK";
  } else if (alt_type.type == mrs_msgs::AltitudeType::VIO) {
    s_diag += "VIO";
  } else if (alt_type.type == mrs_msgs::AltitudeType::ALOAM) {
    s_diag += "ALOAM";
  } else if (alt_type.type == mrs_msgs::AltitudeType::ALOAMGARM) {
    s_diag += "ALOAMGARM";
  } else if (alt_type.type == mrs_msgs::AltitudeType::ALOAMREP) {
    s_diag += "ALOAMREP";
  } else if (alt_type.type == mrs_msgs::AltitudeType::BARO) {
    s_diag += "BARO";
  } else if (alt_type.type == mrs_msgs::AltitudeType::RTK) {
    s_diag += "RTK";
  } else if (alt_type.type == mrs_msgs::AltitudeType::LIOSAM) {
    s_diag += "LIOSAM";
  } else {
    s_diag += "UNKNOWN";
  }
  return s_diag;
}

//}

/* stringInVector() //{ */

bool Odometry::stringInVector(const std::string &value, const std::vector<std::string> &vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

/* calculatePixhawkOdomOffset() //{ */

bool Odometry::calculatePixhawkOdomOffset(void) {

  if (!got_pixhawk_utm_ || !got_odom_pixhawk_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: cannot calculate pixhawk_odom_offset, waiting for data: UTM: %s, ODOM: %s", btoa(got_pixhawk_utm_),
                      btoa(got_odom_pixhawk_));
    return false;
  }

  if (got_pixhawk_odom_offset_) {
    return true;
  }

  // initialize gps_origin to local_origin position
  if (_init_gps_origin_local_) {

    {
      std::scoped_lock lock(mutex_odom_pixhawk_);

      pixhawk_odom_offset_x_ = _init_gps_offset_x_ - odom_pixhawk_.pose.pose.position.x;
      pixhawk_odom_offset_y_ = _init_gps_offset_y_ - odom_pixhawk_.pose.pose.position.y;
    }

    ROS_INFO("[Odometry]: pixhawk_odom_offset based in local_origin calculated as: x: %f, y: %f", pixhawk_odom_offset_x_, pixhawk_odom_offset_y_);

    got_pixhawk_odom_offset_ = true;
    return true;

    // initialize gps_origin to specified utm coordinates
  } else {

    {
      std::scoped_lock lock(mutex_odom_pixhawk_, mutex_pixhawk_utm_position_);

      pixhawk_odom_offset_x_ = (pixhawk_utm_position_x_ - odom_pixhawk_.pose.pose.position.x) - _utm_origin_x_;
      pixhawk_odom_offset_y_ = (pixhawk_utm_position_y_ - odom_pixhawk_.pose.pose.position.y) - _utm_origin_y_;
    }

    ROS_INFO("[Odometry]: pixhawk_odom_offset based in local_utm calculated as: x: %f, y: %f", pixhawk_odom_offset_x_, pixhawk_odom_offset_y_);

    got_pixhawk_odom_offset_ = true;
    return true;
  }

  return false;
}

//}

/* applyOdomOffset //{ */
nav_msgs::Odometry Odometry::applyOdomOffset(const nav_msgs::Odometry &msg, const tf2::Vector3 &pos_offset, const tf2::Quaternion &rot_offset) {
  nav_msgs::Odometry ret = msg;

  tf2::Vector3 v;
  tf2::fromMsg(msg.pose.pose.position, v);
  v = tf2::quatRotate(rot_offset.inverse(), (v - pos_offset));
  /* v = v - pos_offset; */
  tf2::toMsg(v, ret.pose.pose.position);

  tf2::Quaternion q         = mrs_lib::AttitudeConverter(msg.pose.pose.orientation);
  q                         = rot_offset.inverse() * q;
  ret.pose.pose.orientation = mrs_lib::AttitudeConverter(q);

  return ret;
}

//}

/*isValidGate() //{*/
bool Odometry::isValidGate(const double &value, const double &min_value, const double &max_value, const std::string &value_name) {

  // Min value check
  if (value < min_value) {
    if (value_name != "") {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: %s value %f < %f is not valid.", value_name.c_str(), value, min_value);
    }
    return false;
  }

  // Max value check
  if (value > max_value) {
    if (value_name != "") {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: %s value %f > %f is not valid.", value_name.c_str(), value, max_value);
    }
    return false;
  }

  return true;
}
/*//}*/

/* saturateCorrection() //{ */

double Odometry::saturateCorrection(const double &correction, const double max_correction, const std::string &correction_name) {

  if (correction > max_correction) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating %s correction %f -> %f", correction_name.c_str(), correction, max_correction);
    return max_correction;
  } else if (correction < -max_correction) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating %s correction %f -> %f", correction_name.c_str(), correction, -max_correction);
    return -max_correction;
  }
  return correction;
}

//}

// | ------------------ Call service routines ----------------- |

/* callEnableControlCallbacks //{ */

bool Odometry::callEnableControlCallbacks() {

  // Enable control callbacks
  ROS_INFO("[Odometry]: Calling enable callbacks service");
  std_srvs::SetBool enable_callbacks_srv;
  enable_callbacks_srv.request.data = true;
  ser_client_enable_callbacks_.call(enable_callbacks_srv);

  if (enable_callbacks_srv.response.success) {
    ROS_INFO("[Odometry]: Enable callbacks service called successfully: %s", enable_callbacks_srv.response.message.c_str());
    return true;
  } else {
    ROS_INFO("[Odometry]: Enable callbacks service call failed: %s", enable_callbacks_srv.response.message.c_str());
    return false;
  }
}

//}

/* callDisableControlCallbacks //{ */

bool Odometry::callDisableControlCallbacks() {

  // Disable control callbacks
  ROS_INFO("[Odometry]: Calling disable callbacks service");
  std_srvs::SetBool disable_callbacks_srv;
  disable_callbacks_srv.request.data = true;
  ser_client_enable_callbacks_.call(disable_callbacks_srv);

  if (disable_callbacks_srv.response.success) {
    ROS_INFO("[Odometry]: Disable callbacks service called successfully: %s", disable_callbacks_srv.response.message.c_str());
    return true;
  } else {
    ROS_INFO("[Odometry]: Disable callbacks service call failed: %s", disable_callbacks_srv.response.message.c_str());
    return false;
  }
}

//}

/* callHover() //{ */

bool Odometry::callHover() {

  ROS_INFO("[Odometry]: Calling hover service.");
  std_srvs::Trigger hover_srv;
  ser_client_hover_.call(hover_srv);

  if (hover_srv.response.success) {
    ROS_INFO("[Odometry]: Hover service called successfully: %s", hover_srv.response.message.c_str());
    return true;
  } else {
    ROS_INFO("[Odometry]: Hover service call failed: %s", hover_srv.response.message.c_str());
    return false;
  }
}

//}

/* callMpcController() //{ */

bool Odometry::callMpcController() {

  ROS_INFO("[Odometry]: Calling MpcController service.");
  mrs_msgs::String mpc_controller_srv;
  mpc_controller_srv.request.value = "MpcController";
  ser_client_controller_.call(mpc_controller_srv);

  if (mpc_controller_srv.response.success) {
    ROS_INFO("[Odometry]: MpcController service called successfully: %s", mpc_controller_srv.response.message.c_str());
    return true;
  } else {
    ROS_INFO("[Odometry]: MpcController service call failed: %s", mpc_controller_srv.response.message.c_str());
    return false;
  }
}

//}

}  // namespace mrs_uav_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_odometry::Odometry, nodelet::Nodelet)
