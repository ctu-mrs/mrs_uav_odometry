#define VERSION "0.0.5.0"

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
#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/attitude_converter.h>

#include <types.h>
#include <support.h>
#include <StateEstimator.h>
#include <AltitudeEstimator.h>
#include <HeadingEstimator.h>
#include <mrs_uav_odometry/odometry_dynparamConfig.h>

#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <locale>
#include <Eigen/Eigen>
#include <math.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <queue>

//}

#define btoa(x) ((x) ? "true" : "false")
#define NAME_OF(v) #v

namespace mrs_uav_odometry
{

/* //{ class Odometry */

class Odometry : public nodelet::Nodelet {

public:
  std::string  _version_;
  virtual void onInit();

public:
  void publishMessage();
  int  _main_rate_;
  bool is_initialized_      = false;
  bool is_ready_to_takeoff_ = false;

private:
  std::string _uav_name_;
  bool        _simulation_ = false;

  bool   _publish_fused_odom_;
  bool   _publish_local_origin_stable_tf_;
  bool   _publish_pixhawk_velocity_;
  bool   _dynamic_optflow_cov_       = false;
  double _dynamic_optflow_cov_scale_ = 0;
  double twist_q_x_prev              = 0;
  double twist_q_y_prev              = 0;

  double _max_optflow_altitude_;
  double _max_brick_altitude_;
  double _max_plane_altitude_;
  double _max_default_altitude_;

  ros::NodeHandle nh;

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
  ros::Publisher pub_compass_yaw_;
  ros::Publisher pub_brick_yaw_;
  ros::Publisher pub_hector_yaw_;
  ros::Publisher pub_tower_yaw_;
  ros::Publisher pub_aloam_yaw_;
  ros::Publisher pub_lidar_yaw_;
  ros::Publisher pub_vio_yaw_;
  ros::Publisher pub_vslam_yaw_;
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

  ros::Publisher pub_debug_optflow_filter;
  ros::Publisher pub_debug_icp_twist_filter;

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
  ros::Subscriber sub_lidar_odom_;
  ros::Subscriber sub_icp_twist_;
  ros::Subscriber sub_hector_pose_;
  ros::Subscriber sub_tower_pose_;
  ros::Subscriber sub_aloam_odom_;
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
  ros::ServiceServer ser_toggle_rtk_altitude;
  ros::ServiceServer ser_change_odometry_source;
  ros::ServiceServer ser_change_estimator_type;
  ros::ServiceServer ser_change_estimator_type_string;
  ros::ServiceServer ser_change_hdg_estimator_type;
  ros::ServiceServer ser_change_hdg_estimator_type_string;
  ros::ServiceServer ser_change_alt_estimator_type;
  ros::ServiceServer ser_change_alt_estimator_type_string;
  ros::ServiceServer ser_gyro_jump_;
  ros::ServiceServer ser_toggle_callbacks_;

  ros::ServiceClient ser_client_failsafe_;
  ros::ServiceClient ser_client_hover_;
  ros::ServiceClient ser_client_reference_;
  ros::ServiceClient ser_client_ehover_;
  ros::ServiceClient ser_client_tracker_;
  ros::ServiceClient ser_client_controller_;
  ros::ServiceClient ser_client_enable_callbacks_;

private:
  tf2_ros::TransformBroadcaster *             broadcaster_;
  tf2_ros::Buffer                             m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  mrs_lib::Transformer                        transformer_;

  dynamic_reconfigure::Server<mrs_uav_odometry::odometry_dynparamConfig>               odometry_dynparam_server;
  dynamic_reconfigure::Server<mrs_uav_odometry::odometry_dynparamConfig>::CallbackType callback_odometry_dynparam_server;

  nav_msgs::Odometry odom_pixhawk;
  nav_msgs::Odometry odom_pixhawk_previous;
  nav_msgs::Odometry odom_pixhawk_shifted;
  nav_msgs::Odometry odom_pixhawk_previous_shifted;
  double             init_magnetic_heading_ = 0.0;
  double             init_brick_yaw_        = 0.0;
  double             yaw_diff_              = 0.0;
  ros::Publisher     pub_odom_mavros_;

  nav_msgs::Odometry odom_main_inno;
  std::mutex         mutex_odom_main_inno;
  std::mutex         mutex_uav_state;


  // PIXHAWK
  std::mutex  mutex_odom_pixhawk;
  std::mutex  mutex_odom_pixhawk_shifted;
  ros::Time   odom_pixhawk_last_update;
  std::mutex  mutex_gps_local_odom;
  std::mutex  mutex_gps_covariance_;
  double      gps_covariance_                       = 0.0;
  double      _gps_fallback_covariance_limit_       = 0.0;
  double      _gps_fallback_covariance_ok_          = 0.0;
  std::string _gps_fallback_estimator_              = "OPTFLOW";
  bool        _gps_fallback_allowed_                = false;
  bool        _gps_return_after_fallback_           = false;
  bool        gps_in_fallback_                      = false;
  int         c_gps_cov_over_lim_                   = 0;
  int         c_gps_cov_ok_                         = 0;
  int         _gps_fallback_bad_samples_            = 0;
  int         _gps_fallback_good_samples_           = 0;
  double      _gps_fallback_altitude_               = 4.0;
  double      _gps_fallback_wait_for_altitude_time_ = 5.0;

  double des_yaw_rate_ = 0.0;
  double des_yaw_      = 0.0;

  std::vector<nav_msgs::Odometry> vec_odom_aux;

  // OPTFLOW
  geometry_msgs::TwistWithCovarianceStamped optflow_twist;
  std::mutex                                mutex_optflow;
  geometry_msgs::TwistWithCovarianceStamped optflow_twist_previous;
  ros::Time                                 optflow_twist_last_update;
  std::shared_ptr<MedianFilter>             optflow_filter_x;
  std::shared_ptr<MedianFilter>             optflow_filter_y;
  bool                                      _optflow_median_filter_;
  int                                       _optflow_filter_buffer_size_;
  double                                    _optflow_filter_max_valid_;
  double                                    _optflow_filter_max_diff_;
  bool                                      fusing_optflow_low_ = true;
  bool                                      _use_optflow_low_   = false;

  // VIO
  nav_msgs::Odometry odom_vio;
  std::mutex         mutex_odom_vio;
  nav_msgs::Odometry odom_vio_previous;
  ros::Time          odom_vio_last_update;

  // VSLAM
  geometry_msgs::PoseWithCovarianceStamped pose_vslam;
  std::mutex                               mutex_pose_vslam;
  geometry_msgs::PoseWithCovarianceStamped pose_vslam_previous;
  ros::Time                                pose_vslam_last_update;

  // Realsense t265
  nav_msgs::Odometry odom_t265;
  std::mutex         mutex_odom_t265;
  nav_msgs::Odometry odom_t265_previous;
  ros::Time          odom_t265_last_update;

  // brick msgs
  nav_msgs::Odometry odom_brick;
  std::mutex         mutex_odom_brick;
  nav_msgs::Odometry odom_brick_previous;
  ros::Time          odom_brick_last_update;
  int                counter_odom_brick;
  int                c_failed_brick_x_       = 0;
  int                c_failed_brick_y_       = 0;
  int                c_failed_brick_timeout_ = 0;
  int                c_failed_brick_yaw_     = 0;

  // IMU msgs
  sensor_msgs::Imu pixhawk_imu;
  sensor_msgs::Imu pixhawk_imu_previous;
  std::mutex       mutex_pixhawk_imu;
  ros::Time        pixhawk_imu_last_update;

  // Control acceleration msgs
  sensor_msgs::Imu       control_accel;
  sensor_msgs::Imu       control_accel_previous;
  std::mutex             mutex_control_accel;
  ros::Time              control_accel_last_update;
  geometry_msgs::Vector3 acc_global_prev_;

  // Compass msgs
  std_msgs::Float64 compass_hdg;
  std_msgs::Float64 compass_hdg_previous;
  double            yaw_previous;
  std::mutex        mutex_compass_hdg;
  ros::Time         compass_hdg_last_update;

  // Hector heading msgs
  double                        hector_yaw_previous;
  std::mutex                    mutex_hector_hdg;
  ros::Time                     hector_yaw_last_update;
  std::shared_ptr<MedianFilter> hector_yaw_filter;
  bool                          _hector_yaw_median_filter;
  int                           _hector_yaw_filter_buffer_size_;
  double                        _hector_yaw_filter_max_valid;
  double                        _hector_yaw_filter_max_diff_;

  // TOWER heading msgs
  double                        tower_yaw_previous;
  std::mutex                    mutex_tower_hdg;
  ros::Time                     tower_yaw_last_update;
  std::shared_ptr<MedianFilter> tower_yaw_filter;
  bool                          _tower_yaw_median_filter;
  int                           _tower_yaw_filter_buffer_size_;
  double                        _tower_yaw_filter_max_valid;
  double                        _tower_yaw_filter_max_diff_;

  // ALOAM heading msgs
  double                        aloam_yaw_previous;
  std::shared_ptr<MedianFilter> aloam_yaw_filter;
  int                           _aloam_yaw_filter_buffer_size_;
  double                        _aloam_yaw_filter_max_diff_;

  // Lidar heading msgs
  double                        lidar_yaw_previous;
  std::mutex                    mutex_lidar_hdg;
  ros::Time                     lidar_yaw_last_update;
  std::shared_ptr<MedianFilter> lidar_yaw_filter;
  bool                          _lidar_yaw_median_filter;
  int                           _lidar_yaw_filter_buffer_size;
  double                        _lidar_yaw_filter_max_valid;
  double                        _lidar_yaw_filter_max_diff;

  // Aloam heading messages
  std::mutex                    mutex_aloam;
  double                        pos_aloam_x, pos_aloam_y;
  nav_msgs::Odometry            aloam_odom;
  nav_msgs::Odometry            aloam_odom_previous;
  ros::Time                     aloam_odom_last_update;
  std::shared_ptr<MedianFilter> aloam_pos_filter_x;
  std::shared_ptr<MedianFilter> aloam_pos_filter_y;
  bool                          _aloam_pos_median_filter_;
  int                           _aloam_pos_filter_buffer_size_;
  double                        _aloam_pos_filter_max_valid_;
  double                        _aloam_pos_filter_max_diff_;
  Vec2                          aloam_offset_;
  Vec2                          aloam_vel_state_;
  double                        aloam_offset_hdg_;

  // brick heading msgs
  double                        brick_yaw_previous;
  std::mutex                    mutex_brick_hdg;
  ros::Time                     brick_yaw_last_update;
  std::shared_ptr<MedianFilter> brick_yaw_filter;
  bool                          _brick_yaw_median_filter;
  int                           _brick_yaw_filter_buffer_size_;
  double                        _brick_yaw_filter_max_valid;
  double                        _brick_yaw_filter_max_diff_;
  double                        accum_yaw_brick_;
  double                        _accum_yaw_brick_alpha_;

  // VIO heading msgs
  double                        vio_yaw_previous;
  std::mutex                    mutex_vio_hdg;
  ros::Time                     vio_yaw_last_update;
  std::shared_ptr<MedianFilter> vio_yaw_filter;
  bool                          _vio_yaw_median_filter;
  int                           _vio_yaw_filter_buffer_size;
  double                        _vio_yaw_filter_max_valid;
  double                        _vio_yaw_filter_max_diff;

  // VSLAM heading msgs
  double                        vslam_yaw_previous;
  std::mutex                    mutex_vslam_hdg;
  ros::Time                     vslam_yaw_last_update;
  std::shared_ptr<MedianFilter> vslam_yaw_filter;
  bool                          _vslam_yaw_median_filter;
  int                           _vslam_yaw_filter_buffer_size;
  double                        _vslam_yaw_filter_max_valid;
  double                        _vslam_yaw_filter_max_diff;

  geometry_msgs::Vector3Stamped orientation_mavros;
  geometry_msgs::Vector3Stamped orientation_gt;

  double     _uav_mass_estimate_;
  std::mutex mutex_uav_mass_estimate_;

  // Target attitude msgs
  mrs_msgs::AttitudeCommand attitude_command_;
  mrs_msgs::AttitudeCommand attitude_command_prev_;
  ros::Time                 attitude_command_last_update_;
  std::mutex                mutex_attitude_command_;

  // RTK
  std::mutex       mutex_rtk;
  mrs_msgs::RtkGps rtk_odom_previous;
  mrs_msgs::RtkGps rtk_odom;
  ros::Time        rtk_last_update;
  bool             _rtk_fuse_sps_;


  // LIDAR messages
  std::mutex                    mutex_lidar;
  double                        pos_lidar_x, pos_lidar_y;
  nav_msgs::Odometry            lidar_odom;
  nav_msgs::Odometry            lidar_odom_previous;
  ros::Time                     lidar_odom_last_update;
  std::shared_ptr<MedianFilter> lidar_vel_filter_x;
  std::shared_ptr<MedianFilter> lidar_vel_filter_y;
  bool                          _lidar_vel_median_filter_;
  int                           _lidar_vel_filter_buffer_size_;
  double                        _lidar_vel_filter_max_valid_;
  double                        _lidar_vel_filter_max_diff_;

  // Hector messages
  std::mutex                    mutex_hector;
  double                        pos_hector_x, pos_hector_y;
  geometry_msgs::PoseStamped    hector_pose;
  geometry_msgs::PoseStamped    hector_pose_previous;
  ros::Time                     hector_pose_last_update;
  std::shared_ptr<MedianFilter> hector_pos_filter_x;
  std::shared_ptr<MedianFilter> hector_pos_filter_y;
  bool                          _hector_pos_median_filter_;
  int                           _hector_pos_filter_buffer_size_;
  double                        _hector_pos_filter_max_valid_;
  double                        _hector_pos_filter_max_diff_;
  bool                          hector_reset_called_         = false;
  bool                          _reset_hector_after_takeoff_ = false;
  Vec2                          hector_offset_;
  Vec2                          hector_vel_state_;
  double                        hector_offset_hdg_;
  int                           c_hector_msg_;

  // TOWER messages
  std::mutex                    mutex_tower;
  double                        pos_tower_x, pos_tower_y;
  geometry_msgs::PoseStamped    tower_pose;
  geometry_msgs::PoseStamped    tower_pose_previous;
  ros::Time                     tower_pose_last_update;
  std::shared_ptr<MedianFilter> tower_pos_filter_x;
  std::shared_ptr<MedianFilter> tower_pos_filter_y;
  bool                          _tower_pos_median_filter_;
  int                           _tower_pos_filter_buffer_size_;
  double                        _tower_pos_filter_max_valid_;
  double                        _tower_pos_filter_max_diff_;
  int                           c_tower_msg_;

  // VIO messages
  std::mutex                                mutex_icp_twist;
  geometry_msgs::TwistWithCovarianceStamped icp_twist;
  geometry_msgs::TwistWithCovarianceStamped icp_twist_previous;
  ros::Time                                 icp_twist_last_update;
  std::shared_ptr<MedianFilter>             icp_twist_filter_x;
  std::shared_ptr<MedianFilter>             icp_twist_filter_y;
  bool                                      _icp_twist_median_filter_;
  int                                       _icp_twist_filter_buffer_size_;
  double                                    _icp_twist_filter_max_valid_;
  double                                    _icp_twist_filter_max_diff_;
  std::shared_ptr<MedianFilter>             icp_yaw_rate_filter;
  int                                       _icp_yaw_rate_filter_buffer_size_;
  double                                    _icp_yaw_rate_filter_max_valid_;
  double                                    _icp_yaw_rate_filter_max_diff_;
  double                                    icp_yaw_rate_inconsistent_samples;

  // brick messages
  std::mutex                    mutex_brick;
  geometry_msgs::PoseStamped    brick_pose;
  geometry_msgs::PoseStamped    brick_pose_previous;
  ros::Time                     brick_pose_last_update;
  std::shared_ptr<MedianFilter> brick_pos_filter_x;
  std::shared_ptr<MedianFilter> brick_pos_filter_y;
  bool                          _brick_pos_median_filter_;
  int                           _brick_pos_filter_buffer_size_;
  double                        _brick_pos_filter_max_valid_;
  double                        _brick_pos_filter_max_diff_;
  double                        _brick_timeout_;

  std::mutex         mutex_ground_truth;
  nav_msgs::Odometry ground_truth;

  mrs_msgs::RtkGps rtk_local_previous;
  mrs_msgs::RtkGps rtk_local;

  bool                     _is_estimator_tmp;
  mrs_msgs::EstimatorType  _estimator_type;
  mrs_msgs::EstimatorType  fallback_brick_estimator_type;
  mrs_msgs::HeadingType    fallback_brick_hdg_estimator_type;
  mrs_msgs::EstimatorType  _estimator_type_takeoff;
  std::vector<std::string> _estimator_type_names;
  std::vector<std::string> _altitude_type_names;
  std::string              altitude_estimator_name;
  std::mutex               mutex_estimator_type;
  std::mutex               mutex_alt_estimator_type;
  int                      estimator_iteration_;

  std::string child_frame_id;
  std::mutex  mutex_odom_local;
  std::string fcu_frame_id_;
  std::string fcu_untilted_frame_id_;
  std::string local_origin_frame_id_;
  std::string stable_origin_frame_id_;
  std::string last_stable_name_;
  std::string last_local_name_;
  std::string first_frame_;

  bool   got_fcu_untilted_ = false;
  bool   got_init_heading  = false;
  double m_init_heading;

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
  void callbackLidarOdom(const nav_msgs::OdometryConstPtr &msg);
  void callbackHectorPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackTowerPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackAloamOdom(const nav_msgs::OdometryConstPtr &msg);
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
  bool callbackGyroJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackToggleCallbacks(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // | --------------------- helper methods --------------------- |
  bool isReadyToTakeoff();
  void stateEstimatorsPrediction(const geometry_msgs::Vector3 &acc_in, double dt);
  void stateEstimatorsCorrection(double x, double y, const std::string &measurement_name);
  bool changeCurrentEstimator(const mrs_msgs::EstimatorType &desired_estimator);

  void altitudeEstimatorCorrection(double value, const std::string &measurement_name);
  void altitudeEstimatorCorrection(double value, const std::string &measurement_name, const std::shared_ptr<mrs_uav_odometry::AltitudeEstimator> &estimator);
  bool changeCurrentAltitudeEstimator(const mrs_msgs::AltitudeType &desired_estimator);

  void headingEstimatorsPrediction(const double yaw, const double yaw_rate, const double dt);
  void headingEstimatorsCorrection(const double value, const std::string &measurement_name);
  bool changeCurrentHeadingEstimator(const mrs_msgs::HeadingType &desired_estimator);

  void               getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz);
  double             getGlobalZAcceleration(const geometry_msgs::Quaternion &q_msg, const double &acc_z_in);
  void               getRotatedTilt(const geometry_msgs::Quaternion &q_msg, const double &yaw, double &rx, double &ry);
  void               getRotatedVector(const geometry_msgs::Vector3 &acc_in, double yaw_in, geometry_msgs::Vector3 &acc_out);
  void               rotateLateralStates(const double yaw_new, const double yaw_old);
  double             getCurrentHeading();
  bool               isValidType(const mrs_msgs::EstimatorType &type);
  bool               isValidType(const mrs_msgs::HeadingType &type);
  bool               isValidType(const mrs_msgs::AltitudeType &type);
  bool               isTimestampOK(const double curr_sec, const double prev_sec);
  std::string        printOdometryDiag();
  bool               stringInVector(const std::string &value, const std::vector<std::string> &vector);
  nav_msgs::Odometry applyOdomOffset(const nav_msgs::Odometry &msg, const tf2::Vector3 &pos_offset, const tf2::Quaternion &rot_offset);

  // | ------------------ call service routines ----------------- |
  bool callEnableControlCallbacks();
  bool callDisableControlCallbacks();
  bool callHover();
  bool callMpcController();

  nav_msgs::Odometry shared_odom;
  std::mutex         mutex_shared_odometry;

  nav_msgs::Odometry rtk_local_odom;
  std::mutex         mutex_rtk_local_odom;

  nav_msgs::Odometry odom_local;
  tf2::Vector3       m_pos_odom_offset;
  tf2::Quaternion    m_rot_odom_offset;

  nav_msgs::Odometry odom_stable;
  tf2::Vector3       odom_stable_pos_offset_;
  tf2::Quaternion    odom_stable_rot_offset_;

  // Garmin altitude subscriber and callback
  ros::Subscriber               sub_garmin_;
  sensor_msgs::Range            range_garmin;
  sensor_msgs::Range            range_garmin_previous;
  std::mutex                    mutex_range_garmin;
  void                          callbackGarmin(const sensor_msgs::RangeConstPtr &msg);
  std::shared_ptr<MedianFilter> garminFilter;
  int                           _garmin_filter_buffer_size_;
  double                        _garmin_max_valid_altitude_;
  double                        _garmin_filter_max_difference_;
  ros::Time                     garmin_last_update;
  bool                          saturate_garmin_corrections_ = false;
  double                        _garmin_inno_gate_value_sq_;
  bool                          _use_garmin_inno_gate_;
  double                        _garmin_min_altitude_;
  double                        _garmin_max_altitude_;


  bool   callbacks_enabled_ = false;
  bool   baro_corrected_    = false;
  double baro_offset_       = 0.0;

  // sonar altitude subscriber and callback
  ros::Subscriber               sub_sonar_;
  sensor_msgs::Range            range_sonar;
  sensor_msgs::Range            range_sonar_previous;
  std::mutex                    mutex_range_sonar;
  void                          callbackSonar(const sensor_msgs::RangeConstPtr &msg);
  std::shared_ptr<MedianFilter> sonarFilter;
  int                           _sonar_filter_buffer_size_;
  double                        _sonar_max_valid_altitude_;
  double                        _sonar_filter_max_difference_;
  ros::Time                     sonar_last_update;

  // Plane altitude subscriber and callback
  ros::Subscriber               sub_plane_;
  sensor_msgs::Range            range_plane;
  sensor_msgs::Range            range_plane_previous;
  std::mutex                    mutex_range_plane;
  void                          callbackPlane(const sensor_msgs::RangeConstPtr &msg);
  std::shared_ptr<MedianFilter> planeFilter;
  int                           _plane_filter_buffer_size_;
  double                        _plane_max_valid_altitude_;
  double                        _plane_filter_max_difference_;
  ros::Time                     plane_last_update;

  // Brick altitude subscriber and callback
  std::shared_ptr<MedianFilter> brickHeightFilter;
  int                           _brick_filter_buffer_size_;
  double                        _brick_max_valid_altitude_;
  double                        _brick_filter_max_difference_;
  ros::Time                     brick_last_update;

  // VIO altitude subscriber and callback
  std::shared_ptr<MedianFilter> vioHeightFilter;
  int                           _vio_filter_buffer_size_;
  double                        _vio_max_valid_altitude_;
  double                        _vio_filter_max_difference_;
  ros::Time                     vio_last_update;

  // ALOAM height median filter
  std::shared_ptr<MedianFilter> aloamHeightFilter;
  int                           _aloam_height_filter_buffer_size_;
  double                        _aloam_height_max_valid_altitude_;
  double                        _aloam_height_filter_max_difference_;

  bool got_odom_pixhawk     = false;
  bool got_odom_t265        = false;
  bool got_optflow          = false;
  bool got_range            = false;
  bool got_plane            = false;
  bool got_pixhawk_utm      = false;
  bool got_rtk              = false;
  bool got_lidar_odom       = false;
  bool got_hector_pose      = false;
  bool got_tower_pose       = false;
  bool got_aloam_odom       = false;
  bool got_brick_pose       = false;
  bool got_attitude_command = false;
  bool got_vio              = false;
  bool got_vslam            = false;
  bool got_altitude_sensors = false;
  bool got_lateral_sensors  = false;
  bool got_rtk_fix          = false;
  bool got_pixhawk_imu      = false;
  bool got_compass_hdg      = false;
  bool got_control_accel    = false;
  bool got_icp_twist        = false;

  bool failsafe_called = false;

  int  got_lidar_odom_counter;
  int  got_rtk_counter;
  bool got_rtk_local_origin_z;

  bool rtk_odom_initialized = false;

  geometry_msgs::Vector3Stamped des_attitude_global;

  // for setting home position
  double _utm_origin_x_, _utm_origin_y_;
  bool   _use_utm_origin_   = false;
  int    _utm_origin_units_ = 0;
  double rtk_local_origin_z_;
  double _local_origin_x_, _local_origin_y_;
  double land_position_x, land_position_y;
  bool   land_position_set = false;

  // current position in UTM as measure by pixhawk
  double     pixhawk_utm_position_x, pixhawk_utm_position_y;
  std::mutex mutex_pixhawk_utm_position;

  // subscribing to tracker status
  mrs_msgs::ControlManagerDiagnostics control_manager_diag_;
  std::mutex                          mutex_control_manager_diag_;
  void                                callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnosticsConstPtr &msg);
  bool                                got_control_manager_diag_ = false;
  bool                                isUavFlying();
  bool                                isUavLandoff();
  bool                                uav_in_the_air = false;
  std::string                         _null_tracker_;

  // offset to adjust the local origin
  double pixhawk_odom_offset_x, pixhawk_odom_offset_y;
  bool   got_pixhawk_odom_offset  = false;
  bool   got_pixhawk_odom_shifted = false;

  geometry_msgs::Vector3 mavros_glitch;

  // initial position
  double init_pose_x   = 0.0;
  double init_pose_y   = 0.0;
  double init_pose_yaw = 0.0;

  // heading estimation
  int                                                      _heading_n_, _heading_m_, _heading_p_;
  Eigen::MatrixXd                                          A_hdg, B_hdg, R_hdg;
  std::mutex                                               mutex_heading_estimator;
  std::mutex                                               mutex_hdg_estimator_type;
  std::vector<std::string>                                 _heading_estimators_names_;
  std::vector<std::string>                                 _active_heading_estimators_names_;
  std::vector<std::string>                                 _hdg_model_state_names_;
  std::vector<std::string>                                 _hdg_measurement_names_;
  std::map<std::string, std::vector<std::string>>          map_hdg_estimator_measurement;
  std::map<std::string, Eigen::MatrixXd>                   map_hdg_measurement_covariance;
  std::map<std::string, std::string>                       map_hdg_measurement_state;
  std::map<std::string, int>                               map_hdg_measurement_name_id;
  std::map<std::string, Eigen::MatrixXd>                   map_hdg_states;
  std::map<std::string, mrs_msgs::Float64ArrayStamped>     map_hdg_estimator_msg;
  std::map<std::string, ros::Publisher>                    map_hdg_estimator_pub;
  std::map<std::string, std::shared_ptr<HeadingEstimator>> m_heading_estimators;
  std::shared_ptr<HeadingEstimator>                        current_hdg_estimator;
  std::string                                              current_hdg_estimator_name;
  mrs_msgs::HeadingType                                    _hdg_estimator_type;
  mrs_msgs::HeadingType                                    _hdg_estimator_type_takeoff;
  std::vector<std::string>                                 _heading_type_names;
  std::string                                              _heading_estimator_name_;
  std::mutex                                               mutex_current_hdg_estimator;
  int                                                      _optflow_yaw_rate_filter_buffer_size_;
  double                                                   _optflow_yaw_rate_filter_max_valid_;
  double                                                   _optflow_yaw_rate_filter_max_diff_;
  bool                                                     init_hdg_avg_done;
  int                                                      init_hdg_avg_samples;
  double                                                   init_hdg_avg;

  int    _compass_yaw_filter_buffer_size_;
  double _compass_yaw_filter_max_diff_;

  std::shared_ptr<MedianFilter> optflow_yaw_rate_filter;
  std::shared_ptr<MedianFilter> compass_yaw_filter;
  int                           compass_inconsistent_samples;
  int                           optflow_inconsistent_samples;
  bool                          is_heading_estimator_initialized = false;
  bool                          _gyro_fallback_;

  // altitude estimation
  int                                                       _altitude_n_, _altitude_m_, _altitude_p_;
  Eigen::MatrixXd                                           _B_alt_, _R_alt_;
  std::mutex                                                mutex_altitude_estimator;
  std::vector<std::string>                                  _altitude_estimators_names_;
  std::vector<std::string>                                  _alt_model_state_names_;
  std::vector<std::string>                                  _alt_measurement_names_;
  std::map<std::string, std::vector<std::string>>           map_alt_estimator_measurement;
  std::map<std::string, Eigen::MatrixXd>                    map_alt_model;
  std::map<std::string, int>                                map_alt_n_states;
  std::map<std::string, Eigen::MatrixXd>                    map_alt_measurement_covariance;
  std::map<std::string, std::string>                        map_alt_measurement_state;
  std::map<std::string, int>                                map_alt_measurement_name_id;
  std::map<std::string, Eigen::MatrixXd>                    map_alt_states;
  std::map<std::string, mrs_msgs::Float64Stamped>           map_alt_estimator_msg;
  std::map<std::string, ros::Publisher>                     map_alt_estimator_pub;
  std::map<std::string, std::shared_ptr<AltitudeEstimator>> m_altitude_estimators;
  std::shared_ptr<AltitudeEstimator>                        current_alt_estimator;
  std::string                                               current_alt_estimator_name;
  mrs_msgs::AltitudeType                                    _alt_estimator_type;
  mrs_msgs::AltitudeType                                    _alt_estimator_type_takeoff;
  std::mutex                                                mutex_current_alt_estimator;
  bool                                                      is_altitude_estimator_initialized = false;
  bool                                                      is_lateral_estimator_initialized  = false;
  int                                                       counter_altitude                  = 0;
  double                                                    _excessive_tilt_sq_;
  int                                                       current_altitude_type;

  // State estimation
  std::vector<std::string>                               _state_estimators_names_;
  std::vector<std::string>                               _active_state_estimators_names_;
  std::vector<std::string>                               _model_state_names_;
  std::vector<std::string>                               _measurement_names_;
  std::map<std::string, std::vector<std::string>>        map_estimator_measurement;
  std::map<std::string, Mat1>                            map_measurement_covariance;
  std::map<std::string, std::string>                     map_measurement_state;
  std::map<std::string, int>                             map_measurement_name_id;
  std::map<std::string, LatStateCol1D>                   map_states;
  std::map<std::string, nav_msgs::Odometry>              map_estimator_odom;
  std::map<std::string, ros::Publisher>                  map_estimator_pub;
  std::map<std::string, std::shared_ptr<StateEstimator>> m_state_estimators;
  std::shared_ptr<StateEstimator>                        current_estimator;
  std::mutex                                             mutex_current_estimator;
  std::string                                            current_estimator_name;

  bool   _saturate_mavros_position_;
  double _max_mavros_pos_correction_;
  double _max_vio_pos_correction_;
  double _max_vslam_pos_correction_;
  double _max_brick_pos_correction_;
  double _max_brick_yaw_correction_;
  double _max_rtk_pos_correction_;
  double _max_t265_vel_;
  double max_safe_brick_jump_sq_;
  double max_safe_brick_yaw_jump_sq_;

  int           _lateral_n_, _lateral_m_, _lateral_p_;
  LatMat        _A_lat_, _R_lat_;
  LatStateCol1D _B_lat_;

  // RTK LKF
  using lkf_rtk_t = mrs_lib::LKF<2, 2, 2>;
  lkf_rtk_t::A_t        _A_lat_rtk_;
  lkf_rtk_t::B_t        _B_lat_rtk_;
  lkf_rtk_t::H_t        _H_lat_rtk_;
  lkf_rtk_t::R_t        _R_lat_rtk_;
  lkf_rtk_t::Q_t        _Q_lat_rtk_;
  lkf_rtk_t::P_t        _P_lat_rtk_;
  lkf_rtk_t::statecov_t sc_lat_rtk_;

  std::shared_ptr<lkf_rtk_t> estimator_rtk_;
  std::mutex                 mutex_rtk_est_;

  bool got_home_position_fix = false;
  bool calculatePixhawkOdomOffset(void);

  bool odometry_published;

  double     max_altitude_ = 10;
  std::mutex mutex_max_altitude_;
  bool       gps_reliable        = false;
  bool       hector_reliable     = false;
  bool       tower_reliable      = false;
  bool       aloam_reliable      = false;
  bool       _gps_available_     = false;
  bool       _vio_available_     = false;
  bool       vio_reliable        = true;
  bool       _vslam_available_   = false;
  bool       vslam_reliable      = true;
  bool       optflow_reliable    = false;
  bool       _optflow_available_ = false;
  bool       _rtk_available_     = false;
  bool       rtk_reliable        = false;
  bool       _t265_available_    = false;
  bool       t265_reliable       = false;
  bool       _lidar_available_   = false;
  bool       _aloam_available_   = false;
  bool       _brick_available_   = false;
  bool       brick_reliable      = false;
  bool       height_available_   = false;
  bool       icp_reliable        = false;
  bool       plane_reliable      = false;
  bool       gps_active_         = true;

  bool      brick_semi_reliable = false;
  ros::Time brick_semi_reliable_started;

  bool   _pass_rtk_as_odom_ = false;
  double max_pos_correction_rate;
  double _max_altitude_correction_;

  // disabling rangefinders on the flight
  bool garmin_enabled;
  bool sonar_enabled;

  ros::Timer slow_odom_timer;
  ros::Timer diag_timer;
  ros::Timer lkf_states_timer;
  ros::Timer max_altitude_timer;
  ros::Timer topic_watcher_timer;
  ros::Timer hector_reset_routine_timer;
  bool       hector_reset_routine_running_;
  bool       _perform_hector_reset_routine_;
  int        _slow_odom_rate_;
  int        _aux_rate_;
  int        _diag_rate_;
  int        _lkf_states_rate_;
  int        _max_altitude_rate_;
  int        topic_watcher_rate_ = 1;
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
  bool       rtk_altitude_enabled;
  double     rtk_altitude_integral;
  ros::Timer rtk_rate_timer;
  double     rtkQ;
  double     rtk_max_down_difference_;
  double     rtk_max_abs_difference_;

private:
  ros::Timer main_timer, aux_timer;
  void       mainTimer(const ros::TimerEvent &event);
  void       auxTimer(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                        config_mutex_;
  typedef mrs_uav_odometry::odometry_dynparamConfig Config;
  typedef dynamic_reconfigure::Server<Config>   ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>          reconfigure_server_;
  mrs_uav_odometry::odometry_dynparamConfig         last_drs_config;
};

//}

/* //{ onInit() */

void Odometry::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Odometry]: initializing");

  mrs_lib::ParamLoader param_loader(nh, "Odometry");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Odometry]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  // establish frame ids
  param_loader.loadParam("uav_name", _uav_name_);
  fcu_frame_id_           = _uav_name_ + "/fcu";
  fcu_untilted_frame_id_  = _uav_name_ + "/fcu_untilted";
  local_origin_frame_id_  = _uav_name_ + "/local_origin";
  stable_origin_frame_id_ = _uav_name_ + "/stable_origin";
  last_local_name_        = _uav_name_ + "/null_origin";
  last_stable_name_       = _uav_name_ + "/null_origin";

  param_loader.loadParam("uav_mass", _uav_mass_estimate_);
  param_loader.loadParam("null_tracker", _null_tracker_);

  odometry_published        = false;
  got_odom_pixhawk          = false;
  got_optflow               = false;
  got_vio                   = false;
  got_vslam                 = false;
  got_brick_pose            = false;
  got_rtk                   = false;
  got_odom_t265             = false;
  got_rtk_fix               = false;
  got_pixhawk_utm           = false;
  got_control_manager_diag_ = false;
  got_home_position_fix     = false;
  got_altitude_sensors      = false;
  got_lateral_sensors       = false;
  got_pixhawk_imu           = false;
  got_compass_hdg           = false;
  got_icp_twist             = false;

  failsafe_called                = false;
  hector_reset_called_           = false;
  _reset_hector_after_takeoff_   = false;
  _perform_hector_reset_routine_ = false;
  hector_reset_routine_running_  = false;
  hector_offset_ << 0, 0;
  hector_offset_hdg_   = 0;
  c_hector_msg_        = 0;
  estimator_iteration_ = 0;

  aloam_offset_ << 0, 0;
  aloam_offset_hdg_ = 0;

  acc_global_prev_.x = 0.0;
  acc_global_prev_.y = 0.0;
  acc_global_prev_.z = 0.0;

  _is_estimator_tmp = false;
  got_rtk_counter   = 0;

  is_heading_estimator_initialized = false;
  init_hdg_avg_samples             = 0;
  init_hdg_avg                     = 0.0;
  init_hdg_avg_done                = false;

  is_updating_state_     = false;
  finished_state_update_ = false;
  // got_brick_altitude = false;

  pixhawk_utm_position_x = 0;
  pixhawk_utm_position_y = 0;

  rtk_local_origin_z_    = 0;
  got_rtk_local_origin_z = false;

  // ------------------------------------------------------------------------
  // |                        odometry estimator type                       |
  // ------------------------------------------------------------------------

  /* check estimator type to name conversion //{ */

  // prepare the array of names
  // IMPORTANT, update this with each update of the EstimatorType message
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOW));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::GPS));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOWGPS));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::RTK));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::LIDAR));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::VIO));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::BRICK));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::T265));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::HECTOR));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::BRICKFLOW));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::VSLAM));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::ICP));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::ALOAM));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::TOWER));

  ROS_WARN("[Odometry]: SAFETY Checking the EstimatorType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::EstimatorType::TYPE_COUNT; i++) {
    std::size_t found        = _estimator_type_names[i].find_last_of(":");
    _estimator_type_names[i] = _estimator_type_names[i].substr(found + 1);
    ROS_INFO("[Odometry]: _estimator_type[%d]=%s", i, _estimator_type_names[i].c_str());
  }

  // prepare the array of names
  // IMPORTANT, update this with each update of the AltitudeType message
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::HEIGHT));
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::PLANE));
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::BRICK));
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::VIO));
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::ALOAM));
  _altitude_type_names.push_back(NAME_OF(mrs_msgs::AltitudeType::BARO));

  ROS_WARN("[Odometry]: SAFETY Checking the AltitudeType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::AltitudeType::TYPE_COUNT; i++) {
    std::size_t found       = _altitude_type_names[i].find_last_of(":");
    _altitude_type_names[i] = _altitude_type_names[i].substr(found + 1);
    ROS_INFO("[Odometry]: _altitude_type[%d]=%s", i, _altitude_type_names[i].c_str());
  }

  // prepare the array of names
  // IMPORTANT, update this with each update of the HeadingType message
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::PIXHAWK));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::GYRO));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::COMPASS));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::OPTFLOW));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::HECTOR));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::BRICK));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::VIO));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::VSLAM));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::LIDAR));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::ICP));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::BRICKFLOW));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::ALOAM));
  _heading_type_names.push_back(NAME_OF(mrs_msgs::HeadingType::TOWER));

  ROS_WARN("[Odometry]: SAFETY Checking the HeadingType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::HeadingType::TYPE_COUNT; i++) {
    std::size_t found      = _heading_type_names[i].find_last_of(":");
    _heading_type_names[i] = _heading_type_names[i].substr(found + 1);
    ROS_INFO("[Odometry]: _heading_type[%d]=%s", i, _heading_type_names[i].c_str());
  }

  //}

  // | ------------------- parameters loading ------------------- |
  param_loader.loadParam("rate", _main_rate_);

  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("slow_odom_rate", _slow_odom_rate_);
  param_loader.loadParam("diag_rate", _diag_rate_);
  param_loader.loadParam("aux_rate", _aux_rate_);
  param_loader.loadParam("max_altitude_rate", _max_altitude_rate_);
  param_loader.loadParam("lkf_states_rate", _lkf_states_rate_);

  param_loader.loadParam("garminFilterBufferSize", _garmin_filter_buffer_size_);
  param_loader.loadParam("garminFilterMaxValidAltitude", _garmin_max_valid_altitude_);
  param_loader.loadParam("garminFilterMaxDifference", _garmin_filter_max_difference_);

  param_loader.loadParam("sonarFilterBufferSize", _sonar_filter_buffer_size_);
  param_loader.loadParam("sonarFilterMaxValidAltitude", _sonar_max_valid_altitude_);
  param_loader.loadParam("sonarFilterMaxDifference", _sonar_filter_max_difference_);

  param_loader.loadParam("planeFilterBufferSize", _plane_filter_buffer_size_);
  param_loader.loadParam("planeFilterMaxValidAltitude", _plane_max_valid_altitude_);
  param_loader.loadParam("planeFilterMaxDifference", _plane_filter_max_difference_);

  param_loader.loadParam("brickFilterBufferSize", _brick_filter_buffer_size_);
  param_loader.loadParam("brickFilterMaxValidAltitude", _brick_max_valid_altitude_);
  param_loader.loadParam("brickFilterMaxDifference", _brick_filter_max_difference_);

  param_loader.loadParam("vioFilterBufferSize", _vio_filter_buffer_size_);
  param_loader.loadParam("vioFilterMaxValidAltitude", _vio_max_valid_altitude_);
  param_loader.loadParam("vioFilterMaxDifference", _vio_filter_max_difference_);

  param_loader.loadParam("aloamHeightFilterBufferSize", _aloam_height_filter_buffer_size_);
  param_loader.loadParam("aloamHeightFilterMaxValidAltitude", _aloam_height_max_valid_altitude_);
  param_loader.loadParam("aloamHeightFilterMaxDifference", _aloam_height_filter_max_difference_);

  param_loader.loadParam("garmin_z_offset", _garmin_z_offset_);
  param_loader.loadParam("sonar_z_offset", _sonar_z_offset_);
  param_loader.loadParam("fcu_height", _fcu_height_);

  double garmin_inno_gate_value_tmp;
  param_loader.loadParam("garmin/use_inno_gate", _use_garmin_inno_gate_);
  param_loader.loadParam("garmin/inno_gate_value", garmin_inno_gate_value_tmp);
  _garmin_inno_gate_value_sq_ = std::pow(garmin_inno_gate_value_tmp, 2);
  param_loader.loadParam("garmin/min_valid_altitude", _garmin_min_altitude_);
  param_loader.loadParam("garmin/max_valid_altitude", _garmin_max_altitude_);

  // Optic flow
  param_loader.loadParam("use_optflow_low", _use_optflow_low_);
  param_loader.loadParam("max_optflow_altitude", _max_optflow_altitude_);
  param_loader.loadParam("max_brick_altitude", _max_brick_altitude_);
  param_loader.loadParam("max_plane_altitude", _max_plane_altitude_);
  param_loader.loadParam("max_default_altitude", _max_default_altitude_);
  max_altitude_ = _max_default_altitude_;
  param_loader.loadParam("lateral/dynamic_optflow_cov", _dynamic_optflow_cov_);
  param_loader.loadParam("lateral/dynamic_optflow_cov_scale", _dynamic_optflow_cov_scale_);

  // Localization sources availability
  param_loader.loadParam("gps_available", _gps_available_);
  param_loader.loadParam("vio_available", _vio_available_);
  param_loader.loadParam("vslam_available", _vslam_available_);
  param_loader.loadParam("optflow_available", _optflow_available_);
  param_loader.loadParam("rtk_available", _rtk_available_);
  param_loader.loadParam("t265_available", _t265_available_);
  param_loader.loadParam("lidar_available", _lidar_available_);
  param_loader.loadParam("aloam_available", _aloam_available_);
  param_loader.loadParam("brick_available", _brick_available_);
  gps_reliable       = _gps_available_;
  hector_reliable    = _lidar_available_;
  tower_reliable     = _lidar_available_;
  icp_reliable       = _lidar_available_;
  aloam_reliable     = _aloam_available_;
  brick_reliable     = _brick_available_;
  rtk_reliable       = _rtk_available_;
  t265_reliable      = _t265_available_;
  plane_reliable     = _brick_available_;
  counter_odom_brick = 0;

  // Takeoff type
  std::string _takeoff_estimator_;
  param_loader.loadParam("lateral_estimator", _takeoff_estimator_);

  std::transform(_takeoff_estimator_.begin(), _takeoff_estimator_.end(), _takeoff_estimator_.begin(), ::toupper);
  size_t pos = std::distance(_estimator_type_names.begin(), find(_estimator_type_names.begin(), _estimator_type_names.end(), _takeoff_estimator_));
  _estimator_type_takeoff.name = _takeoff_estimator_;
  _estimator_type_takeoff.type = (int)pos;

  param_loader.loadParam("use_utm_origin", _use_utm_origin_);

  // Load UTM origin either in UTM or LatLon units
  param_loader.loadParam("utm_origin_units", _utm_origin_units_);
  if (_utm_origin_units_ == 0) {
    ROS_INFO("[Odometry]: Loading UTM origin in UTM units.");
    param_loader.loadParam("utm_origin_x", _utm_origin_x_);
    param_loader.loadParam("utm_origin_y", _utm_origin_y_);
  } else {
    double lat, lon;
    ROS_INFO("[Odometry]: Loading UTM origin in LatLon units.");
    param_loader.loadParam("utm_origin_lat", lat);
    param_loader.loadParam("utm_origin_lon", lon);
    ROS_INFO("[Odometry]: Converted to UTM x: %f, y: %f.", _utm_origin_x_, _utm_origin_y_);
    mrs_lib::UTM(lat, lon, &_utm_origin_x_, &_utm_origin_y_);
  }

  param_loader.loadParam("local_origin_x", _local_origin_x_);
  param_loader.loadParam("local_origin_y", _local_origin_y_);

  pixhawk_odom_offset_x = 0;
  pixhawk_odom_offset_y = 0;

  param_loader.loadParam("hiccup_time_threshold", _hiccup_thr_);

  /* load parameters of altitude estimator //{ */

  param_loader.loadParam("altitude/numberOfVariables", _altitude_n_);
  param_loader.loadParam("altitude/numberOfInputs", _altitude_m_);
  param_loader.loadParam("altitude/numberOfMeasurements", _altitude_p_);

  param_loader.loadMatrixDynamic("altitude/B", _B_alt_, _altitude_n_, _altitude_m_);
  param_loader.loadMatrixDynamic("altitude/R", _R_alt_, _altitude_n_, _altitude_n_);

  param_loader.loadParam("altitude_estimators/model_states", _alt_model_state_names_);
  param_loader.loadParam("altitude_estimators/measurements", _alt_measurement_names_);
  param_loader.loadParam("altitude_estimators/altitude_estimators", _altitude_estimators_names_);

  double excessive_tilt_tmp;
  param_loader.loadParam("altitude/excessive_tilt", excessive_tilt_tmp);
  _excessive_tilt_sq_ = std::pow(excessive_tilt_tmp, 2);

  param_loader.loadParam("altitude_estimator", altitude_estimator_name);
  size_t pos_alt = std::distance(_altitude_type_names.begin(), find(_altitude_type_names.begin(), _altitude_type_names.end(), altitude_estimator_name));

  _alt_estimator_type_takeoff.name = altitude_estimator_name;
  _alt_estimator_type_takeoff.type = (int)pos_alt;

  garminFilter = std::make_shared<MedianFilter>(_garmin_filter_buffer_size_, _garmin_max_valid_altitude_, 0, _garmin_filter_max_difference_);
  sonarFilter  = std::make_shared<MedianFilter>(_sonar_filter_buffer_size_, _sonar_max_valid_altitude_, 0, _sonar_filter_max_difference_);
  planeFilter  = std::make_shared<MedianFilter>(_plane_filter_buffer_size_, _plane_max_valid_altitude_, 0, _plane_filter_max_difference_);
  brickHeightFilter =
      std::make_shared<MedianFilter>(_brick_filter_buffer_size_, _brick_max_valid_altitude_, -_brick_max_valid_altitude_, _brick_filter_max_difference_);
  vioHeightFilter = std::make_shared<MedianFilter>(_vio_filter_buffer_size_, _vio_max_valid_altitude_, -_vio_max_valid_altitude_, _vio_filter_max_difference_);
  aloamHeightFilter = std::make_shared<MedianFilter>(_aloam_height_filter_buffer_size_, _aloam_height_max_valid_altitude_, -_aloam_height_max_valid_altitude_,
                                                     _aloam_height_filter_max_difference_);

  ROS_INFO("[Odometry]: Garmin max valid altitude: %2.2f", _garmin_max_valid_altitude_);
  ROS_INFO("[Odometry]: Sonar max valid altitude: %2.2f", _sonar_max_valid_altitude_);
  ROS_INFO("[Odometry]: ALOAM max valid altitude: %2.2f", _aloam_height_max_valid_altitude_);

  // Load the measurements fused by each state estimator
  for (std::vector<std::string>::iterator it = _altitude_estimators_names_.begin(); it != _altitude_estimators_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam("altitude_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _alt_measurement_names_)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_alt_estimator_measurement.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));

    // Load the model of each estimator
    int temp_value;
    param_loader.loadParam("altitude_estimators/n_A/" + *it, temp_value);
    map_alt_n_states.insert(std::pair<std::string, int>(*it, temp_value));
    Eigen::MatrixXd A_model = Eigen::MatrixXd::Zero(_altitude_n_, _altitude_n_);
    param_loader.loadMatrixDynamic("altitude_estimators/A/" + *it, A_model, temp_value, temp_value);

    map_alt_model.insert(std::pair<std::string, Eigen::MatrixXd>(*it, A_model));
  }

  // Load the model state of each measurement
  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it != _alt_measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("altitude_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _alt_model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_alt_measurement_state.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  // Load the model state mapping
  for (std::vector<std::string>::iterator it = _alt_model_state_names_.begin(); it != _alt_model_state_names_.end(); ++it) {

    Eigen::MatrixXd temp_P = Eigen::MatrixXd::Zero(1, _altitude_n_);
    param_loader.loadMatrixStatic("altitude_estimators/state_mapping/" + *it, temp_P, 1, _altitude_n_);

    map_alt_states.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_P));
  }

  // Load the covariances of each measurement
  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it != _alt_measurement_names_.end(); ++it) {

    Eigen::MatrixXd temp_matrix;
    param_loader.loadMatrixStatic("altitude/Q/" + *it, temp_matrix, 1, 1);

    map_alt_measurement_covariance.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_matrix));
  }


  for (std::vector<std::string>::iterator it = _alt_measurement_names_.begin(); it < _alt_measurement_names_.end(); it++) {
    map_alt_measurement_name_id.insert(std::pair<std::string, int>(*it, (int)std::distance(_alt_measurement_names_.begin(), it)));
  }

  //}

  /* create altitude estimator //{ */


  // Loop through all estimators
  for (std::vector<std::string>::iterator it = _altitude_estimators_names_.begin(); it != _altitude_estimators_names_.end(); ++it) {

    std::vector<bool>            alt_fusing_measurement;
    std::vector<Eigen::MatrixXd> P_arr_alt, Q_arr_alt;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_alt_estimator_measurement.find(*it);

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
      std::map<std::string, std::string>::iterator pair_measurement_state = map_alt_measurement_state.find(*it2);

      // Find measurement to state mapping
      std::map<std::string, Eigen::MatrixXd>::iterator pair_state_matrix = map_alt_states.find(pair_measurement_state->second);
      P_arr_alt.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, Eigen::MatrixXd>::iterator pair_measurement_covariance = map_alt_measurement_covariance.find(*it2);
      Q_arr_alt.push_back(pair_measurement_covariance->second);
    }

    // Find model matrix A
    int                                              n_states             = map_alt_n_states.find(*it)->second;
    Eigen::MatrixXd                                  A_model              = Eigen::MatrixXd::Zero(n_states, n_states);
    std::map<std::string, Eigen::MatrixXd>::iterator pair_estimator_model = map_alt_model.find(*it);
    A_model                                                               = pair_estimator_model->second;

    // Add pointer to altitude estimator to array
    m_altitude_estimators.insert(std::pair<std::string, std::shared_ptr<AltitudeEstimator>>(
        *it, std::make_shared<AltitudeEstimator>(*it, alt_fusing_measurement, P_arr_alt, Q_arr_alt, A_model, _B_alt_, _R_alt_)));

    // Map odometry to estimator name
    mrs_msgs::Float64Stamped alt_msg;
    std::string              alt_estimator_name = *it;
    std::transform(alt_estimator_name.begin(), alt_estimator_name.end(), alt_estimator_name.begin(), ::tolower);
    map_alt_estimator_msg.insert(std::pair<std::string, mrs_msgs::Float64Stamped>(*it, alt_msg));

    // Map publisher to estimator name
    ros::Publisher pub = nh.advertise<mrs_msgs::Float64Stamped>(alt_estimator_name + "_out", 1);
    map_alt_estimator_pub.insert(std::pair<std::string, ros::Publisher>(*it, pub));

    ROS_INFO_STREAM("[Odometry]: Altitude estimator was initiated with following parameters: n: "
                    << _altitude_n_ << ", m: " << _altitude_m_ << ", p: " << _altitude_p_ << ", A: " << A_model << ", B: " << _B_alt_ << ", R: " << _R_alt_);
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

  /*  //{ load parameters of state estimators */

  param_loader.loadParam("lateral/numberOfVariables", _lateral_n_);
  param_loader.loadParam("lateral/numberOfInputs", _lateral_m_);
  param_loader.loadParam("lateral/numberOfMeasurements", _lateral_p_);

  param_loader.loadParam("state_estimators/state_estimators", _state_estimators_names_);
  param_loader.loadParam("state_estimators/active", _active_state_estimators_names_);
  param_loader.loadParam("state_estimators/model_states", _model_state_names_);
  param_loader.loadParam("state_estimators/measurements", _measurement_names_);
  param_loader.loadMatrixStatic("lateral/A", _A_lat_);
  param_loader.loadMatrixStatic("lateral/B", _B_lat_);
  param_loader.loadMatrixStatic("lateral/R", _R_lat_);

  param_loader.loadMatrixStatic("lateral/rtk/A", _A_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/B", _B_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/R", _R_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/Q", _Q_lat_rtk_);
  param_loader.loadMatrixStatic("lateral/rtk/P", _P_lat_rtk_);
  param_loader.loadParam("lateral/rtk_fuse_sps", _rtk_fuse_sps_);

  param_loader.loadParam("lateral/brick_timeout", _brick_timeout_);

  // Optflow median filter
  param_loader.loadParam("lateral/optflow_median_filter", _optflow_median_filter_);
  param_loader.loadParam("lateral/optflow_filter_buffer_size", _optflow_filter_buffer_size_);
  param_loader.loadParam("lateral/optflow_filter_max_valid", _optflow_filter_max_valid_);
  param_loader.loadParam("lateral/optflow_filter_max_diff", _optflow_filter_max_diff_);

  optflow_filter_x =
      std::make_shared<MedianFilter>(_optflow_filter_buffer_size_, _optflow_filter_max_valid_, -_optflow_filter_max_valid_, _optflow_filter_max_diff_);
  optflow_filter_y =
      std::make_shared<MedianFilter>(_optflow_filter_buffer_size_, _optflow_filter_max_valid_, -_optflow_filter_max_valid_, _optflow_filter_max_diff_);

  // LIDAR median filter
  param_loader.loadParam("lateral/lidar_vel_median_filter", _lidar_vel_median_filter_);
  param_loader.loadParam("lateral/lidar_vel_filter_buffer_size", _lidar_vel_filter_buffer_size_);
  param_loader.loadParam("lateral/lidar_vel_filter_max_valid", _lidar_vel_filter_max_valid_);
  param_loader.loadParam("lateral/lidar_vel_filter_max_diff", _lidar_vel_filter_max_diff_);

  lidar_vel_filter_x =
      std::make_shared<MedianFilter>(_lidar_vel_filter_buffer_size_, _lidar_vel_filter_max_valid_, -_lidar_vel_filter_max_valid_, _lidar_vel_filter_max_diff_);
  lidar_vel_filter_y =
      std::make_shared<MedianFilter>(_lidar_vel_filter_buffer_size_, _lidar_vel_filter_max_valid_, -_lidar_vel_filter_max_valid_, _lidar_vel_filter_max_diff_);

  // Hector median filter
  param_loader.loadParam("lateral/hector_pos_median_filter", _hector_pos_median_filter_);
  param_loader.loadParam("lateral/hector_pos_filter_buffer_size", _hector_pos_filter_buffer_size_);
  param_loader.loadParam("lateral/hector_pos_filter_max_valid", _hector_pos_filter_max_valid_);
  param_loader.loadParam("lateral/hector_pos_filter_max_diff", _hector_pos_filter_max_diff_);

  hector_pos_filter_x = std::make_shared<MedianFilter>(_hector_pos_filter_buffer_size_, _hector_pos_filter_max_valid_, -_hector_pos_filter_max_valid_,
                                                       _hector_pos_filter_max_diff_);
  hector_pos_filter_y = std::make_shared<MedianFilter>(_hector_pos_filter_buffer_size_, _hector_pos_filter_max_valid_, -_hector_pos_filter_max_valid_,
                                                       _hector_pos_filter_max_diff_);

  // TOWER median filter
  param_loader.loadParam("lateral/tower_pos_median_filter", _tower_pos_median_filter_);
  param_loader.loadParam("lateral/tower_pos_filter_buffer_size", _tower_pos_filter_buffer_size_);
  param_loader.loadParam("lateral/tower_pos_filter_max_valid", _tower_pos_filter_max_valid_);
  param_loader.loadParam("lateral/tower_pos_filter_max_diff", _tower_pos_filter_max_diff_);

  tower_pos_filter_x =
      std::make_shared<MedianFilter>(_tower_pos_filter_buffer_size_, _tower_pos_filter_max_valid_, -_tower_pos_filter_max_valid_, _tower_pos_filter_max_diff_);
  tower_pos_filter_y =
      std::make_shared<MedianFilter>(_tower_pos_filter_buffer_size_, _tower_pos_filter_max_valid_, -_tower_pos_filter_max_valid_, _tower_pos_filter_max_diff_);

  // ALOAM median filter
  param_loader.loadParam("lateral/aloam_pos_median_filter", _aloam_pos_median_filter_);
  param_loader.loadParam("lateral/aloam_pos_filter_buffer_size", _aloam_pos_filter_buffer_size_);
  param_loader.loadParam("lateral/aloam_pos_filter_max_valid", _aloam_pos_filter_max_valid_);
  param_loader.loadParam("lateral/aloam_pos_filter_max_diff", _aloam_pos_filter_max_diff_);

  aloam_pos_filter_x =
      std::make_shared<MedianFilter>(_aloam_pos_filter_buffer_size_, _aloam_pos_filter_max_valid_, -_aloam_pos_filter_max_valid_, _aloam_pos_filter_max_diff_);
  aloam_pos_filter_y =
      std::make_shared<MedianFilter>(_aloam_pos_filter_buffer_size_, _aloam_pos_filter_max_valid_, -_aloam_pos_filter_max_valid_, _aloam_pos_filter_max_diff_);

  // ICP median filter
  param_loader.loadParam("lateral/icp_twist_median_filter", _icp_twist_median_filter_);
  param_loader.loadParam("lateral/icp_twist_filter_buffer_size", _icp_twist_filter_buffer_size_);
  param_loader.loadParam("lateral/icp_twist_filter_max_valid", _icp_twist_filter_max_valid_);
  param_loader.loadParam("lateral/icp_twist_filter_max_diff", _icp_twist_filter_max_diff_);

  icp_twist_filter_x =
      std::make_shared<MedianFilter>(_icp_twist_filter_buffer_size_, _icp_twist_filter_max_valid_, -_icp_twist_filter_max_valid_, _icp_twist_filter_max_diff_);
  icp_twist_filter_y =

      std::make_shared<MedianFilter>(_icp_twist_filter_buffer_size_, _icp_twist_filter_max_valid_, -_icp_twist_filter_max_valid_, _icp_twist_filter_max_diff_);

  // brick median filter
  param_loader.loadParam("lateral/brick_pos_median_filter", _brick_pos_median_filter_);
  param_loader.loadParam("lateral/brick_pos_filter_buffer_size", _brick_pos_filter_buffer_size_);
  param_loader.loadParam("lateral/brick_pos_filter_max_valid", _brick_pos_filter_max_valid_);
  param_loader.loadParam("lateral/brick_pos_filter_max_diff", _brick_pos_filter_max_diff_);

  brick_pos_filter_x =
      std::make_shared<MedianFilter>(_brick_pos_filter_buffer_size_, _brick_pos_filter_max_valid_, -_brick_pos_filter_max_valid_, _brick_pos_filter_max_diff_);
  brick_pos_filter_y =
      std::make_shared<MedianFilter>(_brick_pos_filter_buffer_size_, _brick_pos_filter_max_valid_, -_brick_pos_filter_max_valid_, _brick_pos_filter_max_diff_);

  // Check whether GPS is active
  if (!stringInVector("GPS", _active_state_estimators_names_)) {
    gps_active_ = false;
    ROS_INFO("[Odometry]: GPS is not in the list of active estimators.");
  }

  // Load the measurements fused by each state estimator
  for (std::vector<std::string>::iterator it = _active_state_estimators_names_.begin(); it != _active_state_estimators_names_.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.loadParam("state_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _measurement_names_)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_estimator_measurement.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // Load the model state of each measurement
  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it != _measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("state_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_measurement_state.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  // Load the model state mapping
  for (std::vector<std::string>::iterator it = _model_state_names_.begin(); it != _model_state_names_.end(); ++it) {

    LatStateCol1D temp_P;
    param_loader.loadMatrixStatic("state_estimators/state_mapping/" + *it, temp_P);

    map_states.insert(std::pair<std::string, LatStateCol1D>(*it, temp_P));
  }

  // Load the covariances of each measurement
  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it != _measurement_names_.end(); ++it) {

    Mat1 temp_matrix;
    param_loader.loadMatrixStatic("lateral/Q/" + *it, temp_matrix);

    map_measurement_covariance.insert(std::pair<std::string, Mat1>(*it, temp_matrix));
  }

  for (std::vector<std::string>::iterator it = _measurement_names_.begin(); it < _measurement_names_.end(); it++) {
    map_measurement_name_id.insert(std::pair<std::string, int>(*it, (int)std::distance(_measurement_names_.begin(), it)));
  }

  //}

  /*  //{ create state estimators*/
  for (std::vector<std::string>::iterator it = _active_state_estimators_names_.begin(); it != _active_state_estimators_names_.end(); ++it) {

    std::vector<bool>          fusing_measurement;
    std::vector<LatStateCol1D> P_arr_lat, Q_arr_lat;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_estimator_measurement.find(*it);

    // Loop through all measurements
    for (std::vector<std::string>::iterator it2 = _measurement_names_.begin(); it2 != _measurement_names_.end(); ++it2) {

      // Check whether measurement is fused by the estimator
      if (stringInVector(*it2, temp_vec->second)) {
        fusing_measurement.push_back(true);
      } else {
        fusing_measurement.push_back(false);
      }

      // Find state name
      std::map<std::string, std::string>::iterator pair_measurement_state = map_measurement_state.find(*it2);

      // Find state mapping
      std::map<std::string, LatStateCol1D>::iterator pair_state_matrix = map_states.find(pair_measurement_state->second);
      P_arr_lat.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, Mat1>::iterator pair_measurement_covariance = map_measurement_covariance.find(*it2);
      if (*it2 == "vel_optflow") {
        Q_arr_lat.push_back(LatStateCol1D::Ones() * pair_measurement_covariance->second(0) * 1000);
      } else {
        Q_arr_lat.push_back(LatStateCol1D::Ones() * pair_measurement_covariance->second(0));
      }
    }

    // Add state estimator to array
    m_state_estimators.insert(std::pair<std::string, std::shared_ptr<StateEstimator>>(
        *it, std::make_shared<StateEstimator>(*it, fusing_measurement, _A_lat_, _B_lat_, _R_lat_, P_arr_lat, Q_arr_lat)));

    estimator_rtk_ = std::make_shared<lkf_rtk_t>(_A_lat_rtk_, _B_lat_rtk_, _H_lat_rtk_);

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
    map_estimator_odom.insert(std::pair<std::string, nav_msgs::Odometry>(*it, odom));

    // Map publisher to estimator name
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom_" + estimator_name + "_out", 1);
    map_estimator_pub.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }
  is_lateral_estimator_initialized = true;
  //}

  /* load parameters of heading estimator //{ */

  param_loader.loadParam("heading/numberOfVariables", _heading_n_);
  param_loader.loadParam("heading/numberOfInputs", _heading_m_);
  param_loader.loadParam("heading/numberOfMeasurements", _heading_p_);

  param_loader.loadMatrixDynamic("heading/A", A_hdg, _heading_n_, _heading_n_);
  param_loader.loadMatrixDynamic("heading/B", B_hdg, _heading_n_, _heading_m_);
  param_loader.loadMatrixDynamic("heading/R", R_hdg, _heading_n_, _heading_n_);

  param_loader.loadParam("heading_estimators/model_states", _hdg_model_state_names_);
  param_loader.loadParam("heading_estimators/measurements", _hdg_measurement_names_);
  param_loader.loadParam("heading_estimators/heading_estimators", _heading_estimators_names_);
  param_loader.loadParam("heading_estimators/active", _active_heading_estimators_names_);

  param_loader.loadParam("heading/optflow_yaw_rate_filter_buffer_size", _optflow_yaw_rate_filter_buffer_size_);
  param_loader.loadParam("heading/optflow_yaw_rate_filter_max_valid", _optflow_yaw_rate_filter_max_valid_);
  param_loader.loadParam("heading/optflow_yaw_rate_filter_max_diff", _optflow_yaw_rate_filter_max_diff_);

  param_loader.loadParam("heading/icp_yaw_rate_filter_buffer_size", _icp_yaw_rate_filter_buffer_size_);
  param_loader.loadParam("heading/icp_yaw_rate_filter_max_valid", _icp_yaw_rate_filter_max_valid_);
  param_loader.loadParam("heading/icp_yaw_rate_filter_max_diff", _icp_yaw_rate_filter_max_diff_);

  param_loader.loadParam("heading/compass_yaw_filter_buffer_size", _compass_yaw_filter_buffer_size_);
  param_loader.loadParam("heading/compass_yaw_filter_max_diff", _compass_yaw_filter_max_diff_);

  param_loader.loadParam("heading/hector_yaw_filter_buffer_size", _hector_yaw_filter_buffer_size_);
  param_loader.loadParam("heading/hector_yaw_filter_max_diff", _hector_yaw_filter_max_diff_);

  param_loader.loadParam("heading/tower_yaw_filter_buffer_size", _tower_yaw_filter_buffer_size_);
  param_loader.loadParam("heading/tower_yaw_filter_max_diff", _tower_yaw_filter_max_diff_);

  param_loader.loadParam("heading/aloam_yaw_filter_buffer_size", _aloam_yaw_filter_buffer_size_);
  param_loader.loadParam("heading/aloam_yaw_filter_max_diff", _aloam_yaw_filter_max_diff_);

  param_loader.loadParam("heading/brick_yaw_filter_buffer_size", _brick_yaw_filter_buffer_size_);
  param_loader.loadParam("heading/brick_yaw_filter_max_diff", _brick_yaw_filter_max_diff_);
  param_loader.loadParam("heading/max_brick_yaw_correction", _max_brick_yaw_correction_);
  param_loader.loadParam("heading/accum_yaw_brick_alpha", _accum_yaw_brick_alpha_);
  accum_yaw_brick_ = 0.0;

  param_loader.loadParam("heading_estimator", _heading_estimator_name_);
  param_loader.loadParam("heading/gyro_fallback", _gyro_fallback_);

  optflow_yaw_rate_filter = std::make_shared<MedianFilter>(_optflow_yaw_rate_filter_buffer_size_, _optflow_yaw_rate_filter_max_valid_,
                                                           -_optflow_yaw_rate_filter_max_valid_, _optflow_yaw_rate_filter_max_diff_);
  icp_yaw_rate_filter     = std::make_shared<MedianFilter>(_icp_yaw_rate_filter_buffer_size_, _icp_yaw_rate_filter_max_valid_, -_icp_yaw_rate_filter_max_valid_,
                                                       _icp_yaw_rate_filter_max_diff_);
  hector_yaw_filter       = std::make_shared<MedianFilter>(_hector_yaw_filter_buffer_size_, 1000000, -1000000, _hector_yaw_filter_max_diff_);
  tower_yaw_filter        = std::make_shared<MedianFilter>(_tower_yaw_filter_buffer_size_, 1000000, -1000000, _tower_yaw_filter_max_diff_);
  brick_yaw_filter        = std::make_shared<MedianFilter>(_brick_yaw_filter_buffer_size_, 1000000, -1000000, _brick_yaw_filter_max_diff_);
  aloam_yaw_filter        = std::make_shared<MedianFilter>(_aloam_yaw_filter_buffer_size_, 1000000, -1000000, _aloam_yaw_filter_max_diff_);

  compass_yaw_filter                = std::make_shared<MedianFilter>(_compass_yaw_filter_buffer_size_, 1000000, -1000000, _compass_yaw_filter_max_diff_);
  compass_inconsistent_samples      = 0;
  optflow_inconsistent_samples      = 0;
  icp_yaw_rate_inconsistent_samples = 0;

  size_t pos_hdg = std::distance(_heading_type_names.begin(), std::find(_heading_type_names.begin(), _heading_type_names.end(), _heading_estimator_name_));

  _hdg_estimator_type_takeoff.name = _heading_estimator_name_;
  _hdg_estimator_type_takeoff.type = (int)pos_hdg;

  // Check whether PIXHAWK is active
  if (!stringInVector("PIXHAWK", _active_heading_estimators_names_)) {
    gps_active_ = false;
    ROS_INFO("[Odometry]: PIXHAWK is not in the list of active heading estimators.");
  }

  // Load the measurements fused by each heading estimator
  for (std::vector<std::string>::iterator it = _active_heading_estimators_names_.begin(); it != _active_heading_estimators_names_.end(); ++it) {

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

    map_hdg_estimator_measurement.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // Load the model state of each measurement
  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it != _hdg_measurement_names_.end(); ++it) {

    std::string temp_value;
    param_loader.loadParam("heading_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _hdg_model_state_names_)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_hdg_measurement_state.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  // Load the model state mapping
  for (std::vector<std::string>::iterator it = _hdg_model_state_names_.begin(); it != _hdg_model_state_names_.end(); ++it) {

    Eigen::MatrixXd temp_P = Eigen::MatrixXd::Zero(1, _heading_n_);
    param_loader.loadMatrixStatic("heading_estimators/state_mapping/" + *it, temp_P, 1, _heading_n_);

    map_hdg_states.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_P));
  }

  // Load the covariances of each measurement
  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it != _hdg_measurement_names_.end(); ++it) {

    Eigen::MatrixXd temp_matrix;
    param_loader.loadMatrixStatic("heading/Q/" + *it, temp_matrix, 1, 1);

    map_hdg_measurement_covariance.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_matrix));
  }

  for (std::vector<std::string>::iterator it = _hdg_measurement_names_.begin(); it < _hdg_measurement_names_.end(); it++) {
    map_hdg_measurement_name_id.insert(std::pair<std::string, int>(*it, (int)std::distance(_hdg_measurement_names_.begin(), it)));
  }
  for (auto &it : map_hdg_measurement_name_id) {
    ROS_INFO("[Odometry]: heading measurement mapping: %s - %d", it.first.c_str(), it.second);
  }

  //}

  /* create heading estimator //{ */

  // Loop through all estimators
  for (std::vector<std::string>::iterator it = _active_heading_estimators_names_.begin(); it != _active_heading_estimators_names_.end(); ++it) {

    std::vector<bool>            hdg_fusing_measurement;
    std::vector<Eigen::MatrixXd> P_arr_hdg, Q_arr_hdg;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_hdg_estimator_measurement.find(*it);

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
      std::map<std::string, std::string>::iterator pair_measurement_state = map_hdg_measurement_state.find(*it2);

      // Find measurement to state mapping
      std::map<std::string, Eigen::MatrixXd>::iterator pair_state_matrix = map_hdg_states.find(pair_measurement_state->second);
      P_arr_hdg.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, Eigen::MatrixXd>::iterator pair_measurement_covariance = map_hdg_measurement_covariance.find(*it2);
      Q_arr_hdg.push_back(pair_measurement_covariance->second);
    }

    // Add pointer to heading estimator to array
    // this is how to create shared pointers!!! the correct way
    m_heading_estimators.insert(std::pair<std::string, std::shared_ptr<HeadingEstimator>>(
        *it, std::make_shared<HeadingEstimator>(*it, hdg_fusing_measurement, P_arr_hdg, Q_arr_hdg, A_hdg, B_hdg, R_hdg)));

    // Map odometry to estimator name
    mrs_msgs::Float64ArrayStamped hdg_msg;
    std::string                   hdg_estimator_name = *it;
    std::transform(hdg_estimator_name.begin(), hdg_estimator_name.end(), hdg_estimator_name.begin(), ::tolower);
    map_hdg_estimator_msg.insert(std::pair<std::string, mrs_msgs::Float64ArrayStamped>(*it, hdg_msg));

    // Map publisher to heading estimator name
    ros::Publisher pub = nh.advertise<mrs_msgs::Float64ArrayStamped>("hdg_" + hdg_estimator_name + "_out", 1);
    map_hdg_estimator_pub.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }

  ROS_INFO_STREAM("[Odometry]: heading estimator was initiated with following parameters: n: "
                  << _heading_n_ << ", m: " << _heading_m_ << ", p: " << _heading_p_ << ", A: " << A_hdg << ", B: " << B_hdg << ", R: " << R_hdg);

  ROS_INFO("[Odometry]: heading estimator prepared");

  //}

  param_loader.loadParam("publish_fused_odom", _publish_fused_odom_);
  param_loader.loadParam("publish_local_origin_stable_tf", _publish_local_origin_stable_tf_);
  param_loader.loadParam("publish_pixhawk_velocity", _publish_pixhawk_velocity_);
  param_loader.loadParam("pass_rtk_as_odom", _pass_rtk_as_odom_);
  param_loader.loadParam("max_altitude_correction", _max_altitude_correction_);
  param_loader.loadParam("reset_hector_after_takeoff", _reset_hector_after_takeoff_);
  param_loader.loadParam("perform_hector_reset_routine", _perform_hector_reset_routine_);

  param_loader.loadParam("lateral/saturate_mavros_position", _saturate_mavros_position_);
  param_loader.loadParam("lateral/max_mavros_pos_correction", _max_mavros_pos_correction_);
  param_loader.loadParam("lateral/max_vio_pos_correction", _max_vio_pos_correction_);
  param_loader.loadParam("lateral/max_vslam_pos_correction", _max_vslam_pos_correction_);
  param_loader.loadParam("lateral/max_brick_pos_correction", _max_brick_pos_correction_);
  param_loader.loadParam("lateral/max_rtk_pos_correction", _max_rtk_pos_correction_);
  param_loader.loadParam("lateral/max_t265_vel", _max_t265_vel_);
  double max_safe_brick_jump_tmp = 0.0;
  param_loader.loadParam("lateral/max_safe_brick_jump", max_safe_brick_jump_tmp);
  max_safe_brick_jump_sq_            = std::pow(max_safe_brick_jump_tmp, 2);
  double max_safe_brick_yaw_jump_tmp = 0.0;
  param_loader.loadParam("lateral/max_safe_brick_yaw_jump", max_safe_brick_yaw_jump_tmp);
  max_safe_brick_yaw_jump_sq_ = std::pow(max_safe_brick_yaw_jump_tmp, 2);

  param_loader.loadParam("lateral/gps_fallback/allowed", _gps_fallback_allowed_);
  param_loader.loadParam("lateral/gps_fallback/fallback_estimator", _gps_fallback_estimator_);
  param_loader.loadParam("lateral/gps_fallback/cov_limit", _gps_fallback_covariance_limit_);
  param_loader.loadParam("lateral/gps_fallback/cov_ok", _gps_fallback_covariance_ok_);
  param_loader.loadParam("lateral/gps_fallback/return_after_ok", _gps_return_after_fallback_);
  param_loader.loadParam("lateral/gps_fallback/bad_samples", _gps_fallback_bad_samples_);
  param_loader.loadParam("lateral/gps_fallback/good_samples", _gps_fallback_good_samples_);
  param_loader.loadParam("lateral/gps_fallback/altitude", _gps_fallback_altitude_);
  param_loader.loadParam("lateral/gps_fallback/altitude_wait_time", _gps_fallback_wait_for_altitude_time_);


  if (_pass_rtk_as_odom_ && !_rtk_available_) {
    ROS_ERROR("[Odometry]: cannot have _pass_rtk_as_odom_ TRUE when rtk_available FALSE");
    ros::shutdown();
  }

  odom_pixhawk_last_update = ros::Time::now();

  garmin_enabled       = true;
  sonar_enabled        = true;
  rtk_altitude_enabled = false;

  // --------------------------------------------------------------
  // |                         tf listener                        |
  // --------------------------------------------------------------
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, "mrs_uav_odometry");
  transformer_      = mrs_lib::Transformer("Odometry", _uav_name_);


  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh, "Odometry", _profiler_enabled_);

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  /* //{ publishers */
  pub_uav_state_           = nh.advertise<mrs_msgs::UavState>("uav_state_out", 1);
  pub_odom_main_           = nh.advertise<nav_msgs::Odometry>("odom_main_out", 1);
  pub_odom_main_inno_      = nh.advertise<nav_msgs::Odometry>("odom_main_inno_out", 1);
  pub_odom_local_          = nh.advertise<nav_msgs::Odometry>("odom_local_out", 1);
  pub_odom_stable_         = nh.advertise<nav_msgs::Odometry>("odom_stable_out", 1);
  pub_slow_odom_           = nh.advertise<nav_msgs::Odometry>("slow_odom_out", 1);
  pub_odom_mavros_         = nh.advertise<nav_msgs::Odometry>("odom_mavros_out", 1);
  pub_esp_odom_            = nh.advertise<mrs_msgs::EspOdometry>("esp_odom_out", 1);
  pub_odometry_diag_       = nh.advertise<mrs_msgs::OdometryDiag>("odometry_diag_out", 1);
  pub_altitude_            = nh.advertise<mrs_msgs::Float64Stamped>("altitude_out", 1);
  pub_height_              = nh.advertise<mrs_msgs::Float64Stamped>("height_out", 1);
  pub_max_altitude_        = nh.advertise<mrs_msgs::Float64Stamped>("max_altitude_out", 1);
  pub_orientation_         = nh.advertise<nav_msgs::Odometry>("orientation_out", 1);
  pub_lkf_states_x_        = nh.advertise<mrs_msgs::LkfStates>("lkf_states_x_out", 1);
  pub_lkf_states_y_        = nh.advertise<mrs_msgs::LkfStates>("lkf_states_y_out", 1);
  pub_heading_states_      = nh.advertise<mrs_msgs::EstimatedState>("heading_state_out", 1);
  pub_altitude_state_      = nh.advertise<mrs_msgs::EstimatedState>("altitude_state_out", 1);
  pub_alt_cov_             = nh.advertise<mrs_msgs::Float64ArrayStamped>("altitude_covariance_out", 1);
  pub_debug_optflow_filter = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("optflow_filtered_out", 1);
  pub_imu_untilted_        = nh.advertise<sensor_msgs::Imu>("imu_untilted_out", 1);
  pub_brick_diag_          = nh.advertise<mrs_msgs::ReferenceStamped>("brick_diag_out", 1);

  // republisher for rtk local
  pub_rtk_local_ = nh.advertise<mrs_msgs::RtkGps>("rtk_local_out", 1);

  // republisher for rtk local odometry (e.g. for rviz)
  pub_rtk_local_odom_ = nh.advertise<nav_msgs::Odometry>("rtk_local_odom_out", 1);

  // republisher for gps local odometry (e.g. for rviz)
  pub_gps_local_odom_ = nh.advertise<nav_msgs::Odometry>("gps_local_odom_out", 1);

  // publisher for resetting hector map
  pub_hector_reset_ = nh.advertise<std_msgs::String>("hector_map_reset_out", 1);

  // publisher for tf
  broadcaster_ = new tf2_ros::TransformBroadcaster();

  pub_compass_yaw_ = nh.advertise<mrs_msgs::Float64Stamped>("compass_yaw_out", 1);
  pub_hector_yaw_  = nh.advertise<mrs_msgs::Float64Stamped>("hector_yaw_out", 1);
  pub_tower_yaw_   = nh.advertise<mrs_msgs::Float64Stamped>("tower_yaw_out", 1);
  pub_aloam_yaw_   = nh.advertise<mrs_msgs::Float64Stamped>("aloam_yaw_out", 1);
  pub_vio_yaw_     = nh.advertise<mrs_msgs::Float64Stamped>("vio_yaw_out", 1);
  pub_vslam_yaw_   = nh.advertise<mrs_msgs::Float64Stamped>("vslam_yaw_out", 1);
  pub_brick_yaw_   = nh.advertise<mrs_msgs::Float64Stamped>("brick_yaw_out", 1);


  // publishers for roll pitch yaw orientations in local_origin frame
  pub_des_attitude_global_ = nh.advertise<geometry_msgs::Vector3Stamped>("des_attitude_global_out", 1);
  pub_orientation_gt_      = nh.advertise<geometry_msgs::Vector3Stamped>("orientation_gt_out", 1);
  pub_orientation_mavros_  = nh.advertise<geometry_msgs::Vector3Stamped>("orientation_mavros_out", 1);
  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  /* //{ subscribers */
  // subsribe to target attitude
  sub_attitude_command_ = nh.subscribe("attitude_command_in", 1, &Odometry::callbackAttitudeCommand, this, ros::TransportHints().tcpNoDelay());

  // subscribe to pixhawk imu
  sub_pixhawk_imu_ = nh.subscribe("pixhawk_imu_in", 1, &Odometry::callbackPixhawkImu, this, ros::TransportHints().tcpNoDelay());

  // subscribe to compass heading
  sub_pixhawk_compass_ = nh.subscribe("pixhawk_compass_in", 1, &Odometry::callbackPixhawkCompassHdg, this, ros::TransportHints().tcpNoDelay());

  // subscriber to mavros odometry
  sub_pixhawk_ = nh.subscribe("pixhawk_odom_in", 1, &Odometry::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());

  // subscriber to t265 odometry
  if (_t265_available_) {
    sub_t265_odom_ = nh.subscribe("t265_odom_in", 1, &Odometry::callbackT265Odometry, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to optflow velocity
  if (_optflow_available_) {
    sub_optflow_ = nh.subscribe("optflow_in", 1, &Odometry::callbackOptflowTwist, this, ros::TransportHints().tcpNoDelay());
    if (_use_optflow_low_) {
      sub_optflow_low_ = nh.subscribe("optflow_low_in", 1, &Odometry::callbackOptflowTwistLow, this, ros::TransportHints().tcpNoDelay());
    }
  }

  // subscriber to visual odometry
  if (_vio_available_) {
    sub_vio_ = nh.subscribe("vio_in", 1, &Odometry::callbackVioOdometry, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to visual slam (pose)
  if (_vslam_available_) {
    sub_vslam_ = nh.subscribe("vslam_in", 1, &Odometry::callbackVslamPose, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to brick odometry
  if (_brick_available_) {
    sub_brick_pose_ = nh.subscribe("brick_pose_in", 1, &Odometry::callbackBrickPose, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for differential gps
  if (_rtk_available_) {
    rtk_gps_sub_ = nh.subscribe("rtk_gps_in", 1, &Odometry::callbackRtkGps, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for lidar odometry
  if (_lidar_available_) {
    sub_lidar_odom_  = nh.subscribe("lidar_odom_in", 1, &Odometry::callbackLidarOdom, this, ros::TransportHints().tcpNoDelay());
    sub_hector_pose_ = nh.subscribe("hector_pose_in", 1, &Odometry::callbackHectorPose, this, ros::TransportHints().tcpNoDelay());
    sub_tower_pose_  = nh.subscribe("tower_pose_in", 1, &Odometry::callbackTowerPose, this, ros::TransportHints().tcpNoDelay());
    sub_icp_twist_   = nh.subscribe("icp_twist_in", 1, &Odometry::callbackICPTwist, this, ros::TransportHints().tcpNoDelay());
  }

  if (_aloam_available_) {
    sub_aloam_odom_ = nh.subscribe("aloam_odom_in", 1, &Odometry::callbackAloamOdom, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for garmin range
  sub_garmin_ = nh.subscribe("garmin_in", 1, &Odometry::callbackGarmin, this, ros::TransportHints().tcpNoDelay());

  // subscriber for sonar range
  sub_sonar_ = nh.subscribe("sonar_in", 1, &Odometry::callbackSonar, this, ros::TransportHints().tcpNoDelay());

  // subscriber for plane range
  sub_plane_ = nh.subscribe("plane_in", 1, &Odometry::callbackPlane, this, ros::TransportHints().tcpNoDelay());

  // subscriber for ground truth
  sub_ground_truth_ = nh.subscribe("ground_truth_in", 1, &Odometry::callbackGroundTruth, this, ros::TransportHints().tcpNoDelay());

  // subscribe for utm coordinates
  sub_global_position_ = nh.subscribe("global_position_in", 1, &Odometry::callbackPixhawkUtm, this, ros::TransportHints().tcpNoDelay());

  // subscribe for control manager diagnostics
  sub_control_manager_diag_ = nh.subscribe("control_manager_diag_in", 1, &Odometry::callbackControlManagerDiag, this, ros::TransportHints().tcpNoDelay());

  // subscribe for uav mass estimate
  sub_uav_mass_estimate_ = nh.subscribe("uav_mass_estimate_in", 1, &Odometry::callbackUavMassEstimate, this, ros::TransportHints().tcpNoDelay());

  // subscribe for gps covariance
  if (_gps_fallback_allowed_) {
    sub_gps_covariance_ = nh.subscribe("gps_covariance_in", 1, &Odometry::callbackGPSCovariance, this, ros::TransportHints().tcpNoDelay());
  }

  //}

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  /* //{ services */

  // subscribe for reset kalman service
  ser_reset_lateral_kalman_ = nh.advertiseService("reset_lateral_kalman_in", &Odometry::callbackResetEstimator, this);

  // subscribe for reset hector service
  ser_reset_hector_ = nh.advertiseService("reset_hector_in", &Odometry::callbackResetHector, this);

  // subscribe for reliable hector service
  ser_reliable_hector_ = nh.advertiseService("reliable_hector_in", &Odometry::callbackReliableHector, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh.advertiseService("toggle_garmin_in", &Odometry::callbackToggleGarmin, this);

  // change odometry source
  ser_change_odometry_source = nh.advertiseService("change_odometry_source_in", &Odometry::callbackChangeOdometrySource, this);

  // change current estimator
  ser_change_estimator_type = nh.advertiseService("change_estimator_type_in", &Odometry::callbackChangeEstimator, this);

  // change current estimator
  ser_change_estimator_type_string = nh.advertiseService("change_estimator_type_string_in", &Odometry::callbackChangeEstimatorString, this);

  ser_change_hdg_estimator_type = nh.advertiseService("change_hdg_estimator_type_in", &Odometry::callbackChangeHdgEstimator, this);

  ser_change_hdg_estimator_type_string = nh.advertiseService("change_hdg_estimator_type_string_in", &Odometry::callbackChangeHdgEstimatorString, this);

  ser_change_alt_estimator_type = nh.advertiseService("change_alt_estimator_type_in", &Odometry::callbackChangeAltEstimator, this);

  ser_change_alt_estimator_type_string = nh.advertiseService("change_alt_estimator_type_string_in", &Odometry::callbackChangeAltEstimatorString, this);

  ser_gyro_jump_ = nh.advertiseService("gyro_jump_in", &Odometry::callbackGyroJump, this);

  // subscribe for callbacks toggle service
  ser_toggle_callbacks_ = nh.advertiseService("toggle_callbacks_in", &Odometry::callbackToggleCallbacks, this);

  // initialized service clients
  ser_client_failsafe_         = nh.serviceClient<std_srvs::Trigger>("failsafe_out");
  ser_client_hover_            = nh.serviceClient<std_srvs::Trigger>("hover_out");
  ser_client_reference_        = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ereference_out");
  ser_client_ehover_           = nh.serviceClient<std_srvs::Trigger>("ehover_out");
  ser_client_enable_callbacks_ = nh.serviceClient<std_srvs::SetBool>("enable_callbacks_out");
  ser_client_tracker_          = nh.serviceClient<mrs_msgs::String>("tracker_out");
  ser_client_controller_       = nh.serviceClient<mrs_msgs::String>("controller_out");
  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  /* timers //{ */

  main_timer                 = nh.createTimer(ros::Rate(_main_rate_), &Odometry::mainTimer, this);
  slow_odom_timer            = nh.createTimer(ros::Rate(_slow_odom_rate_), &Odometry::slowOdomTimer, this);
  diag_timer                 = nh.createTimer(ros::Rate(_diag_rate_), &Odometry::diagTimer, this);
  lkf_states_timer           = nh.createTimer(ros::Rate(_lkf_states_rate_), &Odometry::lkfStatesTimer, this);
  max_altitude_timer         = nh.createTimer(ros::Rate(_max_altitude_rate_), &Odometry::maxAltitudeTimer, this);
  topic_watcher_timer        = nh.createTimer(ros::Rate(topic_watcher_rate_), &Odometry::topicWatcherTimer, this);
  hector_reset_routine_timer = nh.createTimer(ros::Duration(0.00001), &Odometry::callbackTimerHectorResetRoutine, this, true, false);

  //}

  /* check validity and set takeoff estimator //{ */

  // If required sensor is not available shutdown
  ROS_INFO_ONCE("[Odometry]: Requested %s type for takeoff.", _estimator_type_takeoff.name.c_str());
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOW && !_optflow_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Optflow localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::GPS && !_gps_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. GPS localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOWGPS && !_optflow_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Optflow localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::RTK && !_rtk_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. RTK localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::T265 && !_t265_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. T265 localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::LIDAR && !_lidar_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Lidar localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::HECTOR && !_lidar_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Lidar localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::TOWER && !_lidar_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Lidar localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::ALOAM && !_aloam_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ALOAM localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::ICP && !_lidar_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. ICP localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::VIO && !_vio_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Visual odometry localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::VSLAM && !_vslam_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Visual SLAM localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::BRICKFLOW && !_optflow_available_) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Optflow localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::BRICK) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Takeoff in this odometry mode is not supported. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  bool success;

  success = changeCurrentHeadingEstimator(_hdg_estimator_type_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff heading estimator type %s could not be set. Shutting down.", _hdg_estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  success = changeCurrentAltitudeEstimator(_alt_estimator_type_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff altitude estimator type %s could not be set. Shutting down.", _alt_estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  success = changeCurrentEstimator(_estimator_type_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Shutting down.", _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  //}

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  /* pass current covariances to dynamic reconfigure //{ */
  {
    std::scoped_lock lock(mutex_current_estimator);

    // Lateral position measurement covariances
    current_estimator->getR(last_drs_config.R_pos_mavros, map_measurement_name_id.find("pos_mavros")->second);
    current_estimator->getR(last_drs_config.R_pos_vio, map_measurement_name_id.find("pos_vio")->second);
    current_estimator->getR(last_drs_config.R_pos_vslam, map_measurement_name_id.find("pos_vslam")->second);
    current_estimator->getR(last_drs_config.R_pos_brick, map_measurement_name_id.find("pos_brick")->second);
    current_estimator->getR(last_drs_config.R_pos_lidar, map_measurement_name_id.find("pos_lidar")->second);
    current_estimator->getR(last_drs_config.R_pos_rtk, map_measurement_name_id.find("pos_rtk")->second);
    current_estimator->getR(last_drs_config.R_pos_hector, map_measurement_name_id.find("pos_hector")->second);
    current_estimator->getR(last_drs_config.R_pos_tower, map_measurement_name_id.find("pos_tower")->second);
    current_estimator->getR(last_drs_config.R_pos_aloam, map_measurement_name_id.find("pos_aloam")->second);

    // Lateral velocity measurement covariances
    current_estimator->getR(last_drs_config.R_vel_mavros, map_measurement_name_id.find("vel_mavros")->second);
    current_estimator->getR(last_drs_config.R_vel_vio, map_measurement_name_id.find("vel_vio")->second);
    current_estimator->getR(last_drs_config.R_vel_icp, map_measurement_name_id.find("vel_icp")->second);
    current_estimator->getR(last_drs_config.R_vel_lidar, map_measurement_name_id.find("vel_lidar")->second);
    current_estimator->getR(last_drs_config.R_vel_optflow, map_measurement_name_id.find("vel_optflow")->second);
    current_estimator->getR(last_drs_config.R_vel_rtk, map_measurement_name_id.find("vel_rtk")->second);

    // Lateral imu accelerations measurement covariances
    current_estimator->getR(last_drs_config.R_acc_imu_lat, map_measurement_name_id.find("acc_imu")->second);

    // Lateral process covariances
    current_estimator->getQ(last_drs_config.Q_pos, Eigen::Vector2i(0, 0));
    current_estimator->getQ(last_drs_config.Q_vel, Eigen::Vector2i(1, 1));
    current_estimator->getQ(last_drs_config.Q_acc, Eigen::Vector2i(2, 2));
  }

  {
    std::scoped_lock lock(mutex_current_alt_estimator);

    // Altitude measurement covariances
    current_alt_estimator->getR(last_drs_config.R_height_range, map_alt_measurement_name_id.find("height_range")->second);
    current_alt_estimator->getR(last_drs_config.R_height_plane, map_alt_measurement_name_id.find("height_plane")->second);
    current_alt_estimator->getR(last_drs_config.R_height_brick, map_alt_measurement_name_id.find("height_brick")->second);
    current_alt_estimator->getR(last_drs_config.R_height_vio, map_alt_measurement_name_id.find("height_vio")->second);
    current_alt_estimator->getR(last_drs_config.R_height_aloam, map_alt_measurement_name_id.find("height_aloam")->second);
    current_alt_estimator->getR(last_drs_config.R_height_baro, map_alt_measurement_name_id.find("height_baro")->second);

    // Altitude velocity measurement covariances
    current_alt_estimator->getR(last_drs_config.R_vel_baro, map_alt_measurement_name_id.find("vel_baro")->second);

    // Altitude acceleration measurement covariances
    current_alt_estimator->getR(last_drs_config.R_acc_imu, map_alt_measurement_name_id.find("acc_imu")->second);
  }

  {
    std::scoped_lock lock(mutex_current_hdg_estimator);

    // Heading measurement covariances
    current_hdg_estimator->getR(last_drs_config.R_yaw_compass, map_hdg_measurement_name_id.find("yaw_compass")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_hector, map_hdg_measurement_name_id.find("yaw_hector")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_tower, map_hdg_measurement_name_id.find("yaw_tower")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_aloam, map_hdg_measurement_name_id.find("yaw_aloam")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_brick, map_hdg_measurement_name_id.find("yaw_brick")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_vio, map_hdg_measurement_name_id.find("yaw_vio")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_vslam, map_hdg_measurement_name_id.find("yaw_vslam")->second);
    current_hdg_estimator->getR(last_drs_config.R_yaw_lidar, map_hdg_measurement_name_id.find("yaw_lidar")->second);

    // Heading rate measurement covariances
    current_hdg_estimator->getR(last_drs_config.R_rate_gyro, map_hdg_measurement_name_id.find("rate_gyro")->second);
    current_hdg_estimator->getR(last_drs_config.R_rate_optflow, map_hdg_measurement_name_id.find("rate_optflow")->second);
    current_hdg_estimator->getR(last_drs_config.R_rate_icp, map_hdg_measurement_name_id.find("rate_icp")->second);
  }

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  reconfigure_server_->updateConfig(last_drs_config);
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
}

//}

/* isReadyToTakeoff() //{ */

bool Odometry::isReadyToTakeoff() {

  // Wait for necessary msgs
  ROS_INFO_ONCE("[Odometry]: Requested %s type for takeoff.", _estimator_type_takeoff.name.c_str());
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOW) {
    if (got_optflow) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for OPTFLOW msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::GPS) {
    if (got_odom_pixhawk) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for PIXHAWK msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    if (got_optflow && got_odom_pixhawk) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for optic flow and pixhawk msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::RTK) {
    // return true, since RTK can work even with normal GPS
    return true;
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::T265) {
    if (got_odom_t265) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for T265 msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::LIDAR) {
    if (got_lidar_odom) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for LIDAR odom msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::VIO) {
    if (got_vio) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for VIO msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::VSLAM) {
    if (got_vslam) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for VSLAM msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::HECTOR) {
    if (got_hector_pose) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for HECTOR pose msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::TOWER) {
    if (got_tower_pose) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for TOWER pose msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::ALOAM) {
    if (got_aloam_odom) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ALOAM odometry msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::ICP) {
    if (got_icp_twist) {
      return true;
    } else {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for ICP twist msg to initialize takeoff estimator");
      return false;
    }
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::BRICK || _estimator_type_takeoff.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Takeoff in this odometry mode is not supported. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  return false;
}

//}

/* //{ isUavFlying() */

bool Odometry::isUavFlying() {

  auto control_manager_diag = mrs_lib::get_mutexed(mutex_control_manager_diag_, control_manager_diag_);

  if (got_control_manager_diag_) {

    if (control_manager_diag.active_tracker == _null_tracker_) {

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

    if (control_manager_diag_.active_tracker == "LandoffTracker") {

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
  {
    std::scoped_lock lock(mutex_estimator_height_);

    lkf_height_t::u_t u;
    u << 0;
    sc_height_       = estimator_height_->predict(sc_height_, u, _Q_height_, dt);
    height_msg.value = sc_height_.x(0);
  }


  try {
    pub_height_.publish(height_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_height_.getTopic().c_str());
  }

  //}

  /* heading estimator prediction //{ */

  double des_yaw;
  double des_yaw_rate;

  // set target attitude input to zero when not receiving target attitude msgs
  if (got_attitude_command) {
    std::scoped_lock lock(mutex_attitude_command_);

    des_yaw      = des_yaw_;
    des_yaw_rate = des_yaw_rate_;
  } else {
    des_yaw      = 0.0;
    des_yaw_rate = 0.0;
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Not receiving target attitude.");
  }

  // apply prediction step to all heading estimators
  headingEstimatorsPrediction(des_yaw, des_yaw_rate, dt);

  //}

  /* lateral estimator prediction //{ */

  geometry_msgs::Point   des_acc_point = mrs_lib::get_mutexed(mutex_attitude_command_, attitude_command_.desired_acceleration);
  geometry_msgs::Vector3 des_acc;
  des_acc.x = des_acc_point.x;
  des_acc.y = des_acc_point.y;
  des_acc.z = des_acc_point.z;

  if (!is_updating_state_) {

    if (isUavFlying()) {
      stateEstimatorsPrediction(des_acc, dt);
    } else {
      des_acc.x = 0.0;
      des_acc.y = 0.0;
      des_acc.z = 0.0;
      stateEstimatorsPrediction(des_acc, dt);
    }

    // correction step for hector

    auto pos_hector_x_tmp = mrs_lib::get_mutexed(mutex_hector, pos_hector_x);
    auto pos_hector_y_tmp = mrs_lib::get_mutexed(mutex_hector, pos_hector_y);
    stateEstimatorsCorrection(pos_hector_x_tmp, pos_hector_y_tmp, "pos_hector");

    // correction step for tower
    auto pos_tower_x_tmp = mrs_lib::get_mutexed(mutex_tower, pos_tower_x);
    auto pos_tower_y_tmp = mrs_lib::get_mutexed(mutex_tower, pos_tower_y);
    stateEstimatorsCorrection(pos_tower_x_tmp, pos_tower_y_tmp, "pos_tower");

    // correction step for aloam
    auto pos_aloam_x_tmp = mrs_lib::get_mutexed(mutex_aloam, pos_aloam_x);
    auto pos_aloam_y_tmp = mrs_lib::get_mutexed(mutex_aloam, pos_aloam_y);
    stateEstimatorsCorrection(pos_aloam_x_tmp, pos_aloam_y_tmp, "pos_aloam");

    // correction step for lidar
    auto pos_lidar_x_tmp = mrs_lib::get_mutexed(mutex_lidar, pos_lidar_x);
    auto pos_lidar_y_tmp = mrs_lib::get_mutexed(mutex_lidar, pos_lidar_y);
    stateEstimatorsCorrection(pos_lidar_x_tmp, pos_lidar_y_tmp, "pos_lidar");

  } else {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Rotating lateral state. Skipping prediction.");
  }

  ROS_INFO_ONCE("[Odometry]: Prediction step of all state estimators running.");

  //}

  // return without publishing when pixhawk or rangefinder measurements are missing
  if (!got_odom_pixhawk || !got_range) {
    ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for altitude data from sensors - received? pixhawk: %s, ranger: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                      got_range ? "TRUE" : "FALSE");
    return;
  }

  /* publish altitude  //{ */

  got_altitude_sensors = true;

  mrs_msgs::Float64Stamped new_altitude;
  {
    std::scoped_lock lock(mutex_odom_pixhawk);

    new_altitude.header = odom_pixhawk.header;
    new_altitude.value  = odom_pixhawk.pose.pose.position.z;
  }

  new_altitude.header.frame_id = local_origin_frame_id_;
  new_altitude.header.stamp    = ros::Time::now();

  // update the altitude state
  Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
  {
    std::scoped_lock lock(mutex_altitude_estimator);
    if (!current_alt_estimator->getStates(current_altitude)) {
      ROS_WARN("[Odometry]: Altitude estimator not initialized.");
      return;
    }
    if (_alt_estimator_type.type == mrs_msgs::AltitudeType::HEIGHT) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (_alt_estimator_type.type == mrs_msgs::AltitudeType::PLANE) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (_alt_estimator_type.type == mrs_msgs::AltitudeType::BRICK) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (_alt_estimator_type.type == mrs_msgs::AltitudeType::VIO) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (_alt_estimator_type.type == mrs_msgs::AltitudeType::ALOAM) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else if (_alt_estimator_type.type == mrs_msgs::AltitudeType::BARO) {
      new_altitude.value = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: unknown altitude type: %d, available types: %d, %d, %d, %d. Publishing mavros altitude instead.",
                         _alt_estimator_type.type, mrs_msgs::AltitudeType::HEIGHT, mrs_msgs::AltitudeType::PLANE, mrs_msgs::AltitudeType::BRICK,
                         mrs_msgs::AltitudeType::VIO);
    }
    ROS_INFO_ONCE("[Odometry]: Publishing altitude from estimator type: %d", _alt_estimator_type.type);
  }

  try {
    pub_altitude_.publish(new_altitude);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_altitude_.getTopic().c_str());
  }

  //}

  /* publish altitude state //{ */

  Eigen::MatrixXd alt_state      = Eigen::MatrixXd::Zero(_altitude_n_, 1);
  Eigen::MatrixXd alt_covariance = Eigen::MatrixXd::Zero(_altitude_n_, _altitude_n_);
  current_alt_estimator->getStates(alt_state);
  current_alt_estimator->getCovariance(alt_covariance);

  mrs_msgs::EstimatedState altitude_state_msg;
  {
    std::scoped_lock lock(mutex_odom_pixhawk);
    altitude_state_msg.header = odom_pixhawk.header;
  }

  altitude_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < _altitude_n_; i++) {
    altitude_state_msg.state.push_back(alt_state(i, 0));
    altitude_state_msg.covariance.push_back(alt_covariance(i, i));
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
  if (!init_hdg_avg_done && toUppercase(current_hdg_estimator_name) == "COMPASS") {
    ROS_INFO_THROTTLE(1.0, "[Odometry]: Waiting for averaging of initial heading.");
    return;
  }

  // initialize heading estimators
  if (!is_heading_estimator_initialized) {

    // prepare initial state
    Eigen::VectorXd yaw       = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd yaw_rate  = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd gyro_bias = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd init_cov  = Eigen::MatrixXd::Identity(_heading_n_, _heading_n_);
    init_cov *= 1000;
    yaw_rate << 0.0;
    gyro_bias << 0.0;
    yaw << init_hdg_avg;

    // set initial state to all estimators
    for (auto &estimator : m_heading_estimators) {
      estimator.second->setState(0, yaw);
      estimator.second->setState(1, yaw_rate);
      estimator.second->setState(2, gyro_bias);
      estimator.second->setCovariance(init_cov);
    }
    is_heading_estimator_initialized = true;
  }

  //}

  /* publish heading  //{ */

  // odometry_msg for compatibility with other packages
  nav_msgs::Odometry orientation;

  // initialize with pixhawk orientation
  {
    std::scoped_lock lock(mutex_odom_pixhawk);
    orientation.header                = odom_pixhawk.header;
    orientation.header.frame_id       = _uav_name_ + "/" + "gps_origin";
    orientation.child_frame_id        = fcu_frame_id_;
    orientation.pose.pose.orientation = odom_pixhawk.pose.pose.orientation;
  }

  // get correct yaw if current heading estimator is not pixhawk
  if (current_hdg_estimator->getName() != "PIXHAWK") {

    Eigen::VectorXd yaw(1);
    Eigen::VectorXd yaw_rate(1);

    {
      std::scoped_lock lock(mutex_current_hdg_estimator);

      current_hdg_estimator->getState(0, yaw);
      current_hdg_estimator->getState(1, yaw_rate);
    }

    yaw(0)                            = mrs_lib::wrapAngle(yaw(0));
    orientation.pose.pose.orientation = mrs_lib::AttitudeConverter(orientation.pose.pose.orientation).setYaw(yaw(0));
    {
      std::scoped_lock lock(mutex_current_hdg_estimator);
      orientation.header.frame_id = current_estimator->getName();
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

  if (failsafe_called) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Failsafe called");
    return;
  }

  // Fallback from PLANE
  if (current_alt_estimator_name == "PLANE") {
    if (!plane_reliable) {
      ROS_WARN("[Odometry]: PLANE not reliable. Switching to HEIGHT type.");
      mrs_msgs::AltitudeType altitude_type;
      altitude_type.type = mrs_msgs::AltitudeType::HEIGHT;
      if (!changeCurrentAltitudeEstimator(altitude_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback height estimator not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
  }

  // Fallback from RTK
  if (_estimator_type.type == mrs_msgs::EstimatorType::RTK) {
    if (!gps_reliable && _optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: RTK not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE", got_rtk ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from GPS
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::GPS) {
    if (!gps_reliable && _optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from OPTFLOWGPS
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    if (!gps_reliable && _optflow_available_ && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_optflow) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }


    // Fallback from T265
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::T265) {
    if ((!got_odom_t265 || !t265_reliable) && _optflow_available_ && got_optflow &&
        current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: T265 not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    } else if ((!got_odom_t265 || !t265_reliable) && gps_reliable && got_odom_pixhawk) {
      ROS_WARN("[Odometry]: T265 not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_odom_t265) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, t265: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                        got_odom_t265 ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from Hector Slam
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::HECTOR) {

    if (!got_hector_pose || !hector_reliable) {

      if (_lidar_available_ && got_icp_twist) {

        if (_perform_hector_reset_routine_ && !hector_reset_routine_running_) {

          ROS_WARN("[Odometry]: HECTOR not reliable. Performing HECTOR reset routine.");
          hector_reset_routine_timer.start();

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
            failsafe_called = true;
          }
        }
      } else if (_optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        if (_perform_hector_reset_routine_ && !hector_reset_routine_running_) {

          ROS_WARN("[Odometry]: HECTOR not reliable. Performing HECTOR reset routine.");
          hector_reset_routine_timer.start();

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
            failsafe_called = true;
          }
        }
      } else if (gps_reliable && got_odom_pixhawk) {
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
          failsafe_called = true;
        }
      } else if (!failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_hector_pose) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, hector: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                        got_hector_pose ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from TOWER
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::TOWER) {

    if (!got_tower_pose || !tower_reliable) {

      if (_lidar_available_ && got_icp_twist) {

        ROS_WARN_THROTTLE(1.0, "[Odometry]: tower heading not reliable. Switching to ICP heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: TOWER not reliable. Switching to ICP type.");
        mrs_msgs::EstimatorType icp_type;
        icp_type.type = mrs_msgs::EstimatorType::ICP;
        if (!changeCurrentEstimator(icp_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
      } else if (_optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: TOWER heading not reliable. Switching to OPTFLOW heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::OPTFLOW;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: TOWER not reliable. Switching to OPTFLOW type.");
        mrs_msgs::EstimatorType optflow_type;
        optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
        if (!changeCurrentEstimator(optflow_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
      } else if (gps_reliable && got_odom_pixhawk) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: TOWER heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::PIXHAWK;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: TOWER not reliable. Switching to PIXHAWK type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
      } else if (!failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_tower_pose) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, tower: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                        got_tower_pose ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from ALOAM Slam
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::ALOAM) {
    if (!got_aloam_odom || !aloam_reliable) {
      if (_aloam_available_ && got_icp_twist) {
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
          failsafe_called = true;
        }
      } else if (_optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
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
          failsafe_called = true;
        }
      } else if (gps_reliable && got_odom_pixhawk) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM heading not reliable. Switching to PIXHAWK heading estimator.");
        mrs_msgs::HeadingType desired_estimator;
        desired_estimator.type = mrs_msgs::HeadingType::ICP;
        desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
        changeCurrentHeadingEstimator(desired_estimator);
        ROS_WARN("[Odometry]: ALOAM not reliable. Switching to PIXHAWK type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
      } else if (!failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_aloam_odom) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, aloam: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                        got_aloam_odom ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from ICP
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::ICP) {

    if (!got_icp_twist || !icp_reliable) {
      if (_optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
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
          failsafe_called = true;
        }
      } else if (gps_reliable && got_odom_pixhawk) {
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
          failsafe_called = true;
        }
      } else if (!failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_icp_twist) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, icp: %s",
                        got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                        got_hector_pose ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }
    // Fallback from BRICK
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::BRICK) {
    if (!got_brick_pose || !brick_reliable) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: BRICK altitude not reliable. Switching to HEIGHT altitude estimator.");
      mrs_msgs::AltitudeType desired_alt_estimator;
      desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
      desired_alt_estimator.name = _altitude_estimators_names_[desired_alt_estimator.type];
      changeCurrentAltitudeEstimator(desired_alt_estimator);
      ROS_WARN_THROTTLE(1.0, "[Odometry]: BRICK heading not reliable. Switching to fallback heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = fallback_brick_hdg_estimator_type.type;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      ROS_WARN("[Odometry]: BRICK not reliable. Switching to %s type.", _estimator_type_names[fallback_brick_estimator_type.type].c_str());
      if (!changeCurrentEstimator(fallback_brick_estimator_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry not available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }

    // Fallback from OPTFLOW
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW) {
    if (gps_reliable) {

      if (!optflow_reliable) {
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
          failsafe_called = true;
        }
      } else if (!got_odom_pixhawk || !got_range || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                          got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                          got_optflow ? "TRUE" : "FALSE");
        if (got_lateral_sensors && !failsafe_called) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
        return;
      }

    } else {
      if (!got_odom_pixhawk || !got_range || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, optflow: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                          got_range ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
        if (got_lateral_sensors && !failsafe_called) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
        return;
      }
    }

  } else if (_estimator_type.type == mrs_msgs::EstimatorType::LIDAR) {
    if (!got_odom_pixhawk || !got_range || !got_lidar_odom) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, lidar: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_lidar_odom ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

    // Fallback from BRICKFLOW
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    if (gps_reliable) {

      if (!optflow_reliable) {
        ROS_WARN("[Odometry]: BRICKFLOW not reliable. Switching to GPS type.");
        mrs_msgs::EstimatorType gps_type;
        gps_type.type = mrs_msgs::EstimatorType::GPS;
        if (!changeCurrentEstimator(gps_type)) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
      } else if (!got_odom_pixhawk || !got_range || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                          got_odom_pixhawk ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_pixhawk_utm ? "TRUE" : "FALSE",
                          got_optflow ? "TRUE" : "FALSE");
        if (got_lateral_sensors && !failsafe_called) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
        return;
      }

    } else {
      if (!got_odom_pixhawk || !got_range || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, optflow: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                          got_range ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
        if (got_lateral_sensors && !failsafe_called) {
          ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
          std_srvs::Trigger failsafe_out;
          ser_client_failsafe_.call(failsafe_out);
          failsafe_called = true;
        }
        return;
      }
    }
    // Fallback from VIO
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::VIO) {
    if (!vio_reliable && _optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: VIO not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    } else if (!vio_reliable && gps_reliable && got_odom_pixhawk) {
      ROS_WARN("[Odometry]: VIO not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_vio) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vio: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_vio ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }
    // Fallback from VSLAM
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::VSLAM) {
    if (!vslam_reliable && _optflow_available_ && got_optflow && current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) < _max_optflow_altitude_) {
      ROS_WARN("[Odometry]: VSLAM not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      if (!changeCurrentEstimator(optflow_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    } else if (!vslam_reliable && gps_reliable && got_odom_pixhawk) {
      ROS_WARN("[Odometry]: VSLAM not reliable. Switching to GPS type.");
      mrs_msgs::EstimatorType gps_type;
      gps_type.type = mrs_msgs::EstimatorType::GPS;
      if (!changeCurrentEstimator(gps_type)) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: Fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
    }
    if (!got_odom_pixhawk || !got_range || !got_vslam) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vslam: %s", got_odom_pixhawk ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_vslam ? "TRUE" : "FALSE");
      if (got_lateral_sensors && !failsafe_called) {
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: No fallback odometry available. Triggering failsafe.");
        std_srvs::Trigger failsafe_out;
        ser_client_failsafe_.call(failsafe_out);
        failsafe_called = true;
      }
      return;
    }

  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Unknown odometry type. Not checking sensors.");
  }

  got_lateral_sensors = true;
  ROS_INFO_ONCE("[Odometry]: Lateral sensors ready");


  //}

  /* reset Hector map after takeoff //{ */


  // Call reset of hector map after taking off - cleans up garbage integrated into map during takeoff
  if (_reset_hector_after_takeoff_ && isUavFlying() && !isUavLandoff() && !hector_reset_called_) {

    for (auto &estimator : m_state_estimators) {
      if (estimator.first == "HECTOR") {
        Vec2 new_offset;
        estimator.second->getState(0, new_offset);
        hector_offset_ += new_offset;
      }
    }

    for (auto &estimator : m_heading_estimators) {
      if (estimator.first == "HECTOR") {
        Eigen::VectorXd tmp_hdg_offset(1);
        estimator.second->getState(0, tmp_hdg_offset);
        hector_offset_hdg_ += tmp_hdg_offset(0);
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
  if (toUppercase(current_estimator_name) == "T265") {

    return;
  }

  /* publish aux odometry //{ */

  // Loop through each estimator
  for (auto &estimator : m_state_estimators) {

    std::map<std::string, nav_msgs::Odometry>::iterator odom_aux = map_estimator_odom.find(estimator.first);

    // Initialize odom_aux with pixhawk_odom to obtain attitude and attitude_rate which are not estimated by us
    mrs_lib::set_mutexed(mutex_odom_pixhawk_shifted, odom_pixhawk_shifted, odom_aux->second);

    std::string estimator_name = estimator.first;
    std::transform(estimator_name.begin(), estimator_name.end(), estimator_name.begin(), ::tolower);
    odom_aux->second.header.frame_id = _uav_name_ + "/" + estimator_name + "_origin";
    odom_aux->second.header.stamp    = time_now;
    odom_aux->second.child_frame_id  = fcu_frame_id_;

    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    // update the altitude state
    {
      std::scoped_lock lock(mutex_altitude_estimator);
      if (!current_alt_estimator->getStates(current_altitude)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    Eigen::VectorXd alt(1);
    if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "PLANE") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else if (estimator.first == "PLANE") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "PLANE") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else if (estimator.first == "VIO") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "VIO") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else if (estimator.first == "ALOAM") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "ALOAM") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
      // we might want other than height estimator when in GPS (baro)
    } else if (estimator.first == "GPS") {
      {
        std::scoped_lock lock(mutex_altitude_estimator);
        if (!current_alt_estimator->getStates(current_altitude)) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
          return;
        }
        odom_aux->second.pose.pose.position.z = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
      }
    } else {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "HEIGHT") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    }

    if (estimator.second->getName() == "RTK" && _pass_rtk_as_odom_) {

      mrs_lib::set_mutexed(mutex_rtk_local_odom, rtk_local_odom.pose.pose.position.z, odom_aux->second.pose.pose.position.z);
    }

    if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) == _fcu_height_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Suspicious height detected: %f, %f, %f. Check if altitude fusion is running correctly",
                        current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), current_altitude(mrs_msgs::AltitudeStateNames::VELOCITY),
                        current_altitude(mrs_msgs::AltitudeStateNames::ACCELERATION));
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
    Eigen::VectorXd hdg_vec(1);
    Eigen::VectorXd hdg_vel_vec(1);

    for (auto &hdg_estimator : m_heading_estimators) {

      if (hdg_estimator.first == estimator.first || (hdg_estimator.first == "BRICK" && estimator.first == "BRICKFLOW")) {

        hdg_estimator.second->getState(0, hdg_vec);
        hdg_estimator.second->getState(1, hdg_vel_vec);
        odom_aux->second.pose.pose.orientation = mrs_lib::AttitudeConverter(odom_aux->second.pose.pose.orientation).setYaw(hdg_vec(0));
        odom_aux->second.twist.twist.angular.z = hdg_vel_vec(0);
      }
    }

    // Get inverse trasnform
    tf2::Transform tf_inv        = mrs_uav_odometry::tf2FromPose(odom_aux->second.pose.pose);
    tf_inv                       = tf_inv.inverse();
    geometry_msgs::Pose pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

    // publish TF
    geometry_msgs::TransformStamped tf;
    tf.header.stamp          = ros::Time::now();
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

    // Transform global velocity to twist in fcu frame
    geometry_msgs::Vector3Stamped global_vel;
    global_vel.header.frame_id = odom_aux->second.header.frame_id;
    global_vel.header.stamp    = ros::Time::now();
    global_vel.vector.x        = vel_vec(0);
    global_vel.vector.y        = vel_vec(1);
    global_vel.vector.z        = current_altitude(mrs_msgs::AltitudeStateNames::VELOCITY);

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

    std::map<std::string, ros::Publisher>::iterator pub_odom_aux = map_estimator_pub.find(estimator.second->getName());

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
  for (auto &estimator : m_heading_estimators) {

    mrs_msgs::Float64ArrayStamped heading_aux;

    heading_aux.header.frame_id = local_origin_frame_id_;
    heading_aux.header.stamp    = time_now;

    Eigen::MatrixXd current_heading = Eigen::MatrixXd::Zero(_heading_n_, 1);
    // update the altitude state
    {
      std::scoped_lock lock(mutex_heading_estimator);
      if (!estimator.second->getStates(current_heading)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Heading estimator not initialized.");
        return;
      }
    }

    current_heading(0) = mrs_lib::wrapAngle(current_heading(0));

    for (int i = 0; i < current_heading.rows(); i++) {
      heading_aux.values.push_back(current_heading(i));
    }

    std::map<std::string, ros::Publisher>::iterator pub_hdg_aux = map_hdg_estimator_pub.find(estimator.second->getName());

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
  if (_gps_available_ && !calculatePixhawkOdomOffset()) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Cannot calculate pixhawk odom offset.");
    return;
  }

  if (_gps_available_ && !got_pixhawk_odom_shifted) {
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

  auto odom_pixhawk_shifted_local = mrs_lib::get_mutexed(mutex_odom_pixhawk_shifted, odom_pixhawk_shifted);

  odom_main                  = odom_pixhawk_shifted_local;
  uav_state.pose.orientation = odom_pixhawk_shifted_local.pose.pose.orientation;
  uav_state.velocity.angular = odom_pixhawk_shifted_local.twist.twist.angular;

  // Fill in odometry headers according to the uav name and current estimator
  odom_main.header.stamp    = ros::Time::now();
  odom_main.header.frame_id = _uav_name_ + "/" + toLowercase(current_estimator_name) + "_origin";
  odom_main.child_frame_id  = fcu_frame_id_;
  uav_state.header.stamp    = ros::Time::now();
  uav_state.header.frame_id = _uav_name_ + "/" + toLowercase(current_estimator_name) + "_origin";
  uav_state.child_frame_id  = fcu_frame_id_;

  /* initialize lateral kalman filters //{ */

  if (!odometry_published) {

    ROS_INFO("[Odometry]: Initializing the states of all estimators");

    for (auto &estimator : m_state_estimators) {

      Eigen::VectorXd state(2);

      // estimators not based on GNSS
      if (estimator.second->getName() == "OPTFLOW" || estimator.second->getName() == "HECTOR" || estimator.second->getName() == "TOWER" ||
          estimator.second->getName() == "ALOAM" || estimator.second->getName() == "BRICK" || estimator.second->getName() == "VIO" ||
          estimator.second->getName() == "VSLAM" || estimator.second->getName() == "BRICKFLOW" || estimator.second->getName() == "ICP") {

        state << _local_origin_x_, _local_origin_y_;
        estimator.second->setState(0, state);


        // GNSS based estimators (GPS)
      } else {

        double pos_x = odom_pixhawk_shifted_local.pose.pose.position.x;
        double pos_y = odom_pixhawk_shifted_local.pose.pose.position.y;
        state << pos_x, pos_y;
        estimator.second->setState(0, state);
      }

      // RTK estimator

      state(0) = odom_pixhawk_shifted_local.pose.pose.position.x;
      state(1) = odom_pixhawk_shifted_local.pose.pose.position.y;
      {
        std::scoped_lock lock(mutex_rtk_est_);
        sc_lat_rtk_.x = state;
      }
    }

    if (toUppercase(current_estimator_name) != "GPS" && toUppercase(current_estimator_name) != "RTK") {
      odom_main.pose.pose.position.x  = _local_origin_x_;
      odom_main.pose.pose.position.y  = _local_origin_y_;
      odom_main.pose.pose.orientation = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation).setYaw(0.0);
    }

    // initialize stable odometry
    odom_stable       = odom_main;
    last_stable_name_ = odom_main.header.frame_id;
    first_frame_      = odom_main.header.frame_id;

    // initialize offset of stable_origin
    odom_stable_pos_offset_.setX(0.0);
    odom_stable_pos_offset_.setY(0.0);
    odom_stable_pos_offset_.setZ(0.0);
    odom_stable_rot_offset_ = mrs_lib::AttitudeConverter(0, 0, 0);

    // initialize local odometry
    odom_local       = odom_main;
    last_local_name_ = odom_main.header.frame_id;

    // initialize offset of local_origin
    m_pos_odom_offset.setX(odom_main.pose.pose.position.x);
    m_pos_odom_offset.setY(odom_main.pose.pose.position.y);

    {
      std::scoped_lock lock(mutex_altitude_estimator);

      if (!current_alt_estimator->getStates(current_altitude)) {
        ROS_WARN("[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    m_pos_odom_offset.setZ(current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT));
    double yaw_tmp = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation).getYaw();

    m_rot_odom_offset = mrs_lib::AttitudeConverter(0, 0, yaw_tmp);

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
      std::scoped_lock lock(mutex_altitude_estimator);

      if (!current_alt_estimator->getStates(current_altitude)) {
        ROS_WARN("[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    //}

    /* get lateral states from current filter //{ */

    Vec2 pos_vec, vel_vec, acc_vec;
    {
      std::scoped_lock lock(mutex_current_estimator);

      current_estimator->getState(0, pos_vec);
      current_estimator->getState(1, vel_vec);
      current_estimator->getState(2, acc_vec);
    }

    odom_main.pose.pose.position.z  = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    uav_state.pose.position.z       = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);
    uav_state.acceleration.linear.z = current_altitude(mrs_msgs::AltitudeStateNames::ACCELERATION);

    //}

    // Transform global velocity to twist in fcu frame
    geometry_msgs::Vector3Stamped global_vel;
    global_vel.header.frame_id = odom_main.header.frame_id;
    global_vel.header.stamp    = ros::Time::now();
    global_vel.vector.x        = vel_vec(0);
    global_vel.vector.y        = vel_vec(1);
    global_vel.vector.z        = current_altitude(mrs_msgs::AltitudeStateNames::VELOCITY);

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

    uav_state.estimator_horizontal = _estimator_type;
    uav_state.estimator_vertical   = _alt_estimator_type;
    uav_state.estimator_heading    = _hdg_estimator_type;
    uav_state.estimator_iteration  = estimator_iteration_;

    //}

    /* fill in the current heading //{ */

    // get heading states from current filter
    std::string current_hdg_estimator_name_local;
    {
      std::scoped_lock lock(mutex_current_hdg_estimator);
      current_hdg_estimator_name_local = current_hdg_estimator->getName();
    }

    if (current_hdg_estimator_name_local != "PIXHAWK") {

      Eigen::VectorXd yaw(1);
      Eigen::VectorXd yaw_rate(1);

      {
        std::scoped_lock lock(mutex_current_hdg_estimator);

        current_hdg_estimator->getState(0, yaw);
        current_hdg_estimator->getState(1, yaw_rate);
      }

      odom_main.pose.pose.orientation = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation).setYaw(yaw(0));
      odom_main.twist.twist.angular.z = yaw_rate(0);
      uav_state.pose.orientation      = mrs_lib::AttitudeConverter(uav_state.pose.orientation).setYaw(yaw(0));
      uav_state.velocity.angular.z    = yaw_rate(0);

      uav_state.estimator_heading = _hdg_estimator_type;
    }

    //}

    /* fill in the current position //{ */

    if (toUppercase(current_estimator_name) == "RTK") {
      {
        std::scoped_lock lock(mutex_rtk_est_);

        odom_main.pose.pose.position.x = sc_lat_rtk_.x(0);
        odom_main.pose.pose.position.y = sc_lat_rtk_.x(1);
      }
      uav_state.pose.position.x = odom_main.pose.pose.position.x;
      uav_state.pose.position.y = odom_main.pose.pose.position.y;
    } else {
      odom_main.pose.pose.position.x = pos_vec(0);
      odom_main.pose.pose.position.y = pos_vec(1);
      uav_state.pose.position.x      = pos_vec(0);
      uav_state.pose.position.y      = pos_vec(1);
    }

    //}

    /* fill in the velocity //{ */

    // mavros velocity is correct only in the PIXHAWK heading estimator frame, our velocity estimate should be more accurate anyway
    // mavros velocity should be used only for debug and estimation baseline
    if (!_publish_pixhawk_velocity_) {
      odom_main.twist.twist.linear.x = body_vel.vector.x;
      odom_main.twist.twist.linear.y = body_vel.vector.y;
    }
    odom_main.twist.twist.linear.z = body_vel.vector.z;

    uav_state.velocity.linear.x = vel_vec(0);
    uav_state.velocity.linear.y = vel_vec(1);
    uav_state.velocity.linear.z = current_altitude(mrs_msgs::AltitudeStateNames::VELOCITY);


    //}

    /* fill in the accelerations //{ */

    uav_state.acceleration.linear.x = acc_vec(0);
    uav_state.acceleration.linear.y = acc_vec(1);

    //}

    /* pass rtk as odometry //{ */

    if (_pass_rtk_as_odom_) {

      auto rtk_local_odom_tmp = mrs_lib::get_mutexed(mutex_rtk_local_odom, rtk_local_odom);

      // TODO transform twist to body frame
      odom_main                 = rtk_local_odom_tmp;
      odom_main.header.frame_id = _uav_name_ + "/rtk_origin";  // TODO does this not cause problems?
      odom_main.child_frame_id  = fcu_frame_id_;

      uav_state.header.frame_id = _uav_name_ + "/rtk_origin";
      uav_state.pose            = rtk_local_odom_tmp.pose.pose;
      uav_state.velocity        = rtk_local_odom_tmp.twist.twist;
    }

    //}

    /* obtain local odom offset //{ */

    if (odom_main.header.frame_id != last_local_name_) {
      ROS_WARN("[Odometry]: Changing odometry estimator from %s to %s. Updating offset for stable odometry.", last_local_name_.c_str(),
               odom_main.header.frame_id.c_str());

      last_local_name_ = odom_main.header.frame_id;
      tf2::Vector3 v1, v2;
      tf2::fromMsg(odom_main.pose.pose.position, v1);
      tf2::fromMsg(odom_local.pose.pose.position, v2);
      tf2::Vector3 pos_diff = v1 - v2;
      m_pos_odom_offset     = pos_diff;

      if (odom_local.pose.pose.orientation.w == 0.0) {
        ROS_WARN("[Odometry]: Odom stable quaternion x: %f y: %f z: %f w: %f", odom_local.pose.pose.orientation.x, odom_local.pose.pose.orientation.y,
                 odom_local.pose.pose.orientation.z, odom_local.pose.pose.orientation.w);
        odom_local.pose.pose.orientation = odom_main.pose.pose.orientation;  // this can cause problems TODO find out why it is happening
      }
      tf2::Quaternion q1       = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation);
      tf2::Quaternion q2       = mrs_lib::AttitudeConverter(odom_local.pose.pose.orientation);
      tf2::Quaternion rot_diff = q2 * q1.inverse();
      m_rot_odom_offset        = rot_diff;
      m_rot_odom_offset.normalize();
      /* ROS_WARN("[Odometry]: odometry change stable_q: %f, %f, %f, %f", odom_local.pose.pose.orientation.x, odom_local.pose.pose.orientation.y,
       * odom_local.pose.pose.orientation.z, odom_local.pose.pose.orientation.w); */
      /* ROS_WARN("[Odometry]: q1: %f, %f, %f, %f,\t q2: %f, %f, %f, %f", q1.x(), q1.y(), q1.z(), q1.w(), q2.x(), q2.y(), q2.z(), q2.w()); */
      ROS_WARN("[Odometry]: pos_diff: x: %f y: %f z: %f", pos_diff.getX(), pos_diff.getY(), pos_diff.getZ());
    }

    /* ROS_WARN("[Odometry]: before stable_q: %f, %f, %f, %f", odom_local.pose.pose.orientation.x, odom_local.pose.pose.orientation.y,
     * odom_local.pose.pose.orientation.z, odom_local.pose.pose.orientation.w); */
    odom_local = applyOdomOffset(odom_main, m_pos_odom_offset, m_rot_odom_offset);
    /* ROS_WARN("[Odometry]: after stable_q: %f, %f, %f, %f", odom_local.pose.pose.orientation.x, odom_local.pose.pose.orientation.y,
     * odom_local.pose.pose.orientation.z, odom_local.pose.pose.orientation.w); */
    odom_local.header.frame_id = local_origin_frame_id_;

    //}

    /* publish local odometry //{ */

    try {
      pub_odom_local_.publish(odom_local);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_local_.getTopic().c_str());
    }

    //}

    /* publish local origin tf //{ */

    // Get inverse trasnform
    tf2::Transform tf_inv        = mrs_uav_odometry::tf2FromPose(odom_local.pose.pose);
    tf_inv                       = tf_inv.inverse();
    geometry_msgs::Pose pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

    geometry_msgs::TransformStamped tf;
    tf.header.stamp          = ros::Time::now();
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
  }

  mrs_lib::set_mutexed(mutex_shared_odometry, odom_main, shared_odom);

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
    pub_odom_main_inno_.publish(odom_main_inno);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_main_inno_.getTopic().c_str());
  }

  //}

  odometry_published = true;

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
    tf_stable.header.stamp          = ros::Time::now();
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

/* //{ auxTimer() */

void Odometry::auxTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("auxTimer", _aux_rate_, 0.004, event);

  ros::Time t_pub = ros::Time::now();

  // Loop through each estimator
  for (auto &estimator : m_state_estimators) {

    std::map<std::string, nav_msgs::Odometry>::iterator odom_aux = map_estimator_odom.find(estimator.first);

    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      odom_aux->second = odom_pixhawk;
    }

    std::string estimator_name = estimator.first;
    std::transform(estimator_name.begin(), estimator_name.end(), estimator_name.begin(), ::tolower);
    odom_aux->second.header.frame_id = _uav_name_ + "/" + estimator_name + "_origin";
    odom_aux->second.header.stamp    = t_pub;
    odom_aux->second.child_frame_id  = fcu_frame_id_;

    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    // update the altitude state
    {
      std::scoped_lock lock(mutex_altitude_estimator);
      if (!current_alt_estimator->getStates(current_altitude)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }
    }

    Eigen::VectorXd alt(1);
    if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "BRICK") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else if (estimator.first == "PLANE") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "PLANE") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else if (estimator.first == "VIO") {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "VIO") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    } else {
      for (auto &alt_estimator : m_altitude_estimators) {
        if (alt_estimator.first == "HEIGHT") {
          alt_estimator.second->getState(0, alt);
          odom_aux->second.pose.pose.position.z = alt(0);
        }
      }
    }

    if (estimator.second->getName() == "RTK" && _pass_rtk_as_odom_) {
      {
        std::scoped_lock lock(mutex_rtk_local_odom);

        odom_aux->second.pose.pose.position.z = rtk_local_odom.pose.pose.position.z;
      }
    }

    if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) == _fcu_height_) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Suspicious height detected: %f, %f, %f. Check if altitude fusion is running correctly",
                        current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), current_altitude(mrs_msgs::AltitudeStateNames::VELOCITY),
                        current_altitude(mrs_msgs::AltitudeStateNames::ACCELERATION));
    }

    Vec2 pos_vec;
    Vec2 vel_vec;

    estimator.second->getState(0, pos_vec);
    estimator.second->getState(1, vel_vec);

    odom_aux->second.pose.pose.position.x = pos_vec(0);
    odom_aux->second.pose.pose.position.y = pos_vec(1);

    // Loop through each heading estimator
    Eigen::VectorXd hdg_vec(1);
    Eigen::VectorXd hdg_vel_vec(1);

    for (auto &hdg_estimator : m_heading_estimators) {

      if (hdg_estimator.first == estimator.first || (hdg_estimator.first == "BRICK" && estimator.first == "BRICKFLOW")) {

        hdg_estimator.second->getState(0, hdg_vec);
        hdg_estimator.second->getState(1, hdg_vel_vec);
        odom_aux->second.pose.pose.orientation = mrs_lib::AttitudeConverter(odom_aux->second.pose.pose.orientation).setYaw(hdg_vec(0));
        odom_aux->second.twist.twist.angular.z = hdg_vel_vec(0);
      }
    }

    std::map<std::string, ros::Publisher>::iterator pub_odom_aux = map_estimator_pub.find(estimator.second->getName());

    // Publish odom
    try {
      pub_odom_aux->second.publish(odom_aux->second);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_aux->second.getTopic().c_str());
    }

    // Get inverse trasnform
    tf2::Transform tf_inv        = mrs_uav_odometry::tf2FromPose(odom_aux->second.pose.pose);
    tf_inv                       = tf_inv.inverse();
    geometry_msgs::Pose pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

    // publish TF
    geometry_msgs::TransformStamped tf;
    tf.header.stamp          = ros::Time::now();
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

  // publish the static transform between utm and local gps origin
  if (gps_reliable) {

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
  for (auto &estimator : m_heading_estimators) {

    mrs_msgs::Float64ArrayStamped heading_aux;

    heading_aux.header.frame_id = local_origin_frame_id_;
    heading_aux.header.stamp    = t_pub;

    Eigen::MatrixXd current_heading = Eigen::MatrixXd::Zero(_heading_n_, 1);
    // update the altitude state
    {
      std::scoped_lock lock(mutex_heading_estimator);
      if (!estimator.second->getStates(current_heading)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Heading estimator not initialized.");
        return;
      }
    }

    current_heading(0) = mrs_lib::wrapAngle(current_heading(0));

    for (int i = 0; i < current_heading.rows(); i++) {
      heading_aux.values.push_back(current_heading(i));
    }

    std::map<std::string, ros::Publisher>::iterator pub_hdg_aux = map_hdg_estimator_pub.find(estimator.second->getName());

    try {
      pub_hdg_aux->second.publish(heading_aux);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_hdg_aux->second.getTopic().c_str());
    }
  }
  ROS_INFO_ONCE("[Odometry]: Publishing auxiliary odometry");
}

//}

/* //{ slowOdomTimer() */

void Odometry::slowOdomTimer(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("slowOdomTimer", _slow_odom_rate_, 0.01, event);

  nav_msgs::Odometry slow_odom;

  {
    std::scoped_lock lock(mutex_shared_odometry);

    slow_odom = shared_odom;
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
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_estimator_name).c_str(), toUppercase(current_alt_estimator_name).c_str(),
                        toUppercase(current_hdg_estimator_name).c_str(), gps_cov_tmp, max_altitude_, _hiccup_thr_, c_hiccup_tmp);
    } else {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. GPS Cov: %.2f. Max alt: %.2f.",
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_estimator_name).c_str(), toUppercase(current_alt_estimator_name).c_str(),
                        toUppercase(current_hdg_estimator_name).c_str(), gps_cov_tmp, max_altitude_);
    }
  } else {
    if (c_hiccup_tmp > 0) {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. Max alt: %.2f. Hiccups > %.2f: %d",
                        (ros::Time::now() - t_start).toSec(), toUppercase(current_estimator_name).c_str(), toUppercase(current_alt_estimator_name).c_str(),
                        toUppercase(current_hdg_estimator_name).c_str(), max_altitude_, _hiccup_thr_, c_hiccup_tmp);
    } else {
      ROS_INFO_THROTTLE(5.0, "[Odometry]: Running for %.2f s. Estimators: Lat: %s, Alt: %s, Hdg: %s. Max alt: %.2f.", (ros::Time::now() - t_start).toSec(),
                        toUppercase(current_estimator_name).c_str(), toUppercase(current_alt_estimator_name).c_str(),
                        toUppercase(current_hdg_estimator_name).c_str(), max_altitude_);
    }
  }

  mrs_msgs::OdometryDiag odometry_diag;

  odometry_diag.header.stamp = ros::Time::now();

  odometry_diag.estimator_type = _estimator_type;

  odometry_diag.max_altitude      = mrs_lib::get_mutexed(mutex_max_altitude_, max_altitude_);
  odometry_diag.gps_reliable      = gps_reliable;
  odometry_diag.gps_available     = _gps_available_;
  odometry_diag.optflow_available = _optflow_available_;
  odometry_diag.rtk_available     = _rtk_available_;
  odometry_diag.lidar_available   = _lidar_available_;
  odometry_diag.aloam_available   = _aloam_available_;
  odometry_diag.height_available  = height_available_;


  try {
    pub_odometry_diag_.publish(odometry_diag);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odometry_diag_.getTopic().c_str());
  }

  mrs_msgs::ReferenceStamped servoing_diag_out;

  servoing_diag_out.header.stamp    = ros::Time::now();
  servoing_diag_out.header.frame_id = "visual_servoing_debug";

  servoing_diag_out.reference.position.x = c_failed_brick_x_;
  servoing_diag_out.reference.position.y = c_failed_brick_y_;
  servoing_diag_out.reference.position.z = c_failed_brick_timeout_;
  servoing_diag_out.reference.heading    = c_failed_brick_yaw_;

  try {
    pub_brick_diag_.publish(servoing_diag_out);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_brick_diag_.getTopic().c_str());
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
    std::scoped_lock lock(mutex_current_estimator);

    current_estimator->getStates(states_mat);
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

  Eigen::MatrixXd hdg_state      = Eigen::MatrixXd::Zero(_heading_n_, 1);
  Eigen::MatrixXd hdg_covariance = Eigen::MatrixXd::Zero(_heading_n_, _heading_n_);
  {
    std::scoped_lock lock(mutex_current_hdg_estimator);
    current_hdg_estimator->getStates(hdg_state);
    current_hdg_estimator->getCovariance(hdg_covariance);
  }

  mrs_msgs::EstimatedState hdg_state_msg;
  hdg_state_msg.header.stamp = ros::Time::now();

  if (current_hdg_estimator->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk);
    double           yaw = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).getYaw();
    hdg_state_msg.state.push_back(yaw);

  } else {
    hdg_state(0, 0) = mrs_lib::wrapAngle(hdg_state(0, 0));
    for (int i = 0; i < _heading_n_; i++) {
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
  max_altitude_m_sg.header.frame_id = _uav_name_ + "/" + toLowercase(current_estimator_name) + "_origin";
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

  if (got_rtk) {

    if (got_rtk_counter < 15) {

      ROS_ERROR_THROTTLE(1.0, "[Odometry]: RTK comming at slow rate (%d Hz)!", got_rtk_counter);
    }

    got_rtk_counter = 0;
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
  interval = ros::Time::now() - garmin_last_update;
  if (height_available_ && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Garmin range not received for %f seconds.", interval.toSec());
    height_available_ = false;
  }

  // pixhawk odometry
  interval = ros::Time::now() - odom_pixhawk_last_update;
  if (got_odom_pixhawk && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Pixhawk odometry not received for %f seconds.", interval.toSec());
    got_odom_pixhawk = false;
  }

  // optflow velocities (corrections of lateral kf)
  interval = ros::Time::now() - optflow_twist_last_update;
  if (got_optflow && interval.toSec() > 0.1) {
    ROS_WARN("[Odometry]: Optflow twist not received for %f seconds.", interval.toSec());
    if (got_optflow && interval.toSec() > 1.0) {
      got_optflow      = false;
      optflow_reliable = false;
    }
  }

  //  target attitude (input to lateral kf)
  interval = ros::Time::now() - attitude_command_last_update_;
  if (got_attitude_command && interval.toSec() > 0.1) {
    ROS_WARN("[Odometry]: Attitude command not received for %f seconds.", interval.toSec());
    if (got_attitude_command && interval.toSec() > 1.0) {
      got_attitude_command = false;
    }
  }

  // control acceleration (input to altitude kf)
  interval = ros::Time::now() - control_accel_last_update;
  if (got_control_accel && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Control acceleration not received for %f seconds.", interval.toSec());
    got_control_accel = false;
  }

  // IMU data (corrections to altitude, lateral and heading kf)
  interval = ros::Time::now() - pixhawk_imu_last_update;
  if (got_pixhawk_imu && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: IMU data not received for %f seconds.", interval.toSec());
    got_pixhawk_imu = false;
  }

  //  vio odometry (corrections of lateral kf)
  interval = ros::Time::now() - odom_vio_last_update;
  if (got_vio && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: VIO odometry not received for %f seconds.", interval.toSec());
    got_vio = false;
  }

  //  vslam pose (corrections of lateral kf)
  interval = ros::Time::now() - pose_vslam_last_update;
  if (got_vslam && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: VSLAM odometry not received for %f seconds.", interval.toSec());
    got_vslam = false;
  }

  //  brick odometry (corrections of lateral kf)
  interval = ros::Time::now() - brick_pose_last_update;
  if (got_brick_pose && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: BRICK odometry not received for %f seconds.", interval.toSec());
    got_brick_pose = false;
    brick_reliable = false;
  }

  //  lidar velocities (corrections of lateral kf)
  interval = ros::Time::now() - lidar_odom_last_update;
  if (got_lidar_odom && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: LIDAR velocities not received for %f seconds.", interval.toSec());
    got_lidar_odom = false;
  }
  //  icp twist global
  interval = ros::Time::now() - icp_twist_last_update;
  if (got_icp_twist && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP velocities not received for %f seconds.", interval.toSec());
    got_icp_twist = false;
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
  if (_lidar_available_ && got_icp_twist) {
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
      failsafe_called = true;
      return;
    } else {
      in_icp = true;
    }
  }

  // Change estimator to optflow
  if (_optflow_available_ && got_optflow && !in_icp) {
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
      failsafe_called = true;
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
  for (auto &estimator : m_heading_estimators) {
    if (estimator.first == "HECTOR") {
      Eigen::VectorXd hdg(1);
      Eigen::VectorXd hdg_vel(1);
      Eigen::VectorXd hdg_acc(1);
      hdg << 0;
      hdg_vel << 0;
      hdg_acc << 0;
      estimator.second->setState(0, hdg);
      estimator.second->setState(1, hdg_vel);
      estimator.second->setState(2, hdg_acc);
    }
  }

  // Reset HECTOR position
  for (auto &estimator : m_state_estimators) {
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
  hector_reliable = true;
  mrs_msgs::EstimatorType hector_type;
  hector_type.type = mrs_msgs::EstimatorType::HECTOR;
  if (!changeCurrentEstimator(hector_type)) {
    ROS_WARN("[Odometry]: Switching back to HECTOR type after map reset failed.");
  }

  // Enable callbacks
  callEnableControlCallbacks();

  // Prepare timer for next run
  hector_reset_routine_running_ = false;
  hector_reset_routine_timer.stop();
  hector_reset_routine_timer.setPeriod(ros::Duration(0.00001));
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

    if (got_attitude_command) {

      attitude_command_prev_ = attitude_command_;
      attitude_command_      = *msg;

      attitude_command_last_update_ = ros::Time::now();

    } else {

      attitude_command_      = *msg;
      attitude_command_prev_ = attitude_command_;

      attitude_command_last_update_ = ros::Time::now();
      got_attitude_command          = true;
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
  {
    std::scoped_lock lock(mutex_attitude_command_);
    des_yaw_rate_ = attitude_command_.attitude_rate.z;
    des_yaw_      = mrs_lib::AttitudeConverter(attitude_command_.attitude).getYaw();

    if (!std::isfinite(des_yaw_rate_)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in Mavros variable \"des_yaw_rate_\", prediction with zero input!!!");
      des_yaw_rate_ = 0.0;
    }

    if (!isUavFlying()) {
      des_yaw_rate_ = 0.0;
    }
  }

  double mes;
  {
    std::scoped_lock lock(mutex_attitude_command_);
    mes = attitude_command_.desired_acceleration.z;
  }

  Eigen::VectorXd input(1);
  input(0) = mes;

  for (auto &estimator : m_altitude_estimators) {
    estimator.second->doPrediction(input);
  }
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

  if (got_odom_pixhawk) {

    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      odom_pixhawk_previous         = odom_pixhawk;
      odom_pixhawk_previous_shifted = odom_pixhawk_shifted;
      odom_pixhawk                  = *msg;
      odom_pixhawk_shifted          = *msg;

      // shift the odom_pixhawk_shifted
      if (got_pixhawk_odom_offset) {
        odom_pixhawk_shifted.pose.pose.position.x += pixhawk_odom_offset_x;
        odom_pixhawk_shifted.pose.pose.position.y += pixhawk_odom_offset_y;
        got_pixhawk_odom_shifted = true;
      }
    }

  } else {

    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      odom_pixhawk_previous = *msg;
      odom_pixhawk          = *msg;

      // store the initial magnetic heading (corresponding to 0 of non-magnetic heading estimators)
      init_magnetic_heading_ = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).getYaw();
    }

    if (_simulation_) {
      mavros_glitch.x = 0.0;
      mavros_glitch.y = 0.0;
      mavros_glitch.z = 0.0;
    }

    got_odom_pixhawk         = true;
    odom_pixhawk_last_update = ros::Time::now();
    return;
  }

  odom_pixhawk_last_update = ros::Time::now();

  if (!isTimestampOK(odom_pixhawk.header.stamp.toSec(), odom_pixhawk_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk odom timestamp not OK, not fusing correction.");
    return;
  }

  tf2::Quaternion q;
  {
    std::scoped_lock lock(mutex_odom_pixhawk);

    q = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).setYaw(0.0);
  }
  q = q.inverse();

  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
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
    std::scoped_lock lock(mutex_odom_pixhawk);

    dt = (odom_pixhawk.header.stamp - odom_pixhawk_previous.header.stamp).toSec();
  }

  // Negate weird simulation position jump glitches (caused by gazebo timestamp glitches - should be handled by isTimestampOK(), this check is for redundancy)
  // (smaller jumps in GPS position are handled by safety control mechanisms)
  if (_simulation_) {

    if (std::fabs(odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous.pose.pose.position.x) > 100) {
      mavros_glitch.x = odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous.pose.pose.position.x;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current x: %f, Previous x: %f", odom_pixhawk.pose.pose.position.x,
               odom_pixhawk_previous.pose.pose.position.x);
    }
    if (std::fabs(odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous.pose.pose.position.y) > 100) {
      mavros_glitch.y = odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous.pose.pose.position.y;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current y: %f, Previous y: %f", odom_pixhawk.pose.pose.position.y,
               odom_pixhawk_previous.pose.pose.position.y);
    }
    if (std::fabs(odom_pixhawk.pose.pose.position.z - odom_pixhawk_previous.pose.pose.position.z) > 100) {
      mavros_glitch.z = odom_pixhawk.pose.pose.position.z - odom_pixhawk_previous.pose.pose.position.z;
      ROS_WARN("[Odometry]: Mavros position glitch detected. Current z: %f, Previous z: %f", odom_pixhawk.pose.pose.position.z,
               odom_pixhawk_previous.pose.pose.position.z);
    }

    odom_pixhawk_shifted.pose.pose.position.x -= mavros_glitch.x;
    odom_pixhawk_shifted.pose.pose.position.y -= mavros_glitch.y;
    odom_pixhawk_shifted.pose.pose.position.z -= mavros_glitch.z;
  }

  if (!got_init_heading) {

    double hdg       = getCurrentHeading();
    m_init_heading   = hdg;
    got_init_heading = true;
  }

  if (!got_range) {

    return;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------


  // Transform twist from body frame to ENU frame
  tf2::Transform         tf_pixhawk(mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation));
  tf2::Vector3           vel_body(odom_pixhawk.twist.twist.linear.x, odom_pixhawk.twist.twist.linear.y, odom_pixhawk.twist.twist.linear.z);
  tf2::Vector3           vel_enu_tmp = tf_pixhawk * vel_body;
  geometry_msgs::Vector3 vel_enu;
  vel_enu.x = vel_enu_tmp.getX();
  vel_enu.y = vel_enu_tmp.getY();
  vel_enu.z = vel_enu_tmp.getZ();

  /* altitude estimator update //{ */

  // set the input vector
  Eigen::VectorXd input;
  input = Eigen::VectorXd::Zero(_altitude_m_);

  if (is_altitude_estimator_initialized) {

    /* publish covariance of altitude states //{ */

    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(_altitude_n_, _altitude_n_);
    {
      std::scoped_lock lock(mutex_altitude_estimator);
      current_alt_estimator->getCovariance(cov);
    }
    mrs_msgs::Float64ArrayStamped cov_msg;
    cov_msg.header.stamp    = ros::Time::now();
    cov_msg.header.frame_id = local_origin_frame_id_;
    for (int i = 0; i < cov.rows(); i++) {
      cov_msg.values.push_back(cov(i, i));
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
    for (auto &estimator : m_altitude_estimators) {
      int n_states;
      if (!estimator.second->getN(n_states)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }
      Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);

      if (!current_alt_estimator->getStates(current_altitude)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }

      double altitude;
      /* double _altitude_p_revious; */
      double twist_z;
      {
        std::scoped_lock lock(mutex_odom_pixhawk);
        altitude = odom_pixhawk.pose.pose.position.z - baro_offset_;
        /* _altitude_p_revious = odom_pixhawk_previous.pose.pose.position.z; */
        twist_z = vel_enu.z;
      }

      // fuse zero into baro estimator when on the ground
      if (!isUavFlying()) {
        altitude        = 0.0;
        baro_corrected_ = false;
      } else if (!baro_corrected_) {

        auto odom_pixhawk_tmp = mrs_lib::get_mutexed(mutex_odom_pixhawk, odom_pixhawk);
        auto range_garmin_tmp = mrs_lib::get_mutexed(mutex_range_garmin, range_garmin);
        baro_offset_          = odom_pixhawk_tmp.pose.pose.position.z - range_garmin_tmp.range;
        baro_corrected_       = true;
      }


      {
        std::scoped_lock lock(mutex_altitude_estimator);
        altitudeEstimatorCorrection(twist_z, "vel_baro");
        altitudeEstimatorCorrection(altitude, "height_baro");
      }
    }

    ROS_WARN_ONCE("[Odometry]: fusing barometer altitude");

    //}

  } else {
    Eigen::VectorXd zero_state = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd init_cov   = Eigen::MatrixXd::Identity(_altitude_n_, _altitude_n_);

    // Initialize all altitude estimators
    for (auto &estimator : m_altitude_estimators) {
      estimator.second->setState(0, zero_state);
      estimator.second->setState(1, zero_state);
      estimator.second->setState(2, zero_state);
      estimator.second->setCovariance(init_cov);
    }
    is_altitude_estimator_initialized = true;
  }

  //}

  /* state estimators update //{ */

  if (!got_pixhawk_odom_offset) {
    return;
  }

  /* mavros tilts in inertial frame //{ */

  geometry_msgs::Quaternion orient;

  {
    std::scoped_lock lock(mutex_odom_pixhawk_shifted);

    orient                    = odom_pixhawk_shifted.pose.pose.orientation;
    orientation_mavros.header = odom_pixhawk_shifted.header;
  }

  [[maybe_unused]] double hdg = getCurrentHeading();

  // Rotate the tilt into the current estimation frame
  auto [rot_x, rot_y, rot_z] = mrs_lib::AttitudeConverter(orient);

  // publish orientation for debugging
  orientation_mavros.vector.x = rot_x;
  orientation_mavros.vector.y = rot_y;
  orientation_mavros.vector.z = rot_z;
  try {
    pub_orientation_mavros_.publish(orientation_mavros);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_orientation_mavros_.getTopic().c_str());
  }

  //}

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros odom. Waiting for other sensors.");
    return;
  }

  /* //{ fuse mavros velocity */

  if (_gps_available_) {

    double vel_mavros_x, vel_mavros_y;
    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      vel_mavros_x = (odom_pixhawk_shifted.pose.pose.position.x - odom_pixhawk_previous_shifted.pose.pose.position.x) / dt;
      vel_mavros_y = (odom_pixhawk_shifted.pose.pose.position.y - odom_pixhawk_previous_shifted.pose.pose.position.y) / dt;
    }

    // Apply correction step to all state estimators
    if (fabs(vel_mavros_x) < 100 && fabs(vel_mavros_y) < 100) {
      stateEstimatorsCorrection(vel_mavros_x, vel_mavros_y, "vel_mavros");
      ROS_WARN_ONCE("[Odometry]: Fusing mavros velocity");
    }

    // Set innoation variable if ccurnet estimator is GPS
    if (current_estimator->getName() == "GPS") {
      Vec2 vel_vec, innovation;
      current_estimator->getState(1, vel_vec);

      innovation(0) = vel_mavros_x - vel_vec(0);
      innovation(1) = vel_mavros_y - vel_vec(1);
      {
        std::scoped_lock lock(mutex_odom_main_inno);
        odom_main_inno.twist.twist.linear.x = innovation(0);
        odom_main_inno.twist.twist.linear.y = innovation(1);
      }
    }

    Eigen::VectorXd rtk_input(2);
    rtk_input << vel_mavros_x, vel_mavros_y;

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
        ROS_ERROR("[Odometry]: RTK LKF prediction step failed: %s", e.what());
      }
    }
  }

  //}

  /* //{ fuse mavros position */

  if (_gps_available_) {

    if (!got_odom_pixhawk || !got_pixhawk_utm) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Mavros position. Global position not averaged.");
      return;
    }

    double             pos_mavros_x, pos_mavros_y;
    nav_msgs::Odometry odom_mavros_out;
    {
      std::scoped_lock lock(mutex_odom_pixhawk_shifted);

      odom_mavros_out = odom_pixhawk_shifted;
      pos_mavros_x    = odom_pixhawk_shifted.pose.pose.position.x;
      pos_mavros_y    = odom_pixhawk_shifted.pose.pose.position.y;
    }

    try {
      pub_odom_mavros_.publish(odom_mavros_out);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_odom_mavros_.getTopic().c_str());
    }

    // Saturate correction
    for (auto &estimator : m_state_estimators) {
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
        if (current_estimator->getName() == "GPS") {
          odom_main_inno.pose.pose.position.x = innovation(0);
          odom_main_inno.pose.pose.position.y = innovation(1);
          odom_main_inno.pose.pose.position.z = 0;
        }
      }
    }

    if (finished_state_update_) {
      ROS_INFO("[Odometry]: finished state update");
      for (auto &estimator : m_state_estimators) {
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

    ROS_WARN_ONCE("[Odometry]: Fusing mavros position");
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

  pixhawk_imu_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_pixhawk_imu);

    if (got_pixhawk_imu) {

      pixhawk_imu_previous = pixhawk_imu;
      pixhawk_imu          = *msg;

    } else {

      pixhawk_imu_previous = *msg;
      pixhawk_imu          = *msg;
      got_pixhawk_imu      = true;

      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(pixhawk_imu.header.stamp.toSec(), pixhawk_imu_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk IMU timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_fcu_untilted_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not publishing IMU. Waiting for fcu_untilted_tf");
    return;
  }


  // transform imu accelerations to untilted frame
  geometry_msgs::Vector3Stamped acc_untilted;
  acc_untilted.vector          = pixhawk_imu.linear_acceleration;
  acc_untilted.header          = pixhawk_imu.header;
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

    ROS_WARN_ONCE("[Odometry]: Fusing untilted accelerations");

  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  // transform imu angular rates to untilted frame
  geometry_msgs::Vector3Stamped ang_vel_untilted;
  ang_vel_untilted.vector          = pixhawk_imu.angular_velocity;
  ang_vel_untilted.header          = pixhawk_imu.header;
  ang_vel_untilted.header.frame_id = fcu_frame_id_;
  auto response_ang_vel            = transformer_.transformSingle(fcu_untilted_frame_id_, ang_vel_untilted);

  if (response_ang_vel) {
    ang_vel_untilted = response_ang_vel.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  // transform imu attitude to untilted frame
  geometry_msgs::QuaternionStamped attitude_untilted;
  attitude_untilted.quaternion      = pixhawk_imu.orientation;
  attitude_untilted.header          = pixhawk_imu.header;
  attitude_untilted.header.frame_id = fcu_frame_id_;
  auto response_attitude            = transformer_.transformSingle(fcu_untilted_frame_id_, attitude_untilted);
  if (response_attitude) {
    attitude_untilted = response_attitude.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Transform from %s to %s failed", pixhawk_imu.header.frame_id.c_str(), fcu_untilted_frame_id_.c_str());
  }

  sensor_msgs::Imu untilted_imu    = pixhawk_imu;
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

  double yaw_rate;
  {
    std::scoped_lock lock(mutex_pixhawk_imu);
    yaw_rate = pixhawk_imu.angular_velocity.z;
  }

  if (std::isfinite(yaw_rate)) {
    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(yaw_rate, "rate_gyro");

    ROS_WARN_ONCE("[Odometry]: Fusing gyro yaw rate from PixHawk IMU");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in PixHawk IMU variable \"yaw_rate\", not fusing!!!");
  }

  //////////////////// Fuse Linear Z Acceleration ////////////////////

  geometry_msgs::Quaternion q_body;
  {
    std::scoped_lock lock(mutex_odom_pixhawk);
    q_body = odom_pixhawk.pose.pose.orientation;
  }
}

//}

/* //{ callbackPixhawkCompassHdg() */

void Odometry::callbackPixhawkCompassHdg(const std_msgs::Float64ConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPixhawkCompassHdg");

  if (!init_hdg_avg_done) {
    ROS_INFO_ONCE("[Odometry]: Averaging initial compass heading.");
    double yaw;
    {
      std::scoped_lock lock(mutex_compass_hdg);
      yaw = compass_hdg.data;
    }

    yaw = yaw / 180 * M_PI;

    if (got_compass_hdg) {
      yaw = mrs_lib::unwrapAngle(yaw, yaw_previous);
    }

    init_hdg_avg += M_PI / 2 - yaw;
    if (++init_hdg_avg_samples > 100) {
      init_hdg_avg /= 100;
      init_hdg_avg_done = true;
      ROS_INFO("[Odometry]: Initial compass heading averaged to %f", init_hdg_avg);
    }
    return;
  }


  {
    std::scoped_lock lock(mutex_compass_hdg);

    if (got_compass_hdg) {

      compass_hdg_previous = compass_hdg;
      compass_hdg          = *msg;

    } else {

      compass_hdg_previous    = *msg;
      compass_hdg             = *msg;
      yaw_previous            = msg->data / 180 * M_PI;
      got_compass_hdg         = true;
      compass_hdg_last_update = ros::Time::now();

      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  // Compass heading msg does not have timestamp - check at least time of msg arrival
  if (!isTimestampOK(ros::Time::now().toSec(), compass_hdg_last_update.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Pixhawk compass heading time between msgs not OK, not fusing correction.");
    compass_hdg_last_update = ros::Time::now();
    return;
  }
  compass_hdg_last_update = ros::Time::now();

  //////////////////// Fuse Heading Kalman ////////////////////

  double yaw;
  {
    std::scoped_lock lock(mutex_compass_hdg);
    yaw = compass_hdg.data;
  }

  yaw          = yaw / 180 * M_PI;
  yaw          = mrs_lib::unwrapAngle(yaw, yaw_previous);
  yaw_previous = yaw;
  yaw          = M_PI / 2 - yaw;

  if (!compass_yaw_filter->isValid(yaw) && compass_yaw_filter->isFilled()) {
    compass_inconsistent_samples++;
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Compass yaw inconsistent: %f. Not fusing.", yaw);

    if (toUppercase(current_hdg_estimator_name) == "COMPASS" && _gyro_fallback_ && compass_inconsistent_samples > 20) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Compass inconsistent. Swtiching to GYRO heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = mrs_msgs::HeadingType::GYRO;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      compass_inconsistent_samples = 0;
    }
    return;
  }

  if (std::isfinite(yaw)) {

    compass_inconsistent_samples = 0;

    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(yaw, "yaw_compass");

    yaw = mrs_lib::wrapAngle(yaw);

    mrs_msgs::Float64Stamped compass_yaw_out;
    compass_yaw_out.header.stamp    = ros::Time::now();
    compass_yaw_out.header.frame_id = local_origin_frame_id_;
    compass_yaw_out.value           = yaw;
    pub_compass_yaw_.publish(compass_yaw_out);

    ROS_WARN_ONCE("[Odometry]: Fusing yaw from PixHawk compass");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in PixHawk compass variable \"yaw\", not fusing!!!");
  }
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

  optflow_twist_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_optflow);

    if (got_optflow) {

      optflow_twist_previous = optflow_twist;
      optflow_twist          = *msg;

    } else {

      optflow_twist_previous = *msg;
      optflow_twist          = *msg;

      got_optflow      = true;
      optflow_reliable = true;

      return;
    }
  }

  if (!got_range) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optic flow. No range msgs.");
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(optflow_twist.header.stamp.toSec(), optflow_twist_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Optflow twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
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

  for (auto &estimator : m_state_estimators) {
    if (estimator.first == "OPTFLOW" || estimator.first == "BRICKFLOW") {

      // Get initial Q
      if (!got_init_optflow_Q) {
        std::string                          measurement_name  = "vel_optflow";
        std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id.find(measurement_name);
        if (it_measurement_id == map_measurement_name_id.end()) {
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
    double twist_q_x = optflow_twist.twist.covariance[0];
    double twist_q_y = optflow_twist.twist.covariance[7];

    if (std::isfinite(twist_q_x)) {

      // Scale covariance
      twist_q_x *= _dynamic_optflow_cov_scale_;
      twist_q_y *= _dynamic_optflow_cov_scale_;

      double twist_q = std::max(twist_q_x, twist_q_y);

      std::string                          measurement_name  = "vel_optflow";
      std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id.find(measurement_name);
      if (it_measurement_id == map_measurement_name_id.end()) {
        ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
        return;
      }

      for (auto &estimator : m_state_estimators) {
        if (estimator.first == "OPTFLOW" || estimator.first == "OPTFLOWGPS" || estimator.first == "BRICKFLOW") {
          estimator.second->setR(twist_q, it_measurement_id->second);
          ROS_INFO_THROTTLE(5.0, "[Odometry]: estimator: %s setting Q_optflow_twist to: %f", estimator.first.c_str(), twist_q);
        }
      }
    } else {
      twist_q_x = twist_q_x_prev;
      twist_q_y = twist_q_y_prev;
    }
    twist_q_x_prev = twist_q_x;
    twist_q_y_prev = twist_q_y;
  }

  double optflow_vel_x = optflow_twist.twist.twist.linear.x;
  double optflow_vel_y = optflow_twist.twist.twist.linear.y;

  if (_optflow_median_filter_) {
    if (!optflow_filter_x->isValid(optflow_vel_x)) {

      double median = optflow_filter_x->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow x velocity filtered by median filter. %f -> %f", optflow_vel_x, median);
      optflow_vel_x = median;
    }

    if (!optflow_filter_y->isValid(optflow_vel_y)) {
      double median = optflow_filter_y->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow y velocity filtered by median filter. %f -> %f", optflow_vel_y, median);
      optflow_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped optflow_filtered = optflow_twist;
  optflow_filtered.twist.twist.linear.x                      = optflow_vel_x;
  optflow_filtered.twist.twist.linear.y                      = optflow_vel_y;

  try {
    pub_debug_optflow_filter.publish(optflow_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_optflow_filter.getTopic().c_str());
  }

  // Set innoation variable if ccurnet estimator is OPTFLOW
  if (current_estimator->getName() == "OPTFLOW") {
    Vec2 vel_vec, innovation;
    current_estimator->getState(1, vel_vec);

    innovation(0) = optflow_vel_x - vel_vec(0);
    innovation(1) = optflow_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = 0;
      odom_main_inno.pose.pose.position.y = 0;
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(optflow_vel_x, optflow_vel_y, "vel_optflow");

  ROS_WARN_ONCE("[Odometry]: Fusing optflow velocity");

  double yaw_rate;
  {
    std::scoped_lock lock(mutex_optflow);
    yaw_rate = optflow_twist.twist.twist.angular.z;
  }

  if (!optflow_yaw_rate_filter->isValid(yaw_rate)) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: Yaw rate from optic flow is inconsistent. Not fusing.");
    return;
  }

  if (!optflow_yaw_rate_filter->isValid(yaw_rate) && optflow_yaw_rate_filter->isFilled()) {
    optflow_inconsistent_samples++;
    ROS_WARN("[Odometry]: Optflow yaw rate inconsistent: %f. Not fusing.", yaw_rate);

    if (_gyro_fallback_ && optflow_inconsistent_samples > 20) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optflow yaw rate inconsistent. Swtiching to GYRO heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = mrs_msgs::HeadingType::GYRO;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      --optflow_inconsistent_samples;
      optflow_inconsistent_samples = std::max(0, optflow_inconsistent_samples);
    }
    return;
  }


  if (std::isfinite(yaw_rate)) {

    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(yaw_rate, "rate_optflow");

    ROS_WARN_ONCE("[Odometry]: Fusing optflow yaw rate");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in optflow variable \"yaw_rate\", not fusing!!!");
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

  optflow_twist_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_optflow);

    if (got_optflow) {

      optflow_twist_previous = optflow_twist;
      optflow_twist          = *msg;

    } else {

      optflow_twist_previous = *msg;
      optflow_twist          = *msg;

      got_optflow      = true;
      optflow_reliable = true;

      return;
    }
  }

  if (!got_range) {
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(optflow_twist.header.stamp.toSec(), optflow_twist_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Optflow twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow velocity. Waiting for other sensors.");
    return;
  }

  fusing_optflow_low_ = true;

  if (toUppercase(current_hdg_estimator_name) == "OPTFLOW") {
    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = mrs_msgs::HeadingType::GYRO;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    changeCurrentHeadingEstimator(desired_estimator);
  }

  static int    measurement_id     = 0;
  static bool   got_init_optflow_Q = false;
  static double init_Q             = 0.0;

  for (auto &estimator : m_state_estimators) {
    if (estimator.first == "OPTFLOW" || estimator.first == "BRICKFLOW") {

      // Get initial Q
      if (!got_init_optflow_Q) {
        std::string                          measurement_name  = "vel_optflow";
        std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id.find(measurement_name);
        if (it_measurement_id == map_measurement_name_id.end()) {
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
    double twist_q_x = optflow_twist.twist.covariance[0];
    double twist_q_y = optflow_twist.twist.covariance[7];

    if (std::isfinite(twist_q_x)) {

      // Scale covariance
      twist_q_x *= _dynamic_optflow_cov_scale_;
      twist_q_y *= _dynamic_optflow_cov_scale_;

      double twist_q = std::max(twist_q_x, twist_q_y);

      std::string                          measurement_name  = "vel_optflow";
      std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id.find(measurement_name);
      if (it_measurement_id == map_measurement_name_id.end()) {
        ROS_ERROR("[Odometry]: Tried to set covariance of measurement with invalid name: \'%s\'.", measurement_name.c_str());
        return;
      }

      for (auto &estimator : m_state_estimators) {
        if (estimator.first == "OPTFLOW" || estimator.first == "OPTFLOWGPS" || estimator.first == "BRICKFLOW") {
          estimator.second->setR(twist_q, it_measurement_id->second);
          ROS_INFO_THROTTLE(5.0, "[Odometry]: estimator: %s setting Q_optflow_twist to: %f", estimator.first.c_str(), twist_q);
        }
      }
    } else {
      twist_q_x = twist_q_x_prev;
      twist_q_y = twist_q_y_prev;
    }
    twist_q_x_prev = twist_q_x;
    twist_q_y_prev = twist_q_y;
  }

  double optflow_vel_x = optflow_twist.twist.twist.linear.x;
  double optflow_vel_y = optflow_twist.twist.twist.linear.y;

  if (_optflow_median_filter_) {
    if (!optflow_filter_x->isValid(optflow_vel_x)) {

      double median = optflow_filter_x->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow x velocity filtered by median filter. %f -> %f", optflow_vel_x, median);
      optflow_vel_x = median;
    }

    if (!optflow_filter_y->isValid(optflow_vel_y)) {
      double median = optflow_filter_y->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Optic flow y velocity filtered by median filter. %f -> %f", optflow_vel_y, median);
      optflow_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped optflow_filtered = optflow_twist;
  optflow_filtered.twist.twist.linear.x                      = optflow_vel_x;
  optflow_filtered.twist.twist.linear.y                      = optflow_vel_y;

  try {
    pub_debug_optflow_filter.publish(optflow_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_optflow_filter.getTopic().c_str());
  }
  // Set innoation variable if ccurnet estimator is OPTFLOW
  if (current_estimator->getName() == "OPTFLOW") {
    Vec2 vel_vec, innovation;
    current_estimator->getState(1, vel_vec);

    innovation(0) = optflow_vel_x - vel_vec(0);
    innovation(1) = optflow_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = 0;
      odom_main_inno.pose.pose.position.y = 0;
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(optflow_vel_x, optflow_vel_y, "vel_optflow");

  ROS_WARN_ONCE("[Odometry]: Fusing optflow velocity from OPTFLOW low");
}

//}

/* //{ callbackICPTwist() */

void Odometry::callbackICPTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg) {

  // Note: ICP twist is coming in the UAV body frame (uav/fcu)
  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackICPTwist");

  icp_twist_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_icp_twist);

    if (got_icp_twist) {

      icp_twist_previous = icp_twist;
      icp_twist          = *msg;

    } else {

      icp_twist_previous = *msg;
      icp_twist          = *msg;

      got_icp_twist = true;
      icp_reliable  = true;

      return;
    }
  }

  if (!got_range) {
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(icp_twist.header.stamp.toSec(), icp_twist_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: ICP twist timestamp not OK, not fusing correction.");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP velocity. Waiting for other sensors.");
    return;
  }

  double icp_vel_x = icp_twist.twist.twist.linear.x;
  double icp_vel_y = icp_twist.twist.twist.linear.y;

  if (_icp_twist_median_filter_) {
    if (!icp_twist_filter_x->isValid(icp_vel_x)) {

      double median = icp_twist_filter_x->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP x velocity filtered by median filter. %f -> %f", icp_vel_x, median);
      icp_vel_x = median;
    }

    if (!icp_twist_filter_y->isValid(icp_vel_y)) {
      double median = icp_twist_filter_y->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: ICP y velocity filtered by median filter. %f -> %f", icp_vel_y, median);
      icp_vel_y = median;
    }
  }
  geometry_msgs::TwistWithCovarianceStamped icp_twist_filtered = icp_twist;
  icp_twist_filtered.twist.twist.linear.x                      = icp_vel_x;
  icp_twist_filtered.twist.twist.linear.y                      = icp_vel_y;

  try {
    pub_debug_icp_twist_filter.publish(icp_twist_filtered);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_debug_icp_twist_filter.getTopic().c_str());
  }
  // Set innoation variable if ccurnet estimator is ICP
  if (current_estimator->getName() == "ICP") {
    Vec2 vel_vec, innovation;
    current_estimator->getState(1, vel_vec);

    innovation(0) = icp_vel_x - vel_vec(0);
    innovation(1) = icp_vel_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = 0;
      odom_main_inno.pose.pose.position.y = 0;
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(icp_vel_x, icp_vel_y, "vel_icp");

  ROS_WARN_ONCE("[Odometry]: Fusing icp velocity");

  double yaw_rate;
  {
    std::scoped_lock lock(mutex_icp_twist);
    yaw_rate = icp_twist.twist.twist.angular.z;
  }

  if (!icp_yaw_rate_filter->isValid(yaw_rate)) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: Yaw rate from ICP is inconsistent. Not fusing.");
    return;
  }

  if (!icp_yaw_rate_filter->isValid(yaw_rate) && icp_yaw_rate_filter->isFilled()) {
    icp_yaw_rate_inconsistent_samples++;
    ROS_WARN("[Odometry]: icp yaw rate inconsistent: %f. Not fusing.", yaw_rate);

    if (_gyro_fallback_ && icp_yaw_rate_inconsistent_samples > 20) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: icp yaw rate inconsistent. Swtiching to GYRO heading estimator.");
      mrs_msgs::HeadingType desired_estimator;
      desired_estimator.type = mrs_msgs::HeadingType::GYRO;
      desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
      changeCurrentHeadingEstimator(desired_estimator);
      --icp_yaw_rate_inconsistent_samples;
      icp_yaw_rate_inconsistent_samples = std::max(0.0, icp_yaw_rate_inconsistent_samples);
    }
    return;
  }


  if (std::isfinite(yaw_rate)) {

    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(yaw_rate, "rate_icp");

    ROS_WARN_ONCE("[Odometry]: Fusing icp yaw rate");

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in ICP variable \"yaw_rate\", not fusing!!!");
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
    std::scoped_lock lock(mutex_rtk);

    if (!isUavFlying()) {
      if (++got_rtk_counter < 10) {
        rtk_local_origin_z_ += rtk_utm.pose.pose.position.z;
        ROS_INFO("[Odometry]: RTK ASL altitude sample #%d: %f", got_rtk_counter, rtk_utm.pose.pose.position.z);
        return;

      } else {

        if (!got_rtk_local_origin_z) {
          rtk_local_origin_z_ /= 10;
          rtk_local_origin_z_ -= _fcu_height_;
          got_rtk_local_origin_z = true;
          ROS_INFO("[Odometry]: RTK ASL altitude avg: %f", rtk_local_origin_z_);
        }
      }

    } else {
      if (!got_rtk_local_origin_z) {
        rtk_local_origin_z_ = 0.0;
      }
    }

    got_rtk         = true;
    rtk_last_update = ros::Time::now();

    rtk_local_previous = rtk_local;
    rtk_local          = rtk_utm;
  }

  if (!got_odom_pixhawk || !got_rtk) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not received RTK yet.");
    return;
  }

  if (!isTimestampOK(rtk_local.header.stamp.toSec(), rtk_local_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: RTK local timestamp not OK, not fusing correction.");
    return;
  }

  // check whether we have rtk fix
  got_rtk_fix = (rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FLOAT || rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FIX) ? true : false;

  if (_rtk_fuse_sps_) {
    if (rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::NO_FIX || rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::UNKNOWN) {
      ROS_WARN_THROTTLE(1.0, "RTK fix type: NO_FIX. Not fusing RTK.");
      return;
    }
  } else {
    if (!got_rtk_fix) {
      ROS_WARN_THROTTLE(1.0, "RTK not fusing SPS.");
      return;
    }
  }

  // | ------------- offset the rtk to local_origin ------------- |
  rtk_local.pose.pose.position.x -= _utm_origin_x_;
  rtk_local.pose.pose.position.y -= _utm_origin_y_;
  rtk_local.pose.pose.position.z -= rtk_local_origin_z_;

  rtk_local.header.frame_id = _uav_name_ + "/rtk_origin";

  // | ------------------ publish the rtk local ----------------- |
  mrs_msgs::RtkGps rtk_local_out = rtk_local;

  try {
    pub_rtk_local_.publish(rtk_local_out);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local_.getTopic().c_str());
  }
  // | ------------- publish the rtk local odometry ------------- |
  {
    std::scoped_lock lock(mutex_rtk_local_odom);

    rtk_local_odom.header = rtk_local.header;
    rtk_local_odom.pose   = rtk_local.pose;
    rtk_local_odom.twist  = rtk_local.twist;

    try {
      pub_rtk_local_odom_.publish(rtk_local_odom);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local_odom_.getTopic().c_str());
    }
  }

  if (_rtk_available_) {
    if (!std::isfinite(rtk_local.pose.pose.position.x) || !std::isfinite(rtk_local.pose.pose.position.y)) {

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local.pose.pose.position.x\" or \"rtk_local.pose.pose.position.y\" (rtk)!!!");

      return;
    }

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing RTK position. Waiting for other sensors.");
      return;
    }

    double x_rtk, y_rtk;
    {
      std::scoped_lock lock(mutex_rtk);

      x_rtk = rtk_local.pose.pose.position.x;
      y_rtk = rtk_local.pose.pose.position.y;
    }

    if (!std::isfinite(x_rtk)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"x_rtk\" (callbackRtk)!!!");
      return;
    }

    if (!std::isfinite(y_rtk)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"y_rtk\" (callbackRtk)!!!");
      return;
    }

    /* fuse rtk position //{ */
    // Saturate correction
    double x_est;
    double y_est;
    {
      std::scoped_lock lock(mutex_rtk_est_);

      x_est = sc_lat_rtk_.x(0);
      y_est = sc_lat_rtk_.x(1);
    }

    // X position
    double x_correction = x_rtk - x_est;
    if (!std::isfinite(x_rtk)) {
      x_rtk = 0;
      ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"x_rtk\", setting it to 0 and returning!!!");
      return;
    }
    if (x_correction > _max_rtk_pos_correction_) {
      x_correction = _max_rtk_pos_correction_;
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating RTK X pos correction %f -> %f", x_correction, _max_rtk_pos_correction_);
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
    /* } */

    // Do RTK estimator correction
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

    ROS_WARN_ONCE("[Odometry]: Fusing RTK position");
  }

  //}
}

//}

/* //{ callbackVioOdometry() */

void Odometry::callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackVioOdometry");

  if (got_vio) {

    {
      std::scoped_lock lock(mutex_odom_vio);

      odom_vio_previous = odom_vio;
      odom_vio          = *msg;
    }

  } else {

    {
      std::scoped_lock lock(mutex_odom_vio);

      odom_vio_previous = *msg;
      odom_vio          = *msg;
      vio_yaw_previous  = mrs_lib::AttitudeConverter(odom_vio.pose.pose.orientation).getYaw();
    }

    got_vio              = true;
    odom_vio_last_update = ros::Time::now();
    return;
  }

  odom_vio_last_update = ros::Time::now();

  if (!got_range) {

    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(odom_vio.header.stamp.toSec(), odom_vio_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: VIO timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse vio yaw //{ */

  double yaw_vio;
  {
    std::scoped_lock lock(mutex_odom_vio);
    // TODO yaw != heading
    yaw_vio = mrs_lib::AttitudeConverter(odom_vio.pose.pose.orientation).getYaw();
  }

  yaw_vio          = mrs_lib::unwrapAngle(yaw_vio, vio_yaw_previous);
  vio_yaw_previous = yaw_vio;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_vio, "yaw_vio");

  yaw_vio = mrs_lib::wrapAngle(yaw_vio);

  mrs_msgs::Float64Stamped vio_yaw_out;
  vio_yaw_out.header.stamp    = ros::Time::now();
  vio_yaw_out.header.frame_id = local_origin_frame_id_;
  vio_yaw_out.value           = yaw_vio;
  pub_vio_yaw_.publish(vio_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from VIO");

  //}

  /* fuse vio height //{ */

  //////////////////// Filter out vio height measurement ////////////////////

  bool   vio_altitude_ok = true;
  double measurement     = odom_vio.pose.pose.position.z;
  if (isUavFlying()) {
    if (!vioHeightFilter->isValid(measurement)) {
      /* double filtered = vioHeightFilter->getMedian(); */
      ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO height easurement %f declined by median filter.", measurement);
      vio_altitude_ok = false;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////

  double min_height = -100.0;
  double max_height = 100.0;
  if (measurement < min_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO height measurement %f < %f. Not fusing.", measurement, min_height);
    vio_altitude_ok = false;
  }

  if (measurement > max_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO measurement %f > %f. Not fusing.", measurement, max_height);
    vio_altitude_ok = false;
  }

  // Fuse vio measurement for each altitude estimator
  for (auto &estimator : m_altitude_estimators) {
    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    if (!estimator.second->getStates(current_altitude)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      vio_altitude_ok = false;
    }

    if (vio_altitude_ok) {
      {
        std::scoped_lock lock(mutex_altitude_estimator);
        altitudeEstimatorCorrection(measurement, "height_vio", estimator.second);
        if (fabs(measurement) > 100) {
          ROS_WARN("[Odometry]: VIO height correction: %f", measurement);
        }
      }
    }
  }

  ROS_WARN_ONCE("[Odometry]: Fusing height from VIO pose");

  //}

  //////////////////// Fuse Lateral Kalman ////////////////////

  /* //{ fuse vio velocity */

  double vel_vio_x, vel_vio_y;

  {
    std::scoped_lock lock(mutex_odom_vio);

    vel_vio_x = odom_vio.twist.twist.linear.x;
    vel_vio_y = odom_vio.twist.twist.linear.y;
  }

  // Set innoation variable if ccurnet estimator is VIO
  if (current_estimator->getName() == "VIO") {
    Vec2 vel_vec, innovation;
    current_estimator->getState(1, vel_vec);

    innovation(0) = vel_vio_x - vel_vec(0);
    innovation(1) = vel_vio_y - vel_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  if (vio_reliable && (vel_vio_x > 10 || vel_vio_y > 10)) {
    ROS_WARN("[Odometry]: Estimated VIO velocity > 10. VIO is not reliable.");
    vio_reliable = false;
    return;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vel_vio_x, vel_vio_y, "vel_vio");

  ROS_WARN_ONCE("[Odometry]: Fusing VIO velocity");
  //}

  /* //{ fuse vio position */

  double vio_pos_x, vio_pos_y;

  {
    std::scoped_lock lock(mutex_odom_vio);

    // Correct the position by the current heading
    vio_pos_x = odom_vio.pose.pose.position.x;
    vio_pos_y = odom_vio.pose.pose.position.y;
  }

  // Saturate correction
  for (auto &estimator : m_state_estimators) {
    if (estimator.first == "VIO") {

      Vec2 pos_vec, innovation;
      estimator.second->getState(0, pos_vec);

      innovation(0) = vio_pos_x - pos_vec(0);
      innovation(1) = vio_pos_y - pos_vec(1);

      // X position
      if (!std::isfinite(vio_pos_x)) {
        vio_pos_x = 0;
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vio_pos_x\", setting it to 0 and returning!!!");
        return;
      } else if (innovation(0) > _max_vio_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VIO X pos correction %f -> %f", innovation(0), _max_vio_pos_correction_);
        vio_pos_x = pos_vec(0) + _max_vio_pos_correction_;
      } else if (innovation(0) < -_max_vio_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VIO X pos correction %f -> %f", innovation(0), -_max_vio_pos_correction_);
        vio_pos_x = pos_vec(0) - _max_vio_pos_correction_;
      }

      // Y position
      if (!std::isfinite(vio_pos_y)) {
        vio_pos_y = 0;
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vio_pos_y\", setting it to 0 and returning!!!");
        return;
      } else if (innovation(1) > _max_vio_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VIO Y pos correction %f -> %f", innovation(1), _max_vio_pos_correction_);
        vio_pos_y = pos_vec(1) + _max_vio_pos_correction_;
      } else if (innovation(1) < -_max_vio_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VIO Y pos correction %f -> %f", innovation(1), -_max_vio_pos_correction_);
        vio_pos_y = pos_vec(1) - _max_vio_pos_correction_;
      }

      // Set innoation variable if ccurnet estimator is VIO
      if (current_estimator->getName() == "VIO") {
        {
          std::scoped_lock lock(mutex_odom_main_inno);
          odom_main_inno.pose.pose.position.x = innovation(0);
          odom_main_inno.pose.pose.position.y = innovation(1);
          odom_main_inno.pose.pose.position.z = 0;
        }
      }
    }
  }

  if (vio_reliable && (std::fabs(odom_vio.pose.pose.position.x - odom_vio_previous.pose.pose.position.x) > 10 ||
                       std::fabs(odom_vio.pose.pose.position.y - odom_vio_previous.pose.pose.position.y) > 10)) {
    ROS_WARN("[Odometry]: Estimated difference between VIO positions > 10. VIO is not reliable.");
    vio_reliable = false;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vio_pos_x, vio_pos_y, "pos_vio");

  ROS_WARN_ONCE("[Odometry]: Fusing VIO position");
  //}
}


//}

/* //{ callbackVslamPose() */

void Odometry::callbackVslamPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackVslamPose");

  if (got_vslam) {

    {
      std::scoped_lock lock(mutex_pose_vslam);

      pose_vslam_previous = pose_vslam;
      pose_vslam          = *msg;
    }

  } else {

    {
      std::scoped_lock lock(mutex_pose_vslam);

      pose_vslam_previous = *msg;
      pose_vslam          = *msg;
      vslam_yaw_previous  = mrs_lib::AttitudeConverter(pose_vslam.pose.pose.orientation).getYaw();
    }

    got_vslam              = true;
    pose_vslam_last_update = ros::Time::now();
    return;
  }

  pose_vslam_last_update = ros::Time::now();

  if (!got_range) {

    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(pose_vslam.header.stamp.toSec(), pose_vslam_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: VSLAM timestamp not OK, not fusing correction.");
    return;
  }

  double yaw_vslam;
  {
    std::scoped_lock lock(mutex_pose_vslam);
    yaw_vslam = mrs_lib::AttitudeConverter(pose_vslam.pose.pose.orientation).getYaw();
  }

  yaw_vslam          = mrs_lib::unwrapAngle(yaw_vslam, vslam_yaw_previous);
  vslam_yaw_previous = yaw_vslam;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_vslam, "yaw_vslam");

  yaw_vslam = mrs_lib::wrapAngle(yaw_vslam);

  mrs_msgs::Float64Stamped vslam_yaw_out;
  vslam_yaw_out.header.stamp    = ros::Time::now();
  vslam_yaw_out.header.frame_id = local_origin_frame_id_;
  vslam_yaw_out.value           = yaw_vslam;
  pub_vslam_yaw_.publish(vslam_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from VSLAM");

  //////////////////// Fuse Lateral Kalman ////////////////////

  /* //{ fuse vslam position */

  double vslam_pos_x, vslam_pos_y;

  {
    std::scoped_lock lock(mutex_pose_vslam);

    // Correct the position by the current heading
    vslam_pos_x = pose_vslam.pose.pose.position.x;
    vslam_pos_y = pose_vslam.pose.pose.position.y;
  }

  // Saturate correction
  for (auto &estimator : m_state_estimators) {
    if (estimator.first == "VSLAM") {
      Vec2 pos_vec, innovation;
      current_estimator->getState(0, pos_vec);

      innovation(0) = vslam_pos_x - pos_vec(0);
      innovation(1) = vslam_pos_y - pos_vec(1);

      // X position
      if (!std::isfinite(vslam_pos_x)) {
        vslam_pos_x = 0;
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vslam_pos_x\", setting it to 0 and returning!!!");
        return;
      } else if (innovation(0) > _max_vslam_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VSLAM X pos correction %f -> %f", innovation(0), _max_vslam_pos_correction_);
        vslam_pos_x = pos_vec(0) + _max_vslam_pos_correction_;
      } else if (innovation(0) < -_max_vslam_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VSLAM X pos correction %f -> %f", innovation(0), -_max_vslam_pos_correction_);
        vslam_pos_x = pos_vec(0) - _max_vslam_pos_correction_;
      }

      // Y position
      if (!std::isfinite(vslam_pos_y)) {
        vslam_pos_y = 0;
        ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"vslam_pos_y\", setting it to 0 and returning!!!");
        return;
      } else if (innovation(1) > _max_vslam_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VSLAM Y pos correction %f -> %f", innovation(1), _max_vslam_pos_correction_);
        vslam_pos_y = pos_vec(1) + _max_vslam_pos_correction_;
      } else if (innovation(1) < -_max_vslam_pos_correction_) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Saturating VSLAM Y pos correction %f -> %f", innovation(1), -_max_vslam_pos_correction_);
        vslam_pos_y = pos_vec(1) - _max_vslam_pos_correction_;
      }

      // Set innoation variable if ccurnet estimator is VSLAM
      if (current_estimator->getName() == "VSLAM") {
        {
          std::scoped_lock lock(mutex_odom_main_inno);
          odom_main_inno.pose.pose.position.x = innovation(0);
          odom_main_inno.pose.pose.position.y = innovation(1);
          odom_main_inno.pose.pose.position.z = 0;
          odom_main_inno.twist.twist.linear.x = 0;
          odom_main_inno.twist.twist.linear.y = 0;
        }
      }
    }
  }

  if (vslam_reliable && (std::fabs(pose_vslam.pose.pose.position.x - pose_vslam_previous.pose.pose.position.x) > 10 ||
                         std::fabs(pose_vslam.pose.pose.position.y - pose_vslam_previous.pose.pose.position.y) > 10)) {
    ROS_WARN("[Odometry]: Estimated difference between VSLAM positions > 10. VSLAM is not reliable.");
    vslam_reliable = false;
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vslam_pos_x, vslam_pos_y, "pos_vslam");

  ROS_WARN_ONCE("[Odometry]: Fusing VSLAM position");
  //}
}


//}

/* //{ callbackBrickPose() */

void Odometry::callbackBrickPose(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackBrickPose");

  brick_pose_last_update = ros::Time::now();

  if (got_brick_pose) {

    brick_pose_previous = brick_pose;
    brick_pose          = *msg;

  } else {

    brick_pose_previous = *msg;
    brick_pose          = *msg;
    brick_yaw_previous  = mrs_lib::AttitudeConverter(brick_pose.pose.orientation).getYaw();

    got_brick_pose = true;
    return;
  }

  double diff_x = std::pow(brick_pose.pose.position.x - brick_pose_previous.pose.position.x, 2);
  if (diff_x > max_safe_brick_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump x: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_x), std::sqrt(max_safe_brick_jump_sq_));
    if (brick_reliable && toUppercase(current_estimator_name) == "BRICK") {
      c_failed_brick_x_++;
    }
    brick_reliable = false;
    return;
  }
  double diff_y = std::pow(brick_pose.pose.position.y - brick_pose_previous.pose.position.y, 2);
  if (diff_y > max_safe_brick_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump y: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_y), std::sqrt(max_safe_brick_jump_sq_));
    if (brick_reliable && toUppercase(current_estimator_name) == "BRICK") {
      c_failed_brick_y_++;
    }
    brick_reliable = false;
    return;
  }

  double dt = (brick_pose.header.stamp - brick_pose_previous.header.stamp).toSec();
  if (dt < 0.0001) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: received the same brick pose msg. returning");
    return;
  }

  // brick times out after not being received for some time
  if (brick_reliable || brick_semi_reliable) {

    if ((ros::Time::now() - brick_pose.header.stamp).toSec() > _brick_timeout_) {

      ROS_WARN_THROTTLE(1.0, "[Odometry]: brick timed out, not reliable");
      if (brick_reliable && toUppercase(current_estimator_name) == "BRICK") {
        c_failed_brick_timeout_++;
      }
      brick_reliable      = false;
      brick_semi_reliable = false;

      return;
    }
  }

  // brick appears after not seeing it for long time -> becomes semi-reliable
  if (!brick_reliable && !brick_semi_reliable) {

    if ((ros::Time::now() - brick_pose.header.stamp).toSec() < 1.0) {

      brick_semi_reliable         = true;
      brick_semi_reliable_started = ros::Time::now();

      ROS_WARN_THROTTLE(1.0, "[Odometry]: brick becomes semi-reliable.");
    }
  }

  // brick is semi-reliable for some time -> becomes reliable
  if (brick_semi_reliable && (ros::Time::now() - brick_semi_reliable_started).toSec() > 1.0) {

    for (auto &estimator : m_state_estimators) {

      if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
        Vec2 pos_vec;
        pos_vec << brick_pose.pose.position.x, brick_pose.pose.position.y;
        estimator.second->setState(0, pos_vec);
      }
    }
    for (auto &estimator : m_heading_estimators) {
      if (estimator.first == "BRICK" || estimator.first == "BRICKFLOW") {
        Eigen::VectorXd hdg(1);
        init_brick_yaw_ = mrs_lib::AttitudeConverter(brick_pose.pose.orientation).getYaw();
        hdg << init_brick_yaw_;
        estimator.second->setState(0, hdg);
      }
    }

    ROS_WARN("[Odometry]: Brick is now reliable");
    brick_reliable      = true;
    brick_semi_reliable = false;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(brick_pose.header.stamp.toSec(), brick_pose_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: brick pose timestamp not OK, not fusing correction.");
    return;
  }

  /* brick estimator reset //{ */

  if (brick_pose.pose.position.z == -1.0) {
    ROS_INFO("[Odometry]: Detected -1.0 in Z position of brick pose msg. Starting BRICK estimator reset.");

    LatState2D states;
    bool       success = false;

    states(0, 0) = brick_pose.pose.position.x;
    states(1, 0) = 0.0;
    states(2, 0) = 0.0;
    states(3, 0) = 0.0;
    states(4, 0) = 0.0;
    states(5, 0) = 0.0;
    states(0, 1) = brick_pose.pose.position.y;
    states(1, 1) = 0.0;
    states(2, 1) = 0.0;
    states(3, 1) = 0.0;
    states(4, 1) = 0.0;
    states(5, 1) = 0.0;

    for (auto &estimator : m_state_estimators) {
      if (estimator.first == "BRICK") {
        success = estimator.second->reset(states);
      }
    }

    Eigen::MatrixXd hdg_states = Eigen::MatrixXd::Zero(3, 1);
    states(0, 0)               = mrs_lib::AttitudeConverter(brick_pose.pose.orientation).getYaw();

    if (success) {
      for (auto &estimator : m_heading_estimators) {
        if (estimator.first == "BRICK") {
          success &= estimator.second->reset(hdg_states);
        }
      }
    }
    if (success) {
      ROS_INFO("[Odometry]: BRICK estimator reset finished. New brick position: x: %f y: %f, yaw: %f", states(0, 0), states(0, 1), hdg_states(0, 0));
      if (toUppercase(current_estimator_name) == "BRICK") {
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

  double yaw_tmp = mrs_lib::AttitudeConverter(brick_pose.pose.orientation).getYaw();

  double yaw_brick = mrs_lib::unwrapAngle(yaw_tmp, brick_yaw_previous);

  double diff_yaw = std::pow(yaw_brick - brick_yaw_previous, 2);
  if (diff_yaw > max_safe_brick_yaw_jump_sq_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Jump yaw: %f > %f detected in BRICK pose. Not reliable.", std::sqrt(diff_yaw), std::sqrt(max_safe_brick_yaw_jump_sq_));
    if (brick_reliable && toUppercase(current_estimator_name) == "BRICK") {
      c_failed_brick_yaw_++;
    }
    brick_reliable     = false;
    brick_yaw_previous = yaw_brick;
    return;
  }

  brick_yaw_previous = yaw_brick;

  if (std::isfinite(yaw_brick)) {
    // Apply correction step to all heading estimators
    headingEstimatorsCorrection(yaw_brick, "yaw_brick");

    yaw_brick = mrs_lib::wrapAngle(yaw_brick);

    mrs_msgs::Float64Stamped brick_yaw_out;
    brick_yaw_out.header.stamp    = ros::Time::now();
    brick_yaw_out.header.frame_id = local_origin_frame_id_;
    brick_yaw_out.value           = yaw_brick;
    pub_brick_yaw_.publish(brick_yaw_out);
  } else {
    ROS_WARN("[Odometry]: NaN in brick yaw");
  }

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from brick pose");

  //////////////////// Filter out brick height measurement ////////////////////
  // do not fuse brick height measurements when a height jump is detected - most likely the UAV is flying above an obstacle

  double measurement       = brick_pose.pose.position.z;
  bool   fuse_brick_height = true;
  if (isUavFlying()) {
    if (!brickHeightFilter->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Brick height easurement %f declined by median filter.", measurement);
      fuse_brick_height = false;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////

  double min_height = 0.0;
  double max_height = 100.0;
  if (measurement < min_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Brick height measurement %f < %f. Not fusing.", measurement, min_height);
    fuse_brick_height = false;
  }

  if (measurement > max_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Brick height measurement %f > %f. Not fusing.", measurement, max_height);
    fuse_brick_height = false;
  }

  // Fuse brick measurement for each altitude estimator
  if (fuse_brick_height && brick_reliable) {
    for (auto &estimator : m_altitude_estimators) {
      Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
      if (!estimator.second->getStates(current_altitude)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
        return;
      }

      {
        std::scoped_lock lock(mutex_altitude_estimator);
        altitudeEstimatorCorrection(measurement, "height_brick", estimator.second);
        if (fabs(measurement) > 100) {
          ROS_WARN("[Odometry]: Brick height correction: %f", measurement);
        }
      }
    }

    ROS_WARN_ONCE("[Odometry]: Brick height from brick pose");
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing brick pose. Waiting for other sensors.");
    return;
  }

  double pos_brick_x, pos_brick_y;

  {
    std::scoped_lock lock(mutex_brick);

    pos_brick_x = brick_pose.pose.position.x;
    pos_brick_y = brick_pose.pose.position.y;
  }

  for (auto &estimator : m_heading_estimators) {
    if (estimator.first == "BRICK") {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(1);
      if (!estimator.second->getState(0, state)) {
        ROS_WARN_THROTTLE(1.0, "[Odometry]: Heading estimator not initialized.");
        return;
      }
      break;
    }
  }

  // Set innovation variable if current estimator is VIO
  if (current_estimator->getName() == "BRICK") {
    Vec2 pos_vec, innovation;
    current_estimator->getState(0, pos_vec);

    innovation(0) = pos_brick_x - pos_vec(0);
    innovation(1) = pos_brick_y - pos_vec(1);
    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = innovation(0);
      odom_main_inno.pose.pose.position.y = innovation(1);
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = 0;
      odom_main_inno.twist.twist.linear.y = 0;
    }
  }

  // Apply correction step to all state estimators
  if (brick_reliable) {
    stateEstimatorsCorrection(pos_brick_x, pos_brick_y, "pos_brick");
    ROS_INFO_THROTTLE(1.0, "[Odometry]: fusing brick: x: %f, y: %f", pos_brick_x, pos_brick_y);
  }

  ROS_WARN_ONCE("[Odometry]: Fusing brick position");
}
//}

/* //{ callbackLidarOdom() */

void Odometry::callbackLidarOdom(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackLidarOdom");

  lidar_odom_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_lidar);

    if (got_lidar_odom) {

      lidar_odom_previous = lidar_odom;
      lidar_odom          = *msg;

    } else {

      lidar_odom_previous = *msg;
      lidar_odom          = *msg;
      lidar_yaw_previous  = mrs_lib::AttitudeConverter(lidar_odom.pose.pose.orientation).getYaw();

      got_lidar_odom = true;
      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(lidar_odom.header.stamp.toSec(), lidar_odom_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: LIDAR velocity timestamp not OK, not fusing correction.");
    return;
  }

  // fuse yaw
  double yaw_lidar;
  {
    std::scoped_lock lock(mutex_lidar);
    yaw_lidar = mrs_lib::AttitudeConverter(lidar_odom.pose.pose.orientation).getYaw();
  }

  yaw_lidar          = mrs_lib::unwrapAngle(yaw_lidar, lidar_yaw_previous);
  lidar_yaw_previous = yaw_lidar;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_lidar, "yaw_lidar");

  yaw_lidar = mrs_lib::wrapAngle(yaw_lidar);

  mrs_msgs::Float64Stamped lidar_yaw_out;
  lidar_yaw_out.header.stamp    = ros::Time::now();
  lidar_yaw_out.header.frame_id = local_origin_frame_id_;
  lidar_yaw_out.value           = yaw_lidar;
  pub_lidar_yaw_.publish(lidar_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from Lidar SLAM");

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing LIDAR velocity. Waiting for other sensors.");
    return;
  }

  // velocity correction
  double vel_lidar_x, vel_lidar_y;
  {
    std::scoped_lock lock(mutex_lidar);

    vel_lidar_x = lidar_odom.twist.twist.linear.x;
    vel_lidar_y = lidar_odom.twist.twist.linear.y;
  }

  if (_lidar_vel_median_filter_) {

    if (!lidar_vel_filter_x->isValid(vel_lidar_x)) {

      double median = lidar_vel_filter_x->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: LIDAR x velocity filtered by median filter. %f -> %f", vel_lidar_x, median);
      vel_lidar_x = median;
    }

    if (!lidar_vel_filter_y->isValid(vel_lidar_y)) {

      double median = lidar_vel_filter_y->getMedian();
      ROS_WARN_THROTTLE(1.0, "[Odometry]: LIDAR y velocity filtered by median filter. %f -> %f", vel_lidar_y, median);
      vel_lidar_y = median;
    }
  }

  // Set innoation variable if ccurnet estimator is LIDAR
  if (current_estimator->getName() == "LIDAR") {
    Vec2 vel_vec, innovation;
    current_estimator->getState(1, vel_vec);

    innovation(0) = vel_lidar_x - vel_vec(0);
    innovation(1) = vel_lidar_y - vel_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vel_lidar_x, vel_lidar_y, "vel_lidar");

  ROS_WARN_ONCE("[Odometry]: Fusing LIDAR velocity");

  // Current orientation
  Eigen::VectorXd hdg_state(1);

  if (current_hdg_estimator->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk);
    hdg_state(0) = orientation_mavros.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator);

    current_hdg_estimator->getState(0, hdg_state);
  }

  // position correction
  {
    std::scoped_lock lock(mutex_lidar);

    pos_lidar_x = lidar_odom.pose.pose.position.x;
    pos_lidar_y = lidar_odom.pose.pose.position.y;
  }

  // Set innoation variable if ccurnet estimator is LIDAR
  if (current_estimator->getName() == "LIDAR") {
    Vec2 pos_vec, innovation;
    current_estimator->getState(0, pos_vec);

    innovation(0) = pos_lidar_x - pos_vec(0);
    innovation(1) = pos_lidar_y - pos_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.twist.twist.linear.x = innovation(0);
      odom_main_inno.twist.twist.linear.y = innovation(1);
    }
  }

  ROS_WARN_ONCE("[Odometry]: Fusing LIDAR position");
}
//}

/* //{ callbackHectorPose() */

void Odometry::callbackHectorPose(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackHectorPose");

  hector_pose_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_hector);

    if (got_hector_pose) {

      hector_pose_previous = hector_pose;
      hector_pose          = *msg;

    } else {

      hector_pose_previous = *msg;
      hector_pose          = *msg;
      hector_yaw_previous  = mrs_lib::AttitudeConverter(hector_pose.pose.orientation).getYaw();

      got_hector_pose = true;
      return;
    }

    if (hector_reliable) {

      // Detect jump since previous pose
      if (std::pow(hector_pose.pose.position.x - hector_pose_previous.pose.position.x, 2) > 4 ||
          std::pow(hector_pose.pose.position.y - hector_pose_previous.pose.position.y, 2) > 4) {
        ROS_WARN("[Odometry]: Jump detected in Hector Slam pose. Not reliable");

        hector_reliable = false;

        Vec2 pos_vec, vel_vec;
        for (auto &estimator : m_state_estimators) {
          if (estimator.first == "HECTOR") {
            estimator.second->getState(0, pos_vec);
            estimator.second->getState(1, vel_vec);
          }
        }

        hector_reliable = false;
      }

      if (current_estimator->getName() == "HECTOR") {
        Vec2 vel_vec;
        current_estimator->getState(1, vel_vec);
        if (vel_vec(0) > 5 || vel_vec(1) > 5) {
          ROS_WARN("[Odometry]: Hector Slam velocity too large. Not reliable.");

          hector_reliable = false;
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

  if (!isTimestampOK(hector_pose.header.stamp.toSec(), hector_pose_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Hector pose timestamp not OK, not fusing correction.");
    return;
  }

  double yaw_hector;
  {
    std::scoped_lock lock(mutex_hector);
    yaw_hector = mrs_lib::AttitudeConverter(hector_pose.pose.orientation).getYaw();
  }

  yaw_hector = mrs_lib::unwrapAngle(yaw_hector, hector_yaw_previous);
  yaw_hector += hector_offset_hdg_;
  hector_yaw_previous = yaw_hector;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_hector, "yaw_hector");

  yaw_hector = mrs_lib::wrapAngle(yaw_hector);

  mrs_msgs::Float64Stamped hector_yaw_out;
  hector_yaw_out.header.stamp    = ros::Time::now();
  hector_yaw_out.header.frame_id = local_origin_frame_id_;
  hector_yaw_out.value           = yaw_hector;
  pub_hector_yaw_.publish(hector_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from Hector SLAM");

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Hector pose. Waiting for other sensors.");
    return;
  }

  {
    std::scoped_lock lock(mutex_hector);

    pos_hector_x = hector_pose.pose.position.x + hector_offset_(0);
    pos_hector_y = hector_pose.pose.position.y + hector_offset_(1);
  }

  // Current orientation
  Eigen::VectorXd hdg_state(1);

  if (current_hdg_estimator->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk);
    hdg_state(0) = orientation_mavros.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator);

    current_hdg_estimator->getState(0, hdg_state);
  }


  // Set innoation variable if ccurnet estimator is HECTOR
  if (current_estimator->getName() == "HECTOR") {
    Vec2 pos_vec, innovation;
    current_estimator->getState(0, pos_vec);

    {
      std::scoped_lock lock(mutex_hector);
      innovation(0) = pos_hector_x - pos_vec(0);
      innovation(1) = pos_hector_y - pos_vec(1);
    }

    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = innovation(0);
      odom_main_inno.pose.pose.position.y = innovation(1);
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = 0;
      odom_main_inno.twist.twist.linear.y = 0;
    }
  }

  ROS_WARN_ONCE("[Odometry]: Fusing Hector position");
}
//}

/* //{ callbackTowerPose() */

void Odometry::callbackTowerPose(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackTowerPose");

  tower_pose_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_tower);

    if (got_tower_pose) {

      tower_pose_previous = tower_pose;
      tower_pose          = *msg;

    } else {

      tower_pose_previous = *msg;
      tower_pose          = *msg;
      tower_yaw_previous  = mrs_lib::AttitudeConverter(tower_pose.pose.orientation).getYaw();

      got_tower_pose = true;
      return;
    }

    if (tower_reliable) {
      // Detect jump since previous pose
      if (std::pow(tower_pose.pose.position.x - tower_pose_previous.pose.position.x, 2) > 4 ||
          std::pow(tower_pose.pose.position.y - tower_pose_previous.pose.position.y, 2) > 4) {
        ROS_WARN("[Odometry]: Jump detected in Tower Slam pose. Not reliable");

        tower_reliable = false;

        Vec2 pos_vec, vel_vec;
        for (auto &estimator : m_state_estimators) {
          if (estimator.first == "TOWER") {
            estimator.second->getState(0, pos_vec);
            estimator.second->getState(1, vel_vec);
          }
        }

        tower_reliable = false;
      }

      if (current_estimator->getName() == "TOWER") {
        Vec2 vel_vec;
        current_estimator->getState(1, vel_vec);
        if (vel_vec(0) > 5 || vel_vec(1) > 5) {
          ROS_WARN("[Odometry]: Tower Slam velocity too large. Not reliable.");

          tower_reliable = false;
        }
      }
    }


    if (c_tower_msg_ < 100) {
      c_tower_msg_++;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(tower_pose.header.stamp.toSec(), tower_pose_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Tower pose timestamp not OK, not fusing correction.");
    return;
  }

  double yaw_tower;
  {
    std::scoped_lock lock(mutex_tower);
    yaw_tower = mrs_lib::AttitudeConverter(tower_pose.pose.orientation).getYaw();
  }

  yaw_tower          = mrs_lib::unwrapAngle(yaw_tower, tower_yaw_previous);
  tower_yaw_previous = yaw_tower;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_tower, "yaw_tower");

  yaw_tower = mrs_lib::wrapAngle(yaw_tower);

  mrs_msgs::Float64Stamped tower_yaw_out;
  tower_yaw_out.header.stamp    = ros::Time::now();
  tower_yaw_out.header.frame_id = local_origin_frame_id_;
  tower_yaw_out.value           = yaw_tower;
  pub_tower_yaw_.publish(tower_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from Tower SLAM");

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Tower pose. Waiting for other sensors.");
    return;
  }

  {
    std::scoped_lock lock(mutex_tower);

    pos_tower_x = tower_pose.pose.position.x;
    pos_tower_y = tower_pose.pose.position.y;
  }

  // Current orientation
  Eigen::VectorXd hdg_state(1);

  if (current_hdg_estimator->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk);
    hdg_state(0) = orientation_mavros.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator);

    current_hdg_estimator->getState(0, hdg_state);
  }

  // Set innoation variable if ccurnet estimator is tower
  if (current_estimator->getName() == "TOWER") {
    Vec2 pos_vec, innovation;
    current_estimator->getState(0, pos_vec);

    innovation(0) = pos_tower_x - pos_vec(0);
    innovation(1) = pos_tower_y - pos_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = innovation(0);
      odom_main_inno.pose.pose.position.y = innovation(1);
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = 0;
      odom_main_inno.twist.twist.linear.y = 0;
    }
  }

  ROS_WARN_ONCE("[Odometry]: Fusing Tower position");
}
//}

/* //{ callbackAloamOdom() */

void Odometry::callbackAloamOdom(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackAloamOdom");

  aloam_odom_last_update = ros::Time::now();

  {
    std::scoped_lock lock(mutex_aloam);

    if (got_aloam_odom) {
      aloam_odom_previous = aloam_odom;
      aloam_odom          = *msg;

    } else {

      aloam_odom_previous = *msg;
      aloam_odom          = *msg;
      aloam_yaw_previous  = mrs_lib::AttitudeConverter(aloam_odom.pose.pose.orientation).getYaw();

      got_aloam_odom = true;
      return;
    }
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  if (!isTimestampOK(aloam_odom.header.stamp.toSec(), aloam_odom_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: ALOAM odom timestamp not OK, not fusing correction.");
    return;
  }

  /* fuse aloam height //{ */

  //////////////////// Filter out aloam height measurement ////////////////////

  bool   aloam_height_ok = true;
  double measurement;
  {
    std::scoped_lock lock(mutex_aloam);
    measurement = aloam_odom.pose.pose.position.z;
  }
  if (isUavFlying()) {
    if (!aloamHeightFilter->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: VIO height easurement %f declined by median filter.", measurement);
      aloam_height_ok = false;
    }
  }

  /* //////////////////// Fuse main altitude kalman //////////////////// */

  double min_height = -100.0;
  double max_height = 100.0;
  if (measurement < min_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM height measurement %f < %f. Not fusing.", measurement, min_height);
    aloam_height_ok = false;
  }

  if (measurement > max_height) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: ALOAM height measurement %f > %f. Not fusing.", measurement, max_height);
    aloam_height_ok = false;
  }

  // Fuse vio measurement for each altitude estimator
  for (auto &estimator : m_altitude_estimators) {
    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    if (!estimator.second->getStates(current_altitude)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      aloam_height_ok = false;
    }

    if (aloam_height_ok) {
      {
        std::scoped_lock lock(mutex_altitude_estimator);
        altitudeEstimatorCorrection(measurement, "height_aloam", estimator.second);
        if (fabs(measurement) > 100) {
          ROS_WARN("[Odometry]: ALOAM height correction: %f", measurement);
        }
      }
    }
  }

  //}

  /*//{ fuse aloam heading*/
  double yaw_aloam;
  {
    std::scoped_lock lock(mutex_aloam);
    yaw_aloam = mrs_lib::AttitudeConverter(aloam_odom.pose.pose.orientation).getYaw();
  }

  yaw_aloam = mrs_lib::unwrapAngle(yaw_aloam, aloam_yaw_previous);
  yaw_aloam += aloam_offset_hdg_;
  aloam_yaw_previous = yaw_aloam;

  // Apply correction step to all heading estimators
  headingEstimatorsCorrection(yaw_aloam, "yaw_aloam");

  yaw_aloam = mrs_lib::wrapAngle(yaw_aloam);

  mrs_msgs::Float64Stamped aloam_yaw_out;
  aloam_yaw_out.header.stamp    = ros::Time::now();
  aloam_yaw_out.header.frame_id = local_origin_frame_id_;
  aloam_yaw_out.value           = yaw_aloam;
  pub_aloam_yaw_.publish(aloam_yaw_out);

  ROS_WARN_ONCE("[Odometry]: Fusing yaw from ALOAM SLAM");

  /*//}*/

  /*//{ fuse aloam xy */
  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ALOAM odom. Waiting for other sensors.");
    return;
  }

  {
    std::scoped_lock lock(mutex_aloam);

    pos_aloam_x = aloam_odom.pose.pose.position.x + aloam_offset_(0);
    pos_aloam_y = aloam_odom.pose.pose.position.y + aloam_offset_(1);
  }

  // Current orientation
  Eigen::VectorXd hdg_state(1);

  if (current_hdg_estimator->getName() == "PIXHAWK") {

    std::scoped_lock lock(mutex_odom_pixhawk);
    hdg_state(0) = orientation_mavros.vector.z;

  } else {

    std::scoped_lock lock(mutex_current_hdg_estimator);

    current_hdg_estimator->getState(0, hdg_state);
  }

  // Set innoation variable if ccurnet estimator is ALOAM
  if (current_estimator->getName() == "ALOAM") {
    Vec2 pos_vec, innovation;
    current_estimator->getState(0, pos_vec);

    innovation(0) = pos_aloam_x - pos_vec(0);
    innovation(1) = pos_aloam_y - pos_vec(1);

    {
      std::scoped_lock lock(mutex_odom_main_inno);
      odom_main_inno.pose.pose.position.x = innovation(0);
      odom_main_inno.pose.pose.position.y = innovation(1);
      odom_main_inno.pose.pose.position.z = 0;
      odom_main_inno.twist.twist.linear.x = 0;
      odom_main_inno.twist.twist.linear.y = 0;
    }
  }

  ROS_WARN_ONCE("[Odometry]: Fusing ALOAM position");
  /*//}*/
}
//}

/* //{ callbackGarmin() */

void Odometry::callbackGarmin(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGarmin");

  if (got_range) {
    {
      std::scoped_lock lock(mutex_range_garmin);
      range_garmin_previous = range_garmin;
      range_garmin          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_garmin);
    {
      range_garmin_previous = *msg;
      range_garmin          = *msg;
    }
    got_range = true;
  }

  auto range_garmin_tmp = mrs_lib::get_mutexed(mutex_range_garmin, range_garmin);

  height_available_  = true;
  garmin_last_update = ros::Time::now();

  if (!isTimestampOK(range_garmin.header.stamp.toSec(), range_garmin_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Garmin range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: callbackGarmin(): No odom_pixhawk -> cannot untilt range measurement. Returning.");
    return;
  }

  if (!garmin_enabled) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin disabled. Returning.");
    return;
  }

  got_range = true;

  if (!std::isfinite(range_garmin_tmp.range)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Garmin variable \"measurement\" (garmin)!!!");

    return;
  }
  // non-positive measurements check
  if (range_garmin_tmp.range < _garmin_min_altitude_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f < %f. Not fusing.", range_garmin_tmp.range, _garmin_min_altitude_);
    return;
  }

  // max value of measurements check
  if (range_garmin_tmp.range > _garmin_max_altitude_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f > max_altitude: %f. Not fusing.", range_garmin_tmp.range, _garmin_max_altitude_);
    return;
  }

  // innovation gate check
  if (_use_garmin_inno_gate_) {
    for (auto &alt_estimator : m_altitude_estimators) {
      if (alt_estimator.first == "HEIGHT") {
        Eigen::VectorXd alt(1);
        if (!alt_estimator.second->getState(0, alt)) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
          return;
        }
        if (std::pow(range_garmin_tmp.range - alt(0), 2) > _garmin_inno_gate_value_sq_) {
          ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f declined by innovation gate. State value: %f. Not fusing.", range_garmin_tmp.range, alt(0));
          return;
        }
      }
    }
  }

  auto odom_pixhawk_local = mrs_lib::get_mutexed(mutex_odom_pixhawk, odom_pixhawk);
  auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(odom_pixhawk_local.pose.pose.orientation);

  std::ignore = yaw;  // stops the compiler from yelling about not using this variable

  // Check for excessive tilts
  // we do not want to fuse garmin with large tilts as the range will be too unreliable
  // barometer will do a better job in this situation
  double excessive_tilt = false;
  if (std::pow(roll, 2) > _excessive_tilt_sq_ || std::pow(pitch, 2) > _excessive_tilt_sq_) {
    excessive_tilt = true;
  } else {
    excessive_tilt = false;
  }

  if (excessive_tilt) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Excessive tilt detected - roll: %f, pitch: %f. Not fusing.", roll, pitch);
    return;
  }


  double range_fcu = 0;
  try {
    geometry_msgs::TransformStamped tf_fcu2garmin = m_tf_buffer.lookupTransform(fcu_frame_id_, range_garmin_tmp.header.frame_id, ros::Time(0));
    range_fcu = range_garmin_tmp.range - tf_fcu2garmin.transform.translation.z + tf_fcu2garmin.transform.translation.x * tan(pitch) +
                tf_fcu2garmin.transform.translation.y * tan(roll);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_ONCE("Error during transform from \"%s\" frame to \"%s\" frame. Using offset from config file instead. \n\tMSG: %s",
                  range_garmin_tmp.header.frame_id.c_str(), (fcu_frame_id_).c_str(), ex.what());
    range_fcu = range_garmin_tmp.range + _garmin_z_offset_;
  }

  double measurement;
  measurement = range_fcu * cos(roll) * cos(pitch);

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
    if (!garminFilter->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Garmin measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (!garmin_enabled) {
    ROS_WARN_ONCE("[Odometry]: Garmin not enabled. Not fusing range corrections.");
    return;
  }

  // Fuse garmin measurement for each altitude estimator
  for (auto &estimator : m_altitude_estimators) {
    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    if (!estimator.second->getStates(current_altitude)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // create a correction value
    double correction;
    correction = measurement - current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);

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
      height_range = current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) + correction;
    } else {
      height_range = measurement;
    }

    {
      std::scoped_lock lock(mutex_altitude_estimator);
      altitudeEstimatorCorrection(height_range, "height_range", estimator.second);
      if (std::pow(height_range, 2) > 10000) {
        ROS_WARN("[Odometry]: Garmin height correction: %f", height_range);
      }
      estimator.second->getStates(current_altitude);
    }
  }

  ROS_WARN_ONCE("[Odometry]: fusing Garmin rangefinder");
}

//}

/* //{ callbackSonar() */

void Odometry::callbackSonar(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackSonar");

  if (got_range) {
    {
      std::scoped_lock lock(mutex_range_sonar);
      range_sonar_previous = range_sonar;
      range_sonar          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_sonar);
    {
      range_sonar_previous = *msg;
      range_sonar          = *msg;
    }
    got_range = true;
  }

  if (!isTimestampOK(range_sonar.header.stamp.toSec(), range_sonar_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: sonar range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk) {
    return;
  }

  if (!sonar_enabled) {
    return;
  }

  auto odom_pixhawk_local = mrs_lib::get_mutexed(mutex_odom_pixhawk, odom_pixhawk);
  auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(odom_pixhawk_local.pose.pose.orientation);

  std::ignore = yaw;  // stops the compiler from yelling about not using this variable

  // Check for excessive tilts
  // we do not want to fuse sonar with large tilts as the range will be too unreliable
  // barometer will do a better job in this situation
  double excessive_tilt = false;
  if (std::pow(roll, 2) > _excessive_tilt_sq_ || std::pow(pitch, 2) > _excessive_tilt_sq_) {
    excessive_tilt = true;
  } else {
    excessive_tilt = false;
  }


  double range_fcu = 0;
  {
    std::scoped_lock lock(mutex_range_sonar);
    try {
      const ros::Duration             timeout(1.0 / 1000.0);
      geometry_msgs::TransformStamped tf_fcu2sonar = m_tf_buffer.lookupTransform(fcu_frame_id_, range_sonar.header.frame_id, range_sonar.header.stamp, timeout);
      range_fcu = range_sonar.range - tf_fcu2sonar.transform.translation.z + tf_fcu2sonar.transform.translation.x * tan(pitch) +
                  tf_fcu2sonar.transform.translation.y * tan(roll);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(10.0, "Error during transform from \"%s\" frame to \"%s\" frame. Using offset from config file instead. \n\tMSG: %s",
                        range_sonar.header.frame_id.c_str(), (fcu_frame_id_).c_str(), ex.what());
      range_fcu = range_sonar.range + _sonar_z_offset_;
    }
  }

  // calculate the vertical component of range measurement
  double measurement = range_fcu * cos(roll) * cos(pitch);

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in sonar variable \"measurement\" (sonar)!!!");

    return;
  }

  got_range = true;

  //////////////////// Filter out sonar measurement ////////////////////
  // do not fuse sonar measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!sonarFilter->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: sonar measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (!sonar_enabled) {
    ROS_WARN_ONCE("[Odometry]: sonar not enabled. Not fusing range corrections.");
    return;
  }

  if (measurement < 0.01) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: sonar measurement %f < %f. Not fusing.", measurement, 0.01);
    return;
  }

  if (measurement > _sonar_max_valid_altitude_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: sonar measurement %f > %f. Not fusing.", measurement, _sonar_max_valid_altitude_);
    return;
  }

  if (excessive_tilt) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Excessive tilt detected - roll: %f, pitch: %f. Not fusing sonar.", roll, pitch);
    return;
  }

  // Fuse sonar measurement for each altitude estimator
  for (auto &estimator : m_altitude_estimators) {
    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    if (!estimator.second->getStates(current_altitude)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // create a correction value
    double correction;
    correction = measurement - current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT);

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
      std::scoped_lock lock(mutex_altitude_estimator);
      altitudeEstimatorCorrection(height_range, "height_sonar", estimator.second);
      if (fabs(height_range) > 100) {
        ROS_WARN("[Odometry]: sonar height correction: %f", height_range);
      }
      estimator.second->getStates(current_altitude);
      if (estimator.second->getName() == "HEIGHT") {
      }
    }
  }

  ROS_WARN_ONCE("[Odometry]: fusing sonar rangefinder");
}

//}

/* //{ callbackPlane() */

void Odometry::callbackPlane(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackPlane");

  if (got_plane) {
    {
      std::scoped_lock lock(mutex_range_plane);
      range_plane_previous = range_plane;
      range_plane          = *msg;
    }
  } else {
    std::scoped_lock lock(mutex_range_plane);
    {
      range_plane_previous = *msg;
      range_plane          = *msg;
    }
    got_plane = true;
  }

  plane_last_update = ros::Time::now();

  if (!isTimestampOK(range_plane.header.stamp.toSec(), range_plane_previous.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Plane range timestamp not OK, not fusing correction.");
    return;
  }

  if (!got_odom_pixhawk) {
    return;
  }

  double measurement = range_plane.range;

  //////////////////// Filter out plane measurement ////////////////////
  // do not fuse plane measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (isUavFlying()) {
    if (!planeFilter->isValid(measurement)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Plane measurement %f declined by median filter.", measurement);
      return;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////

  if (measurement < range_plane.min_range) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Plane measurement %f < %f. Not fusing.", measurement, range_plane.min_range);
    return;
  }

  if (measurement > range_plane.max_range) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Plane measurement %f > %f. Not fusing.", measurement, range_plane.max_range);
    return;
  }

  if (measurement > _max_plane_altitude_) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Plane measurement %f > %f. Not reliable.", measurement, _max_plane_altitude_);
    return;
  }

  // Fuse plane measurement for each altitude estimator
  for (auto &estimator : m_altitude_estimators) {
    Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
    if (!estimator.second->getStates(current_altitude)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator not initialized.");
      return;
    }

    // set the measurement vector
    double height_range = measurement;
    {
      std::scoped_lock lock(mutex_altitude_estimator);
      altitudeEstimatorCorrection(height_range, "height_plane", estimator.second);
      if (fabs(height_range) > 100) {
        ROS_WARN("[Odometry]: Plane height correction: %f", height_range);
        return;
      }
      estimator.second->getStates(current_altitude);
      if (estimator.second->getName() == "HEIGHT") {
      }
    }
  }

  ROS_WARN_ONCE("[Odometry]: fusing Plane rangefinder");
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
    std::scoped_lock lock(mutex_pixhawk_utm_position);

    pixhawk_utm_position_x = out_x;
    pixhawk_utm_position_y = out_y;
  }

  got_pixhawk_utm = true;

  nav_msgs::Odometry gps_local_odom;
  gps_local_odom.header          = msg->header;
  gps_local_odom.header.frame_id = _uav_name_ + "/gps_origin";

  // | ------------- offset the gps to local_origin ------------- |
  gps_local_odom.pose.pose.position.x = pixhawk_utm_position_x - _utm_origin_x_;
  gps_local_odom.pose.pose.position.y = pixhawk_utm_position_y - _utm_origin_y_;

  {
    std::scoped_lock lock(mutex_odom_pixhawk);

    gps_local_odom.pose.pose.position.z  = odom_pixhawk.pose.pose.position.z;
    gps_local_odom.pose.pose.orientation = odom_pixhawk.pose.pose.orientation;
    gps_local_odom.twist                 = odom_pixhawk.twist;
  }

  // | ------------- publish the gps local odometry ------------- |
  {
    std::scoped_lock lock(mutex_gps_local_odom);

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

  if (uav_in_the_air && control_manager_diag.active_tracker == _null_tracker_) {

    // save the current position
    Vec2 pose;
    current_estimator->getState(0, pose);
    land_position_x   = pose[0];
    land_position_y   = pose[1];
    land_position_set = true;

    uav_in_the_air = false;
  } else if (!uav_in_the_air && control_manager_diag.active_tracker != _null_tracker_) {
    uav_in_the_air = true;
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
    ROS_INFO_THROTTLE(1.0, "[Odometry]: c_bad %d, c_good %d", c_gps_cov_over_lim_, c_gps_cov_ok_);
  } else if (cov_tmp < _gps_fallback_covariance_ok_ && c_gps_cov_ok_ < _gps_fallback_good_samples_ + 1) {
    c_gps_cov_ok_++;
    c_gps_cov_over_lim_ = 0;
    ROS_INFO_THROTTLE(1.0, "[Odometry]: c_bad %d, c_good %d", c_gps_cov_over_lim_, c_gps_cov_ok_);
  }

  // Fallback when GPS covariance over threshold
  if (!gps_in_fallback_ && toUppercase(current_estimator_name) == "GPS" && c_gps_cov_over_lim_ > _gps_fallback_bad_samples_) {

    ROS_WARN_THROTTLE(1.0, "[Odometry]: GPS covariance %f > %f", cov_tmp, _gps_fallback_covariance_limit_);

    std::transform(_gps_fallback_estimator_.begin(), _gps_fallback_estimator_.end(), _gps_fallback_estimator_.begin(), ::toupper);

    // Fallback to optflow
    if (_gps_fallback_estimator_ == "OPTFLOW") {

      if (_optflow_available_ && got_optflow) {
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
        Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
        double          have_alt         = false;
        {
          std::scoped_lock lock(mutex_altitude_estimator);
          if (current_alt_estimator->getStates(current_altitude)) {
            have_alt = true;
          } else {
            ROS_WARN("[Odometry]: Altitude estimator not initialized.");
            have_alt = false;
          }
        }

        if (have_alt) {
          double target_altitude = _gps_fallback_altitude_ - current_altitude(0);

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
              std::scoped_lock lock(mutex_altitude_estimator);
              current_alt_estimator->getStates(current_altitude);
            }

            ros::Duration(t_step).sleep();
            if (std::fabs(current_altitude(0) - target_altitude) < 1.0) {
              break;
            }

            t += t_step;
          }
          ROS_INFO("[Odometry]: Waited %f seonds to reach the target altitude.", t);


          {
            std::scoped_lock lock(mutex_altitude_estimator);
            if (current_alt_estimator->getStates(current_altitude)) {
              ROS_INFO_THROTTLE(1.0, "[Odometry]: Descended to %f m altitude", current_altitude(0));
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
            gps_reliable     = false;
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
    gps_reliable = true;
  }
}

//}

/* //{ callbackGroundTruth() */
void Odometry::callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackGroundTruth");

  {
    std::scoped_lock lock(mutex_ground_truth);

    ground_truth = *msg;
  }

  double rot_x, rot_y, rot_z;
  getGlobalRot(ground_truth.pose.pose.orientation, rot_x, rot_y, rot_z);

  orientation_gt.header   = odom_pixhawk.header;
  orientation_gt.vector.x = rot_x;
  orientation_gt.vector.y = rot_y;
  orientation_gt.vector.z = rot_z;

  try {
    pub_orientation_gt_.publish(orientation_gt);
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

  if (got_odom_t265) {

    {
      std::scoped_lock lock(mutex_odom_t265);

      odom_t265_previous = odom_t265;
      odom_t265          = *msg;
    }

  } else {

    {
      std::scoped_lock lock(mutex_odom_t265);

      odom_t265_previous = *msg;
      odom_t265          = *msg;
    }

    got_odom_t265         = true;
    odom_t265_last_update = ros::Time::now();
    return;
  }

  odom_t265_last_update = ros::Time::now();

  if (!isTimestampOK(odom_t265.header.stamp.toSec(), odom_t265.header.stamp.toSec())) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: T265 odom timestamp not OK.");
  }

  double dt;
  {
    std::scoped_lock lock(mutex_odom_t265);

    dt = (odom_t265.header.stamp - odom_pixhawk_previous.header.stamp).toSec();
  }

  if (!got_init_heading) {

    auto odom_t265_local = mrs_lib::get_mutexed(mutex_odom_t265, odom_t265);
    m_init_heading       = mrs_lib::AttitudeConverter(odom_t265_local.pose.pose.orientation).getYaw();

    got_init_heading = true;
  }

  if (!got_range) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Waiting for rangefinder.");
    return;
  }


  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  /* check NaNs //{ */
  if (t265_reliable && !std::isfinite(odom_t265.pose.pose.position.x)) {
    odom_t265.pose.pose.position.x = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265.pose.pose.position.x\", T265 odom is now unreliable!!!");
    t265_reliable = false;
    return;
  }

  if (t265_reliable && !std::isfinite(odom_t265.pose.pose.position.y)) {
    odom_t265.pose.pose.position.y = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265.pose.pose.position.y\", T265 odom is now unreliable!!!");
    t265_reliable = false;
    return;
  }

  if (t265_reliable && !std::isfinite(odom_t265.pose.pose.position.z)) {
    odom_t265.pose.pose.position.z = 0;
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"odom_t265.pose.pose.position.z\", T265 odom is now unreliable!!!");
    t265_reliable = false;
    return;
  }
  //}

  /* check maximum velocity //{ */

  double vel_x, vel_y, vel_z;
  {
    std::scoped_lock lock(mutex_odom_t265);
    vel_x = (odom_t265.pose.pose.position.x - odom_t265_previous.pose.pose.position.x) / dt;
    vel_y = (odom_t265.pose.pose.position.y - odom_t265_previous.pose.pose.position.y) / dt;
    vel_z = (odom_t265.pose.pose.position.z - odom_t265_previous.pose.pose.position.z) / dt;
  }

  if (vel_x > _max_t265_vel_ || vel_y > _max_t265_vel_ || vel_z > _max_t265_vel_) {
    t265_reliable = false;
    ROS_WARN_THROTTLE(1.0, "[Odometry]: T265 velocity: x: %f, y: %f, z: %f, exceeded %f m/s", vel_x, vel_y, vel_z, _max_t265_vel_);
    return;
  }

  //}

  /* republish t265 odometry //{ */

  if (toUppercase(current_estimator_name) == "T265") {

    /* publish t265 altitude //{ */
    mrs_msgs::Float64Stamped new_altitude;
    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      new_altitude.header = odom_t265.header;
      new_altitude.value  = odom_t265.pose.pose.position.z;
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
      std::scoped_lock lock(mutex_odom_t265);
      orientation.header                = odom_t265.header;
      orientation.pose.pose.orientation = odom_t265.pose.pose.orientation;
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
      std::scoped_lock lock(mutex_odom_t265);
      odom_main = odom_t265;
    }

    odom_main.header.frame_id = _uav_name_ + "/" + toLowercase(current_estimator_name) + "_origin";
    odom_main.child_frame_id  = fcu_frame_id_;
    odom_main.header.stamp    = ros::Time::now();

    if (!odometry_published) {
      std::scoped_lock lock(mutex_odom_local);
      odom_local                         = odom_main;
      odom_local.pose.pose.orientation.x = 0.0;
      odom_local.pose.pose.orientation.y = 0.0;
      odom_local.pose.pose.orientation.z = 0.0;
      odom_local.pose.pose.orientation.w = 1.0;
      m_pos_odom_offset.setZero();
      m_rot_odom_offset = mrs_lib::AttitudeConverter(0, 0, 0);
      m_rot_odom_offset.normalize();
      last_stable_name_ = odom_main.header.frame_id;
      last_local_name_  = odom_main.header.frame_id;
    }
    {
      std::scoped_lock lock(mutex_odom_local);
      if (odom_main.header.frame_id != last_local_name_) {

        last_local_name_ = odom_main.header.frame_id;

        tf2::Vector3 v1, v2;
        tf2::fromMsg(odom_main.pose.pose.position, v1);
        tf2::fromMsg(odom_local.pose.pose.position, v2);
        tf2::Vector3 pos_diff = v1 - v2;
        m_pos_odom_offset     = pos_diff;

        if (odom_local.pose.pose.orientation.w == 0.0) {
          odom_local.pose.pose.orientation = odom_pixhawk.pose.pose.orientation;
        }
        tf2::Quaternion q1       = mrs_lib::AttitudeConverter(odom_main.pose.pose.orientation);
        tf2::Quaternion q2       = mrs_lib::AttitudeConverter(odom_local.pose.pose.orientation);
        tf2::Quaternion rot_diff = q2 * q1.inverse();
        m_rot_odom_offset        = rot_diff;
        m_rot_odom_offset.normalize();
        ROS_WARN("[Odometry]: Changed odometry estimator. Updating offset for stable odometry.");
      }

      odom_local                 = applyOdomOffset(odom_main, m_pos_odom_offset, m_rot_odom_offset);
      odom_local.header.frame_id = local_origin_frame_id_;

      try {
        pub_odom_local_.publish(odom_local);
      }
      catch (...) {
        ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_local_.getTopic().c_str());
      }
    }

    {
      std::scoped_lock lock(mutex_shared_odometry);

      shared_odom = odom_main;
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
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
  } else if (type == "LIDAR") {
    desired_estimator.type     = mrs_msgs::EstimatorType::LIDAR;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::LIDAR;
    desired_alt_estimator.type = mrs_msgs::AltitudeType::HEIGHT;
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
  } else if (type == "TOWER") {
    desired_estimator.type     = mrs_msgs::EstimatorType::TOWER;
    desired_hdg_estimator.type = mrs_msgs::HeadingType::TOWER;
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

  if (toUppercase(current_hdg_estimator_name) == toUppercase(desired_hdg_estimator.name)) {

    success_hdg = true;
    ROS_INFO("[Odometry]: Heading estimator %s already active.", desired_hdg_estimator.name.c_str());

  } else {

    std::scoped_lock lock(mutex_hdg_estimator_type);

    success_hdg = changeCurrentHeadingEstimator(desired_hdg_estimator);
  }

  desired_estimator.name = _state_estimators_names_[desired_estimator.type];

  bool success = false;

  if (toUppercase(current_estimator_name) == toUppercase(desired_estimator.name)) {

    success = true;
    ROS_INFO("[Odometry]: Lateral estimator %s already active.", desired_estimator.name.c_str());

  } else {

    std::scoped_lock lock(mutex_estimator_type);

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
  if (toUppercase(current_alt_estimator_name) == toUppercase(desired_alt_estimator.name)) {

    success_alt = true;
    ROS_INFO("[Odometry]: Altitude estimator %s already active.", desired_alt_estimator.name.c_str());

  } else {
    std::scoped_lock lock(mutex_alt_estimator_type);

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
      std::scoped_lock lock(mutex_estimator_type);

      res.estimator_type.type = _estimator_type.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_estimator_type);

    mrs_msgs::EstimatorType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _state_estimators_names_[desired_estimator.type];
    success                = changeCurrentEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_estimator_type);

    res.estimator_type.type = _estimator_type.type;
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
  } else if (type == "LIDAR") {
    desired_estimator.type = mrs_msgs::EstimatorType::LIDAR;
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
  } else if (type == "TOWER") {
    desired_estimator.type = mrs_msgs::EstimatorType::TOWER;
  } else if (type == "ALOAM") {
    desired_estimator.type = mrs_msgs::EstimatorType::ALOAM;
  } else if (type == "BRICKFLOW") {
    desired_estimator.type = mrs_msgs::EstimatorType::BRICKFLOW;
  } else if (type == "ICP") {
    desired_estimator.type = mrs_msgs::EstimatorType::ICP;
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

  if (toUppercase(current_estimator_name) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Lateral estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  {
    std::scoped_lock lock(mutex_estimator_type);

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
      std::scoped_lock lock(mutex_hdg_estimator_type);

      res.estimator_type.type = _hdg_estimator_type.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_hdg_estimator_type);

    mrs_msgs::HeadingType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _heading_estimators_names_[desired_estimator.type];
    success                = changeCurrentHeadingEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_hdg_estimator_type);

    res.estimator_type.type = _hdg_estimator_type.type;
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
  } else if (type == "TOWER") {
    desired_estimator.type = mrs_msgs::HeadingType::TOWER;
  } else if (type == "ALOAM") {
    desired_estimator.type = mrs_msgs::HeadingType::ALOAM;
  } else if (type == "LIDAR") {
    desired_estimator.type = mrs_msgs::HeadingType::LIDAR;
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

  if (toUppercase(current_hdg_estimator_name) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Heading estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_hdg_estimator_type);

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
      std::scoped_lock lock(mutex_alt_estimator_type);

      res.estimator_type.type = _alt_estimator_type.type;
    }
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_alt_estimator_type);

    mrs_msgs::AltitudeType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _altitude_estimators_names_[desired_estimator.type];
    success                = changeCurrentAltitudeEstimator(desired_estimator);
  }

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  {
    std::scoped_lock lock(mutex_alt_estimator_type);

    res.estimator_type.type = _alt_estimator_type.type;
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
  } else if (type == "BARO") {
    desired_estimator.type = mrs_msgs::AltitudeType::BARO;
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

  if (toUppercase(current_alt_estimator_name) == toUppercase(desired_estimator.name)) {

    ROS_INFO("[Odometry]: Altitude estimator %s already active.", desired_estimator.name.c_str());
    res.success = true;
    res.message = (printOdometryDiag().c_str());
    return true;
  }

  bool success = false;
  {
    std::scoped_lock lock(mutex_alt_estimator_type);

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

  garmin_enabled = req.data;

  // after enabling garmin we want to start correcting the altitude slowly
  if (garmin_enabled) {
    saturate_garmin_corrections_ = true;
    ROS_INFO("[Odometry]: Saturating garmin corrections: true");
  }

  res.success = true;
  res.message = (garmin_enabled ? "Garmin enabled" : "Garmin disabled");

  if (garmin_enabled) {

    ROS_INFO("[Odometry]: Garmin enabled.");
    rtk_altitude_enabled = false;

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
    std::scoped_lock lock(mutex_current_estimator);
    success = current_estimator->getStates(states);
  }

  if (_estimator_type.type == mrs_msgs::EstimatorType::GPS || _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
      _estimator_type.type == mrs_msgs::EstimatorType::RTK) {

    states(0, 0) = odom_pixhawk_shifted.pose.pose.position.x;
    states(0, 1) = odom_pixhawk_shifted.pose.pose.position.y;

  } else {
    // TODO there is a bug: when taking off, the position is set to local_origin instead of current pixhawk odom
    if (!land_position_set) {  // if taking off for the first time

      if (_estimator_type.type == mrs_msgs::EstimatorType::GPS || _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
          _estimator_type.type == mrs_msgs::EstimatorType::RTK) {
        if (!calculatePixhawkOdomOffset()) {
          ROS_ERROR("[Odometry]: Calculating pixhawk odom offset failed");
        }
        states(0, 0) = odom_pixhawk_shifted.pose.pose.position.x;
        states(0, 1) = odom_pixhawk_shifted.pose.pose.position.y;
        ROS_INFO("[Odometry]: Resetting estimators to pijhawk shifted odom x: %f y: %f", states(0, 0), states(0, 1));
      } else {
        states(0, 0) = _local_origin_x_;
        states(0, 1) = _local_origin_y_;
        ROS_INFO("[Odometry]: Resetting estimators to local_origin x: %f y: %f", states(0, 0), states(0, 1));
      }

    } else {  // taking off again
      if (_estimator_type.type == mrs_msgs::EstimatorType::GPS || _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
          _estimator_type.type == mrs_msgs::EstimatorType::RTK) {
        if (!calculatePixhawkOdomOffset()) {
          ROS_ERROR("[Odometry]: Calculating pixhawk odom offset failed");
        }
        states(0, 0) = odom_pixhawk_shifted.pose.pose.position.x;
        states(0, 1) = odom_pixhawk_shifted.pose.pose.position.y;
        ROS_INFO("[Odometry]: Resetting estimators to pixhawk shifted odom x: %f y: %f", states(0, 0), states(0, 1));
      } else {
        states(0, 0) = land_position_x;
        states(0, 1) = land_position_y;
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
    std::scoped_lock lock(mutex_current_estimator);
    success = current_estimator->reset(states);
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

  if (toUppercase(current_estimator_name) == "HECTOR" || toUppercase(current_hdg_estimator_name) == "HECTOR") {
    res.success = false;
    res.message = ("Cannot reset when HECTOR in feedback");
    ROS_WARN("[Odometry]: Cannot switch to HECTOR, when HECTOR is in feedback.");
    return true;
  }

  if (!got_hector_pose) {
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
    geometry_msgs::Pose hector_pose_tmp = mrs_lib::get_mutexed(mutex_hector, hector_pose.pose);
    ros::Duration(t_step).sleep();
    if (std::fabs(hector_pose_tmp.position.x) < 1.0 && std::fabs(hector_pose_tmp.position.y) < 1.0) {
      break;
    }

    t += t_step;
  }

  ROS_INFO("[Odometry]: Waited %f seconds for hector map reset.", t);

  // Reset HECTOR heading
  for (auto &estimator : m_heading_estimators) {
    if (estimator.first == "HECTOR") {
      Eigen::VectorXd hdg(1);
      Eigen::VectorXd hdg_rate(1);
      Eigen::VectorXd hdg_acc(1);
      hdg << 0;
      hdg_rate << 0;
      hdg_acc << 0;
      estimator.second->setState(0, hdg);
      estimator.second->setState(1, hdg_rate);
      estimator.second->setState(2, hdg_acc);
    }
  }

  // Reset HECTOR position
  for (auto &estimator : m_state_estimators) {
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

  hector_reliable = true;

  ROS_WARN("[Odometry]: Hector manually set to reliable.");

  res.success = true;
  res.message = "Hector manually set to reliable";

  return true;
}
//}

/* //{ callbackGyroJump() */

bool Odometry::callbackGyroJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized_)
    return false;

  if (!_simulation_)
    return false;

  if (!callbacks_enabled_) {
    res.success = false;
    res.message = ("Service callbacks are disabled");
    ROS_WARN("[Odometry]: Ignoring service call. Callbacks are disabled.");
    return true;
  }

  Eigen::VectorXd state = Eigen::VectorXd::Zero(1);

  for (auto &estimator : m_heading_estimators) {
    std::scoped_lock lock(mutex_heading_estimator);
    if (estimator.first == "GYRO") {
      estimator.second->getState(0, state);
      state(0) += 1.57;
    }
  }

  ROS_WARN("[Odometry]: Triggered jump in gyro estimator.");

  res.success = true;
  res.message = "Triggered jump in gyro estimator";

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
      "R_pos_lidar: %f\n"
      "R_pos_rtk: %f\n"
      "R_pos_brick: %f\n"
      "R_pos_hector: %f\n"
      "R_pos_tower: %f\n"

      "\nVelocity:\n"
      "R_vel_mavros: %f\n"
      "R_vel_vio: %f\n"
      "R_vel_icp: %f\n"
      "R_vel_lidar: %f\n"
      "R_vel_optflow: %f\n"
      "R_vel_rtk: %f\n"

      "\nAcceleration:\n"
      "R_acc_imu: %f\n",

      config.R_pos_mavros, config.R_pos_vio, config.R_pos_vslam, config.R_pos_lidar, config.R_pos_rtk, config.R_pos_brick, config.R_pos_hector,
      config.R_pos_tower, config.R_vel_mavros, config.R_vel_vio, config.R_vel_icp, config.R_vel_lidar, config.R_vel_optflow, config.R_vel_rtk,
      config.R_acc_imu_lat);

  for (auto &estimator : m_state_estimators) {
    estimator.second->setR(config.R_pos_mavros, map_measurement_name_id.find("pos_mavros")->second);
    estimator.second->setR(config.R_pos_vio, map_measurement_name_id.find("pos_vio")->second);
    estimator.second->setR(config.R_pos_vslam, map_measurement_name_id.find("pos_vslam")->second);
    estimator.second->setR(config.R_pos_lidar, map_measurement_name_id.find("pos_lidar")->second);
    estimator.second->setR(config.R_pos_rtk, map_measurement_name_id.find("pos_rtk")->second);
    estimator.second->setR(config.R_pos_brick, map_measurement_name_id.find("pos_brick")->second);
    estimator.second->setR(config.R_pos_hector, map_measurement_name_id.find("pos_hector")->second);
    estimator.second->setR(config.R_pos_tower, map_measurement_name_id.find("pos_tower")->second);
    estimator.second->setR(config.R_pos_aloam, map_measurement_name_id.find("pos_aloam")->second);

    estimator.second->setR(config.R_vel_mavros, map_measurement_name_id.find("vel_mavros")->second);
    estimator.second->setR(config.R_vel_vio, map_measurement_name_id.find("vel_vio")->second);
    estimator.second->setR(config.R_vel_icp, map_measurement_name_id.find("vel_icp")->second);
    estimator.second->setR(config.R_vel_lidar, map_measurement_name_id.find("vel_lidar")->second);
    estimator.second->setR(config.R_vel_optflow, map_measurement_name_id.find("vel_optflow")->second);
    estimator.second->setR(config.R_vel_rtk, map_measurement_name_id.find("vel_rtk")->second);

    estimator.second->setR(config.R_acc_imu_lat, map_measurement_name_id.find("acc_imu")->second);

    ROS_INFO(
        "Lateral process covariance:\n"
        "Position (0,0): %f\n"
        "Velocity (1,1): %f\n"
        "Acceleration (2,2): %f\n",
        config.Q_pos, config.Q_vel, config.Q_acc);

    estimator.second->setQ(config.Q_pos, Eigen::Vector2i(0, 0));
    estimator.second->setQ(config.Q_vel, Eigen::Vector2i(1, 1));
    estimator.second->setQ(config.Q_acc, Eigen::Vector2i(2, 2));
  }

  ROS_INFO(
      "Altitude measurement covariance:\n"
      "R_height_range: %f\n"
      "R_height_plane: %f\n"
      "R_height_brick: %f\n"
      "R_height_aloam: %f\n"
      "R_height_baro: %f\n"
      "R_vel_baro: %f\n"
      "R_acc_imu: %f\n",
      config.R_height_range, config.R_height_plane, config.R_height_brick, config.R_height_aloam, config.R_height_baro, config.R_vel_baro, config.R_acc_imu);

  for (auto &estimator : m_altitude_estimators) {
    estimator.second->setR(config.R_height_range, map_alt_measurement_name_id.find("height_range")->second);
    estimator.second->setR(config.R_height_plane, map_alt_measurement_name_id.find("height_plane")->second);
    estimator.second->setR(config.R_height_brick, map_alt_measurement_name_id.find("height_brick")->second);
    estimator.second->setR(config.R_height_aloam, map_alt_measurement_name_id.find("height_aloam")->second);
    estimator.second->setR(config.R_height_baro, map_alt_measurement_name_id.find("height_baro")->second);
    estimator.second->setR(config.R_vel_baro, map_alt_measurement_name_id.find("vel_baro")->second);
    estimator.second->setR(config.R_acc_imu, map_alt_measurement_name_id.find("acc_imu")->second);
  }

  ROS_INFO(
      "Heading measurement covariance:\n"
      "R_yaw_compass: %f\n"
      "R_yaw_hector: %f\n"
      "R_yaw_tower: %f\n"
      "R_yaw_aloam: %f\n"
      "R_yaw_brick: %f\n"
      "R_yaw_vio: %f\n"
      "R_yaw_vslam: %f\n"
      "R_yaw_lidar: %f\n"
      "R_rate_gyro: %f\n"
      "R_rate_optflow: %f\n"
      "R_rate_icp: %f\n",
      config.R_yaw_compass, config.R_yaw_hector, config.R_yaw_tower, config.R_yaw_aloam, config.R_yaw_brick, config.R_yaw_vio, config.R_yaw_vslam,
      config.R_yaw_lidar, config.R_rate_gyro, config.R_rate_optflow, config.R_rate_icp);

  for (auto &estimator : m_heading_estimators) {
    estimator.second->setR(config.R_yaw_compass, map_hdg_measurement_name_id.find("yaw_compass")->second);
    estimator.second->setR(config.R_yaw_hector, map_hdg_measurement_name_id.find("yaw_hector")->second);
    estimator.second->setR(config.R_yaw_tower, map_hdg_measurement_name_id.find("yaw_tower")->second);
    estimator.second->setR(config.R_yaw_aloam, map_hdg_measurement_name_id.find("yaw_aloam")->second);
    estimator.second->setR(config.R_yaw_brick, map_hdg_measurement_name_id.find("yaw_brick")->second);
    estimator.second->setR(config.R_yaw_vio, map_hdg_measurement_name_id.find("yaw_vio")->second);
    estimator.second->setR(config.R_yaw_vslam, map_hdg_measurement_name_id.find("yaw_vslam")->second);
    estimator.second->setR(config.R_yaw_lidar, map_hdg_measurement_name_id.find("yaw_lidar")->second);
    estimator.second->setR(config.R_rate_gyro, map_hdg_measurement_name_id.find("rate_gyro")->second);
    estimator.second->setR(config.R_rate_optflow, map_hdg_measurement_name_id.find("rate_optflow")->second);
    estimator.second->setR(config.R_rate_icp, map_hdg_measurement_name_id.find("rate_icp")->second);
  }
}
//}

// --------------------------------------------------------------
// |                      helper functions                      |
// --------------------------------------------------------------

/*  //{ stateEstimatorsPrediction() */

void Odometry::stateEstimatorsPrediction(const geometry_msgs::Vector3 &acc_in, double dt) {

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

  for (auto &estimator : m_state_estimators) {

    // Rotate body frame measurements into estimator frame
    Eigen::VectorXd current_yaw = Eigen::VectorXd::Zero(1);
    for (auto &hdg_estimator : m_heading_estimators) {
      if (estimator.first == "GPS" || estimator.first == "RTK") {
        {
          std::scoped_lock lock(mutex_odom_pixhawk);

          current_yaw(0) = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).getYaw();
        }
        break;
      } else {
        if (estimator.first == hdg_estimator.first) {
          hdg_estimator.second->getState(0, current_yaw);
          break;
        }
      }
    }

    if (estimator.first == "BRICK" && !brick_reliable) {
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
    getRotatedVector(acc_untilted.vector, current_yaw(0), acc_global);

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


    estimator.second->doPrediction(input, dt);
  }
}

//}

/*  //{ stateEstimatorsCorrection() */

void Odometry::stateEstimatorsCorrection(double x, double y, const std::string &measurement_name) {

  std::map<std::string, int>::iterator it_measurement_id = map_measurement_name_id.find(measurement_name);
  if (it_measurement_id == map_measurement_name_id.end()) {
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

  for (auto &estimator : m_state_estimators) {

    mes(0) = x;
    mes(1) = y;

    // Rotate body frame measurements into estimator frame (vel_mavros) is missing, because it is calculated as difference of ENU positions
    // TODO find out what frame of are the VIO velocities
    if (measurement_name == "vel_optflow" || measurement_name == "vel_icp" || measurement_name == "acc_imu") {

      Eigen::VectorXd current_yaw = Eigen::VectorXd::Zero(1);
      for (auto &hdg_estimator : m_heading_estimators) {
        if (estimator.first == "GPS" || estimator.first == "RTK") {
          {
            std::scoped_lock lock(mutex_odom_pixhawk);

            current_yaw(0) = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).getYaw();
          }
          break;
        } else {
          if (estimator.first == hdg_estimator.first) {
            hdg_estimator.second->getState(0, current_yaw);
            break;
          }
        }
      }
      double mes_x, mes_y;
      mes_x  = mes(0) * cos(current_yaw(0)) - mes(1) * sin(current_yaw(0));
      mes_y  = mes(0) * sin(current_yaw(0)) + mes(1) * cos(current_yaw(0));
      mes(0) = mes_x;
      mes(1) = mes_y;
    }

    estimator.second->doCorrection(mes, it_measurement_id->second);
  }
}

//}

/*  //{ altitudeEstimatorCorrection() */

void Odometry::altitudeEstimatorCorrection(double value, const std::string &measurement_name) {

  std::map<std::string, int>::iterator it_measurement_id = map_alt_measurement_name_id.find(measurement_name);
  if (it_measurement_id == map_alt_measurement_name_id.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" (altitudeEstimatorCorrection) !!!");
    return;
  }

  if (fabs(value) > 100) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Altitude estimator correction: %f", value);
  }

  Eigen::VectorXd mes = Eigen::VectorXd::Zero(1);
  mes << value;


  for (auto &estimator : m_altitude_estimators) {
    estimator.second->doCorrection(mes, it_measurement_id->second);
  }
}

//}

/*  //{ altitudeEstimatorCorrection() */

void Odometry::altitudeEstimatorCorrection(double value, const std::string &measurement_name,
                                           const std::shared_ptr<mrs_uav_odometry::AltitudeEstimator> &estimator) {

  std::map<std::string, int>::iterator it_measurement_id = map_alt_measurement_name_id.find(measurement_name);
  if (it_measurement_id == map_alt_measurement_name_id.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" (altitudeEstimatorCorrection) !!!");
    return;
  }

  Eigen::VectorXd mes = Eigen::VectorXd::Zero(1);
  mes << value;

  estimator->doCorrection(mes, it_measurement_id->second);
}

//}

/*  //{ headingEstimatorsPrediction() */

void Odometry::headingEstimatorsPrediction(const double yaw, const double yaw_rate, const double dt) {

  if (dt <= 0.0) {
    ROS_DEBUG_THROTTLE(1.0, "[Odometry]: Lateral estimator prediction dt=%f, skipping prediction.", dt);
    return;
  }

  if (!std::isfinite(yaw)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"yaw\" (headingEstimatorsPrediction) !!!");
    return;
  }

  if (!std::isfinite(yaw_rate)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"yaw rate\" (headingEstimatorsPrediction) !!!");
    return;
  }


  for (auto &estimator : m_heading_estimators) {

    Eigen::VectorXd input = Eigen::VectorXd::Zero(2);
    input << yaw, yaw_rate;

    Eigen::VectorXd current_yaw = Eigen::VectorXd::Zero(1);
    estimator.second->getState(0, current_yaw);
    input(0) = mrs_lib::unwrapAngle(input(0), current_yaw(0));
    if (estimator.second->getName() == "COMPASS") {
    }
    estimator.second->doPrediction(input, dt);
    Eigen::VectorXd yaw_state(1);
    Eigen::VectorXd yaw_rate_state(1);
    estimator.second->getState(0, yaw_state);
    estimator.second->getState(1, yaw_rate_state);
  }
}

//}

/*  //{ headingEstimatorsCorrection() */

void Odometry::headingEstimatorsCorrection(const double value, const std::string &measurement_name) {

  std::map<std::string, int>::iterator it_measurement_id = map_hdg_measurement_name_id.find(measurement_name);
  if (it_measurement_id == map_hdg_measurement_name_id.end()) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: Tried to fuse measurement with invalid name: \'%s\'.", measurement_name.c_str());
    return;
  }

  if (!std::isfinite(value)) {
    ROS_ERROR_THROTTLE(1.0, "[Odometry]: NaN detected in variable \"value\" (headingEstimatorsCorrection) !!!");
    return;
  }


  for (auto &estimator : m_heading_estimators) {

    Eigen::VectorXd mes = Eigen::VectorXd::Zero(1);
    mes << value;

    if (measurement_name == "yaw_compass") {
      Eigen::VectorXd current_yaw = Eigen::VectorXd::Zero(1);
      estimator.second->getState(0, current_yaw);

      mes(0) = mrs_lib::unwrapAngle(mes(0), current_yaw(0));
      if (estimator.second->getName() == "COMPASS") {
      }
    }

    if (estimator.second->doCorrection(mes, it_measurement_id->second)) {
      Eigen::VectorXd yaw_state(1);
      Eigen::VectorXd yaw_rate_state(1);
      estimator.second->getState(0, yaw_state);
      estimator.second->getState(1, yaw_rate_state);
    }
  }
}

//}

/* //{ getGlobalRot() */
void Odometry::getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz) {

  // Get roll, pitch, yaw in body frame
  double r_new, p_new;

  auto [r, p, y] = mrs_lib::AttitudeConverter(q_msg);

  p_new = p * cos(-y) - r * sin(-y);
  r_new = r * cos(-y) + p * sin(-y);

  rx = r_new;
  ry = p_new;
  rz = y;
}
//}

/* //{ getRotatedTilt() */
void Odometry::getRotatedTilt(const geometry_msgs::Quaternion &q_msg, const double &yaw, double &rx, double &ry) {

  tf2::Quaternion q_body = mrs_lib::AttitudeConverter(q_msg).setYaw(0);

  tf2::Quaternion q_yaw = mrs_lib::AttitudeConverter(0, 0, yaw);

  // Get axis pointing upward from the body frame
  tf2::Vector3 body_axis;
  tf2::Vector3 z_axis(0, 0, 1);
  body_axis = quatRotate(q_body, z_axis);

  // Transform upward axis to the world frame
  body_axis = quatRotate(q_yaw, body_axis);

  // Get roll and pitch angles in world frame
  ry = std::atan2(body_axis.getX(), body_axis.getZ());
  rx = std::atan2(body_axis.getY(), body_axis.getZ());
}
//}

/* //{ getRotatedVector() */
void Odometry::getRotatedVector(const geometry_msgs::Vector3 &acc_in, double yaw_in, geometry_msgs::Vector3 &acc_out) {

  tf2::Quaternion q_yaw = mrs_lib::AttitudeConverter(0, 0, yaw_in);

  tf2::Vector3 acc_tf2(acc_in.x, acc_in.y, acc_in.z);

  acc_tf2   = quatRotate(q_yaw, acc_tf2);
  acc_out.x = acc_tf2.getX();
  acc_out.y = acc_tf2.getY();
  acc_out.z = acc_tf2.getZ();
}
//}

/* rotateLateralStates() //{ */

void Odometry::rotateLateralStates(const double yaw_new, const double yaw_old) {

  yaw_diff_ = yaw_new - yaw_old;
  double cy = cos(yaw_diff_);
  double sy = sin(yaw_diff_);

  for (auto &estimator : m_state_estimators) {
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
    for (int i = 0; i < _lateral_n_; i++) {
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
  if (current_hdg_estimator->getName() == "PIXHAWK") {

    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      hdg = mrs_lib::AttitudeConverter(odom_pixhawk.pose.pose.orientation).getYaw();
    }

  } else {

    Eigen::VectorXd hdg_state(1);
    {
      std::scoped_lock lock(mutex_current_hdg_estimator);
      current_hdg_estimator->getState(0, hdg_state);
    }
    hdg = hdg_state(0);
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
  target_estimator.name                    = _estimator_type_names[target_estimator.type];

  Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
  {
    std::scoped_lock lock(mutex_altitude_estimator);
    if (isUavFlying() && !current_alt_estimator->getStates(current_altitude)) {
      ROS_WARN("[Odometry]: Altitude estimator not initialized.");
      return false;
    }
  }

  // Return if already active
  if (toUppercase(current_estimator_name) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired lateral estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  // Optic flow type
  if (target_estimator.type == mrs_msgs::EstimatorType::OPTFLOW) {

    if (!_optflow_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. Optic flow not available in this world.");
      return false;
    }

    if (!got_optflow && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. No new optic flow msgs received.");
      return false;
    }

    if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. Current altitude %f. Must descend to %f.",
                current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), _max_optflow_altitude_);
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_optflow_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_optflow_altitude_);

    // Mavros GPS type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::GPS) {

    if (!_gps_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. GPS not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // Optic flow + Mavros GPS type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {

    if (!_optflow_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Optic flow not available in this world.");
      return false;
    }

    if (!got_optflow && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. No new optic flow msgs received.");
      return false;
    }

    if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Current altitude %f. Must descend to %f.",
                current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), _max_optflow_altitude_);
      return false;
    }

    if (!_gps_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // RTK GPS type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::RTK) {

    if (!_rtk_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. RTK signal not available in this world.");
      return false;
    }

    if (!got_rtk && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. No new rtk msgs received.");
      return false;
    }

    if (!_gps_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // T265 type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::T265) {

    if (!_t265_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. T265 not available in this world.");
      return false;
    }

    if (!got_odom_t265 && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. No new T265 odom msgs received.");
      return false;
    }

    if (!t265_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to T265 type. T265 odometry is not reliable.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // LIDAR localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::LIDAR) {

    if (!_lidar_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to LIDAR type. Lidar localization not available in this world.");
      return false;
    }

    if (!got_lidar_odom && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to LIDAR type. No new lidar odom msgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // Hector SLAM localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::HECTOR) {

    if (!_lidar_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. Lidar localization not available in this world.");
      return false;
    }

    if (!got_hector_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. No new hector msgs received.");
      return false;
    }

    hector_reliable = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // TOWER localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::TOWER) {

    if (!_lidar_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to TOWER type. Lidar localization not available in this world.");
      return false;
    }

    if (!got_tower_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to TOWER type. No new tower msgs received.");
      return false;
    }

    tower_reliable = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // ALOAM SLAM localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ALOAM) {

    if (!_aloam_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAM type. Lidar localization not available in this world.");
      return false;
    }

    if (!got_aloam_odom && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ALOAM type. No new aloam msgs received.");
      return false;
    }

    aloam_reliable = true;

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // ICP localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::ICP) {

    if (!_lidar_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP type. ICP localization not available in this world.");
      return false;
    }

    if (!got_icp_twist && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP type. No new ICP stmsgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // Vio localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::VIO) {

    if (!_vio_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO type. Visual odometry not available in this world.");
      return false;
    }

    if (!got_vio && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO type. No new vio msgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // VSLAM localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::VSLAM) {

    if (!_vslam_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to VSLAM type. Visual odometry not available in this world.");
      return false;
    }

    if (!got_vslam && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to VSLAM type. No new VSLAM msgs received.");
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // brick localization type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::BRICK) {


    if (!_brick_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. brick odometry not available in this world.");
      return false;
    }

    if (!got_brick_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. No new brick msgs received.");
      return false;
    }

    if (!brick_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. brick detection is not reliable");
      return false;
    }

    if (_estimator_type.type != mrs_msgs::EstimatorType::BRICK) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Already in BRICK state estimator.");
      fallback_brick_estimator_type = _estimator_type;
      ROS_INFO("[Odometry]: Fallback from BRICK estimator: %s", _estimator_type.name.c_str());
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // Brick flow type
  } else if (target_estimator.type == mrs_msgs::EstimatorType::BRICKFLOW) {

    if (!_optflow_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. Optic flow not available in this world.");
      return false;
    }

    if (!got_optflow && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. No new optic flow msgs received.");
      return false;
    }

    if (current_altitude(2) > _max_optflow_altitude_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICKFLOW type. Current altitude %f. Must descend to %f.", current_altitude(2), _max_optflow_altitude_);
      return false;
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_default_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_default_altitude_);

    // Mavros GPS type
  } else {

    ROS_ERROR("[Odometry]: Rejected transition to invalid type %s.", target_estimator.name.c_str());
    return false;
  }

  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _active_state_estimators_names_)) {

    {
      std::scoped_lock lock(mutex_current_estimator);

      current_estimator      = m_state_estimators.find(target_estimator.name)->second;
      current_estimator_name = current_estimator->getName();
    }

    ROS_WARN("[Odometry]: Transition to %s state estimator successful", toUppercase(current_estimator_name).c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to non-active state estimator %s", target_estimator.name.c_str());
    is_updating_state_ = false;
    return false;
  }

  _estimator_type      = target_estimator;
  _estimator_type.name = _estimator_type_names[_estimator_type.type];
  estimator_iteration_++;

  is_updating_state_ = false;
  return true;
}

//}

/* //{ changeCurrentAltitudeEstimator() */
bool Odometry::changeCurrentAltitudeEstimator(const mrs_msgs::AltitudeType &desired_estimator) {

  mrs_msgs::AltitudeType target_estimator = desired_estimator;
  target_estimator.name                   = _altitude_type_names[target_estimator.type];

  if (target_estimator.type != mrs_msgs::AltitudeType::HEIGHT && target_estimator.type != mrs_msgs::AltitudeType::PLANE &&
      target_estimator.type != mrs_msgs::AltitudeType::BRICK && target_estimator.type != mrs_msgs::AltitudeType::VIO &&
      target_estimator.type != mrs_msgs::AltitudeType::ALOAM && target_estimator.type != mrs_msgs::AltitudeType::BARO) {
    ROS_ERROR("[Odometry]: Rejected transition to invalid altitude type %d: %s.", target_estimator.type, target_estimator.name.c_str());
    return false;
  }

  // Return if already active
  if (toUppercase(current_alt_estimator_name) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired altitue estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  /* brick type //{ */

  if (target_estimator.type == mrs_msgs::AltitudeType::BRICK) {

    if (!_brick_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. Bricks not available in this world.");
      return false;
    }

    if (!got_brick_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK type. No new brick msgs received.");
      return false;
    }

    if (isUavFlying()) {

      // update the altitude state
      Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
      {
        std::scoped_lock lock(mutex_altitude_estimator);
        if (!current_alt_estimator->getStates(current_altitude)) {
          ROS_WARN("[Odometry]: Altitude estimator not initialized.");
          return false;
        }
      }
      if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_brick_altitude_) {
        ROS_ERROR("[Odometry]: Cannot transition to BRICK type. Current altitude %f. Must descend to %f.",
                  current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), _max_brick_altitude_);
        return false;
      }
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_brick_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_brick_altitude_);
  }

  //}

  /* plane type //{ */

  if (target_estimator.type == mrs_msgs::AltitudeType::PLANE) {

    if (!got_plane && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to PLANE type. No new plane msgs received.");
      return false;
    }

    if (isUavFlying()) {
      // update the altitude state
      Eigen::MatrixXd current_altitude = Eigen::MatrixXd::Zero(_altitude_n_, 1);
      {
        std::scoped_lock lock(mutex_altitude_estimator);
        if (!current_alt_estimator->getStates(current_altitude)) {
          ROS_WARN("[Odometry]: Altitude estimator not initialized.");
          return false;
        }
      }
      if (current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT) > _max_plane_altitude_) {
        ROS_ERROR("[Odometry]: Cannot transition to PLANE type. Current altitude %f. Must descend to %f.",
                  current_altitude(mrs_msgs::AltitudeStateNames::HEIGHT), _max_plane_altitude_);
        return false;
      }
    }

    mrs_lib::set_mutexed(mutex_max_altitude_, _max_plane_altitude_, max_altitude_);
    ROS_WARN("[Odometry]: Setting max_altitude to %.2f", _max_plane_altitude_);
  }

  //}

  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _altitude_estimators_names_)) {
    {
      std::scoped_lock lock(mutex_current_alt_estimator);

      current_alt_estimator      = m_altitude_estimators.find(target_estimator.name)->second;
      current_alt_estimator_name = current_alt_estimator->getName();
    }

    ROS_WARN("[Odometry]: Transition to %s altitude estimator successful", current_alt_estimator_name.c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to nonexistent altitude estimator %s", target_estimator.name.c_str());
    is_updating_state_ = false;
    return false;
  }

  _alt_estimator_type      = target_estimator;
  _alt_estimator_type.name = _altitude_type_names[_alt_estimator_type.type];
  estimator_iteration_++;
  is_updating_state_ = false;
  return true;
}

//}

/* //{ changeCurrentHeadingEstimator() */
bool Odometry::changeCurrentHeadingEstimator(const mrs_msgs::HeadingType &desired_estimator) {

  mrs_msgs::HeadingType target_estimator = desired_estimator;
  target_estimator.name                  = _heading_type_names[target_estimator.type];

  if (target_estimator.type != mrs_msgs::HeadingType::PIXHAWK && target_estimator.type != mrs_msgs::HeadingType::GYRO &&
      target_estimator.type != mrs_msgs::HeadingType::COMPASS && target_estimator.type != mrs_msgs::HeadingType::OPTFLOW &&
      target_estimator.type != mrs_msgs::HeadingType::LIDAR && target_estimator.type != mrs_msgs::HeadingType::HECTOR &&
      target_estimator.type != mrs_msgs::HeadingType::TOWER && target_estimator.type != mrs_msgs::HeadingType::BRICK &&
      target_estimator.type != mrs_msgs::HeadingType::VIO && target_estimator.type != mrs_msgs::HeadingType::VSLAM &&
      target_estimator.type != mrs_msgs::HeadingType::ICP && target_estimator.type != mrs_msgs::HeadingType::BRICKFLOW &&
      target_estimator.type != mrs_msgs::HeadingType::ALOAM) {
    ROS_ERROR("[Odometry]: Rejected transition to invalid type %s.", target_estimator.name.c_str());
    return false;
  }

  // Return if already active
  if (toUppercase(current_hdg_estimator_name) == toUppercase(target_estimator.name)) {
    ROS_INFO("[Odometry]: Desired heading estimator %s already active. Not switching.", target_estimator.name.c_str());
    return true;
  }

  // brick heading type
  if (target_estimator.type == mrs_msgs::HeadingType::BRICK && _hdg_estimator_type.type != mrs_msgs::HeadingType::BRICK) {

    if (!_brick_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. brick odometry not available in this world.");
      return false;
    }

    if (!got_brick_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. No new brick msgs received.");
      return false;
    }

    if (!brick_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to BRICK heading type. brick detection is not reliable");
      return false;
    }

    fallback_brick_hdg_estimator_type = _hdg_estimator_type;
    ROS_INFO("[Odometry]: Fallback from BRICK heading estimator: %s", _hdg_estimator_type.name.c_str());

    // Hector SLAM localization type
  } else if (target_estimator.type == mrs_msgs::HeadingType::HECTOR) {

    if (!_lidar_available_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. Lidar localization not available in this world.");
      return false;
    }

    if (!got_hector_pose && is_ready_to_takeoff_) {
      ROS_ERROR("[Odometry]: Cannot transition to HECTOR type. No new hector msgs received.");
      return false;
    }
  }
  is_updating_state_ = true;
  if (stringInVector(target_estimator.name, _active_heading_estimators_names_)) {
    if (is_initialized_) {

      {
        std::scoped_lock lock(mutex_current_hdg_estimator);
        current_hdg_estimator      = m_heading_estimators.find(target_estimator.name)->second;
        current_hdg_estimator_name = current_hdg_estimator->getName();
      }

    } else {

      {
        std::scoped_lock lock(mutex_current_hdg_estimator);

        current_hdg_estimator      = m_heading_estimators.find(target_estimator.name)->second;
        current_hdg_estimator_name = current_hdg_estimator->getName();
      }
    }


    ROS_WARN("[Odometry]: Transition to %s heading estimator successful", toUppercase(current_hdg_estimator_name).c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to nonexistent heading estimator %s", toUppercase(target_estimator.name).c_str());
    is_updating_state_ = false;
    return false;
  }

  _hdg_estimator_type      = target_estimator;
  _hdg_estimator_type.name = _heading_type_names[_hdg_estimator_type.type];
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
      type.type == mrs_msgs::EstimatorType::RTK || type.type == mrs_msgs::EstimatorType::LIDAR || type.type == mrs_msgs::EstimatorType::VIO ||
      type.type == mrs_msgs::EstimatorType::VSLAM || type.type == mrs_msgs::EstimatorType::BRICK || type.type == mrs_msgs::EstimatorType::T265 ||
      type.type == mrs_msgs::EstimatorType::HECTOR || type.type == mrs_msgs::EstimatorType::TOWER || type.type == mrs_msgs::EstimatorType::BRICKFLOW ||
      type.type == mrs_msgs::EstimatorType::ICP || type.type == mrs_msgs::EstimatorType::ALOAM) {
    return true;
  }

  return false;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::HeadingType &type) {

  if (type.type == mrs_msgs::HeadingType::PIXHAWK || type.type == mrs_msgs::HeadingType::GYRO || type.type == mrs_msgs::HeadingType::COMPASS ||
      type.type == mrs_msgs::HeadingType::OPTFLOW || type.type == mrs_msgs::HeadingType::HECTOR || type.type == mrs_msgs::HeadingType::TOWER ||
      type.type == mrs_msgs::HeadingType::BRICK || type.type == mrs_msgs::HeadingType::VIO || type.type == mrs_msgs::HeadingType::VSLAM ||
      type.type == mrs_msgs::HeadingType::ICP || type.type == mrs_msgs::HeadingType::BRICKFLOW || type.type == mrs_msgs::HeadingType::ALOAM) {
    return true;
  }

  return false;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::AltitudeType &type) {

  if (type.type == mrs_msgs::AltitudeType::HEIGHT || type.type == mrs_msgs::AltitudeType::PLANE || type.type == mrs_msgs::AltitudeType::BRICK ||
      type.type == mrs_msgs::AltitudeType::VIO || type.type == mrs_msgs::AltitudeType::ALOAM || type.type == mrs_msgs::AltitudeType::BARO) {
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
    std::scoped_lock lock(mutex_estimator_type);

    type.type = _estimator_type.type;
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
  } else if (type.type == mrs_msgs::EstimatorType::LIDAR) {
    s_diag += "LIDAR";
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
  } else if (type.type == mrs_msgs::EstimatorType::TOWER) {
    s_diag += "TOWER";
  } else if (type.type == mrs_msgs::EstimatorType::ALOAM) {
    s_diag += "ALOAM";
  } else if (type.type == mrs_msgs::EstimatorType::BRICKFLOW) {
    s_diag += "BRICKFLOW";
  } else if (type.type == mrs_msgs::EstimatorType::ICP) {
    s_diag += "ICP";
  } else {
    s_diag += "UNKNOWN";
  }

  //}

  /* heading //{ */

  mrs_msgs::HeadingType hdg_type;

  {
    std::scoped_lock lock(mutex_hdg_estimator_type);

    hdg_type.type = _hdg_estimator_type.type;
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
  } else if (hdg_type.type == mrs_msgs::HeadingType::TOWER) {
    s_diag += "TOWER";
  } else if (hdg_type.type == mrs_msgs::HeadingType::ALOAM) {
    s_diag += "ALOAM";
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
  } else {
    s_diag += "UNKNOWN";
  }

  //}

  // altitude
  mrs_msgs::AltitudeType alt_type;

  {
    std::scoped_lock lock(mutex_alt_estimator_type);

    alt_type.type = _alt_estimator_type.type;
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
  } else if (alt_type.type == mrs_msgs::AltitudeType::BARO) {
    s_diag += "BARO";
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

  if (!got_pixhawk_utm || !got_odom_pixhawk) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: cannot calculate pixhawk_odom_offset, waiting for data: UTM: %s, ODOM: %s", btoa(got_pixhawk_utm),
                      btoa(got_odom_pixhawk));
    return false;
  }

  if (got_pixhawk_odom_offset) {
    return true;
  }

  // when we have defined our home position, set local origin offset
  if (got_odom_pixhawk) {

    {
      std::scoped_lock lock(mutex_odom_pixhawk, mutex_pixhawk_utm_position);

      pixhawk_odom_offset_x = (pixhawk_utm_position_x - odom_pixhawk.pose.pose.position.x) - _utm_origin_x_;
      pixhawk_odom_offset_y = (pixhawk_utm_position_y - odom_pixhawk.pose.pose.position.y) - _utm_origin_y_;
    }

    ROS_INFO("[Odometry]: pixhawk_odom_offset based in local_utm calculated as: x: %f, y: %f", pixhawk_odom_offset_x, pixhawk_odom_offset_y);

    got_pixhawk_odom_offset = true;
    return true;

    // when we have not define our home position, define it as our averaged home position
  } else {

    {
      std::scoped_lock lock(mutex_odom_pixhawk);

      pixhawk_odom_offset_x = _local_origin_x_ - odom_pixhawk.pose.pose.position.x;
      pixhawk_odom_offset_y = _local_origin_y_ - odom_pixhawk.pose.pose.position.y;
    }

    ROS_INFO("[Odometry]: pixhawk_odom_offset based in local_origin calculated as: x: %f, y: %f", pixhawk_odom_offset_x, pixhawk_odom_offset_y);

    got_pixhawk_odom_offset = true;
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
