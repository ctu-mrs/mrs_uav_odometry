#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/RtkFixType.h>
#include <mrs_msgs/OdometryDiag.h>
#include <mrs_msgs/OdometryMode.h>
#include <mrs_msgs/ChangeOdometryMode.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/LkfStates.h>
#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/String.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/Lkf.h>
#include <mrs_lib/GpsConversions.h>
#include <mrs_lib/ParamLoader.h>

#include <range_filter.h>
#include <mrs_odometry/lkfConfig.h>

#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <locale>
#include <Eigen/Eigen>
#include <math.h>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <fstream>

#define USE_RANGEFINDER 1
#define STRING_EQUAL 0
#define btoa(x) ((x) ? "true" : "false")
#define NAME_OF(v) #v

namespace mrs_odometry
{

/* //{ class Odometry */

class Odometry : public nodelet::Nodelet {

public:
  virtual void onInit();

public:
  void publishMessage();  // definition of callback function
  int  rate_;
  bool is_initialized = false;

private:
  std::string uav_name;
  bool        simulation_;
  bool        _rosbag;
  bool        use_gt_orientation_;

  bool _fuse_optflow_velocity = false;
  bool _fuse_mavros_tilts     = true;
  bool _fuse_mavros_velocity  = true;
  bool _fuse_mavros_position  = true;
  bool _fuse_rtk_position     = false;
  bool _fuse_vio_velocity     = false;
  bool _fuse_vio_position     = false;
  bool _fuse_icp_velocity     = false;
  bool _fuse_icp_position     = false;
  bool _publish_fused_odom;
  bool _dynamic_optflow_cov = false;

  double _max_optflow_altitude;
  double _max_default_altitude;
  int    _min_satellites;

  ros::NodeHandle nh_;

private:
  ros::Publisher pub_odom_;       // the main fused odometry
  ros::Publisher pub_slow_odom_;  // the main fused odometry, just slow
  ros::Publisher pub_rtk_local;
  ros::Publisher pub_rtk_local_odom;
  ros::Publisher pub_gps_local_odom;
  ros::Publisher pub_orientation_gt_;
  ros::Publisher pub_orientation_mavros_;
  ros::Publisher pub_target_attitude_global_;
  ros::Publisher pub_odometry_diag_;
  ros::Publisher pub_altitude_;
  ros::Publisher pub_max_altitude_;
  ros::Publisher pub_lkf_states_;

private:
  ros::Subscriber sub_global_position_;
  ros::Subscriber sub_tracker_status_;

  // Pixhawk odometry subscriber and callback
  ros::Subscriber sub_pixhawk_;
  ros::Subscriber sub_optflow_;
  ros::Subscriber sub_optflow_stddev_;
  ros::Subscriber sub_vio_;
  ros::Subscriber rtk_gps_sub_;
  ros::Subscriber sub_icp_relative__;
  ros::Subscriber sub_icp_global__;
  ros::Subscriber sub_target_attitude_;
  ros::Subscriber sub_ground_truth_;
  ros::Subscriber sub_mavros_diagnostic_;

private:
  ros::ServiceServer ser_reset_lateral_kalman_;
  ros::ServiceServer ser_averaging_;
  ros::ServiceServer ser_teraranger_;
  ros::ServiceServer ser_garmin_;
  ros::ServiceServer ser_toggle_rtk_altitude;
  ros::ServiceServer ser_toggle_rtk_pos_fusion;
  ros::ServiceServer ser_toggle_vio_pos_fusion;
  ros::ServiceServer ser_toggle_vio_vel_fusion;
  ros::ServiceServer ser_toggle_icp_pos_fusion;
  ros::ServiceServer ser_toggle_icp_vel_fusion;
  ros::ServiceServer ser_toggle_optflow_vel_fusion;
  ros::ServiceServer ser_toggle_mavros_pos_fusion;
  ros::ServiceServer ser_toggle_mavros_vel_fusion;
  ros::ServiceServer ser_toggle_mavros_tilts_fusion;
  ros::ServiceServer ser_change_odometry_mode;
  ros::ServiceServer ser_change_odometry_mode_string;

private:
  tf::TransformBroadcaster *broadcaster_;

  /* dynamic_reconfigure::Server<mrs_odometry::lkfConfig>               server; */
  /* dynamic_reconfigure::Server<mrs_odometry::lkfConfig>::CallbackType f; */

  nav_msgs::Odometry odom_pixhawk;
  std::mutex         mutex_odom;
  nav_msgs::Odometry odom_pixhawk_previous;
  ros::Time          odom_pixhawk_last_update;
  std::mutex         mutex_gps_local_odom;

  geometry_msgs::TwistStamped optflow_twist;
  std::mutex                  mutex_optflow;
  geometry_msgs::TwistStamped optflow_twist_previous;
  ros::Time                   optflow_twist_last_update;
  geometry_msgs::Vector3      optflow_stddev;
  std::mutex                  mutex_optflow_stddev;

  nav_msgs::Odometry odom_vio;
  std::mutex         mutex_odom_vio;
  nav_msgs::Odometry odom_vio_previous;
  ros::Time          odom_vio_last_update;

  geometry_msgs::Vector3Stamped orientation_mavros;
  geometry_msgs::Vector3Stamped orientation_gt;

  mavros_msgs::AttitudeTarget target_attitude;
  mavros_msgs::AttitudeTarget target_attitude_previous;
  ros::Time                   target_attitude_last_update;
  std::mutex                  mutex_target_attitude;

  std::mutex       mutex_rtk;
  mrs_msgs::RtkGps rtk_odom_previous;
  mrs_msgs::RtkGps rtk_odom;
  ros::Time        rtk_last_update;

  std::mutex         mutex_icp;
  nav_msgs::Odometry icp_odom;
  nav_msgs::Odometry icp_odom_previous;
  ros::Time          icp_odom_last_update;

  std::mutex         mutex_icp_global;
  nav_msgs::Odometry icp_global_odom;
  nav_msgs::Odometry icp_global_odom_previous;
  ros::Time          icp_global_odom_last_update;

  std::mutex         mutex_ground_truth;
  nav_msgs::Odometry ground_truth;

  mrs_msgs::RtkGps rtk_local_previous;
  mrs_msgs::RtkGps rtk_local;

  mrs_msgs::OdometryMode   _odometry_mode;
  mrs_msgs::OdometryMode   _odometry_mode_takeoff;
  std::vector<std::string> _odometry_mode_names;
  std::mutex               mutex_odometry_mode;

  void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackOptflowTwist(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackOptflowStddev(const geometry_msgs::Vector3ConstPtr &msg);
  void callbackGlobalPosition(const sensor_msgs::NavSatFixConstPtr &msg);
  bool callbackToggleTeraranger(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleRtkHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleRtkPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleVioPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleVioVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleIcpPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleIcpVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleMavrosPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleMavrosVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleMavrosTilts(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleOptflowVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackChangeOdometryMode(mrs_msgs::ChangeOdometryMode::Request &req, mrs_msgs::ChangeOdometryMode::Response &res);
  bool callbackChangeOdometryModeString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackResetKalman(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void        callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg);
  void        callbackIcpRelative(const nav_msgs::OdometryConstPtr &msg);
  void        callbackIcpAbsolute(const nav_msgs::OdometryConstPtr &msg);
  void        callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg);
  void        callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg);
  void        callbackReconfigure(mrs_odometry::lkfConfig &config, uint32_t level);
  void        callbackMavrosDiag(const mrs_msgs::MavrosDiagnosticsConstPtr &msg);
  void        getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz);
  bool        setOdometryModeTo(const mrs_msgs::OdometryMode &target_mode);
  bool        isValidMode(const mrs_msgs::OdometryMode &mode);
  std::string printOdometryDiag();

  // for keeping new odom
  nav_msgs::Odometry shared_odom;
  std::mutex         mutex_shared_odometry;

  nav_msgs::Odometry rtk_local_odom;
  std::mutex         mutex_rtk_local_odom;

  // Teraranger altitude subscriber and callback
  ros::Subscriber    sub_terarangerone_;
  sensor_msgs::Range range_terarangerone_;
  void               callbackTeraranger(const sensor_msgs::RangeConstPtr &msg);
  RangeFilter *      terarangerFilter;
  int                trg_filter_buffer_size;
  double             trg_max_valid_altitude;
  double             trg_filter_max_difference;
  ros::Time          trg_last_update;
  double             TrgMaxQ, TrgMinQ, TrgQChangeRate;

  // Garmin altitude subscriber and callback
  ros::Subscriber    sub_garmin_;
  sensor_msgs::Range range_garmin_;
  void               callbackGarmin(const sensor_msgs::RangeConstPtr &msg);
  RangeFilter *      garminFilter;
  int                garmin_filter_buffer_size;
  double             garmin_max_valid_altitude;
  double             garmin_filter_max_difference;
  ros::Time          garmin_last_update;
  double             GarminMaxQ, GarminMinQ, GarminQChangeRate;

  bool got_odom, got_optflow, got_range, got_global_position, got_rtk, got_icp, got_icp_global, got_target_attitude, got_vio;
  bool got_altitude_sensors, got_lateral_sensors;
  int  got_icp_counter;
  int  got_icp_global_counter;
  int  got_rtk_counter;
  bool got_rtk_fix;

  geometry_msgs::Vector3Stamped target_attitude_global;

  // for setting home position
  bool   set_home_on_start;
  double home_utm_x, home_utm_y;          // position set as a utm home position
  double utm_position_x, utm_position_y;  // current utm position

  // subscribing to tracker status
  mrs_msgs::TrackerStatus tracker_status;
  void                    callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg);
  bool                    got_tracker_status;
  bool                    isUavFlying();

  // offset to adjust the local origin
  double local_origin_offset_x, local_origin_offset_y;

  // initial position
  double init_pose_x, init_pose_y, init_pose_z, init_pose_yaw;
  bool   init_pose_set = false;

  // altitude kalman
  int             altitude_n, altitude_m, altitude_p;
  Eigen::MatrixXd A1, B1, R1, Q1, Q3, P1;
  mrs_lib::Lkf *  main_altitude_kalman;
  mrs_lib::Lkf *  failsafe_teraranger_kalman;
  std::mutex      mutex_main_altitude_kalman;
  std::mutex      mutex_failsafe_altitude_kalman;

  // lateral kalman
  int             lateral_n, lateral_m, lateral_p;
  Eigen::MatrixXd A2, B2, R2, P_vel, P_pos, P_ang;
  Eigen::MatrixXd Q_pos_rtk, Q_pos_icp, Q_vel_icp, Q_vel_mavros, Q_pos_mavros, Q_vel_optflow, Q_tilt, Q_pos_vio, Q_vel_vio;
  Eigen::MatrixXd Q_vel_optflow_dynamic_x, Q_vel_optflow_dynamic_y;
  mrs_lib::Lkf *  lateralKalmanX;
  mrs_lib::Lkf *  lateralKalmanY;
  std::mutex      mutex_lateral_kalman_x, mutex_lateral_kalman_y;

  // averaging of home position
  double gpos_average_x, gpos_average_y;
  double start_position_average_x, start_position_average_y;
  bool   averaging, averaging_started, done_averaging;
  int    averaging_num_samples;
  void   startAveraging();
  int    averaging_got_samples;
  bool   callbackAveraging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool   got_home_position_fix;

  bool odometry_published;

  mrs_msgs::MavrosDiagnostics mavros_diag;
  std::mutex                  mutex_mavros_diag;

  // reliability of gps
  int  max_altitude;
  bool gps_reliable       = false;
  bool _gps_available     = false;
  bool _vio_available     = false;
  bool _optflow_available = false;
  bool _rtk_available     = false;
  bool _lidar_available   = false;

  // use differential gps
  bool   use_differential_gps = false;
  bool   pass_rtk_as_new_odom = false;
  double max_pos_correction;
  double max_altitude_correction_;

  // disabling teraranger on the flight
  bool teraranger_enabled;
  bool garmin_enabled;

  ros::Timer slow_odom_timer;
  ros::Timer diag_timer;
  ros::Timer lkf_states_timer;
  ros::Timer max_altitude_timer;
  ros::Timer topic_watcher_timer;
  int        slow_odom_rate;
  int        diag_rate;
  int        lkf_states_rate;
  int        max_altitude_rate;
  int        topic_watcher_rate = 1;
  void       slowOdomTimer(const ros::TimerEvent &event);
  void       diagTimer(const ros::TimerEvent &event);
  void       lkfStatesTimer(const ros::TimerEvent &event);
  void       rtkRateTimer(const ros::TimerEvent &event);
  void       maxAltitudeTimer(const ros::TimerEvent &event);
  void       topicWatcherTimer(const ros::TimerEvent &event);

  // for fusing rtk altitude
  double trg_z_offset_;
  double garmin_z_offset_;

private:
  // for fusing rtk altitude
  bool       rtk_altitude_enabled;
  double     rtk_altitude_integral;
  ros::Timer rtk_rate_timer;
  double     rtkQ;
  double     rtk_max_down_difference_;
  double     rtk_max_abs_difference_;

private:
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler *profiler;
  mrs_lib::Routine * routine_main_timer;
  mrs_lib::Routine * routine_odometry_callback;
  mrs_lib::Routine * routine_optflow_callback;
  mrs_lib::Routine * routine_vio_callback;
  mrs_lib::Routine * routine_garmin_callback;
  mrs_lib::Routine * routine_rtk_callback;
  mrs_lib::Routine * routine_icp_callback;
  mrs_lib::Routine * routine_icp_global_callback;


  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  boost::recursive_mutex                      config_mutex_;
  typedef mrs_odometry::lkfConfig             Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>        reconfigure_server_;
  void                                        drs_callback(mrs_odometry::lkfConfig &config, uint32_t level);
  mrs_odometry::lkfConfig                     last_drs_config;

private:
  // ############### stuff for adding another source of altitude data ###############

  /* bool object_altitude_enabled; */
  /* bool got_object_altitude; */
  /* object_detection::ObjectWithType object_altitude; */
  /* std::mutex mutex_object_altitude; */
  /* void callbackObjectHeight(const object_detection::ObjectWithTypeConstPtr &msg); */
  /* TrgFilter * objectAltitudeFilter; */
  /* int object_filter_buffer_size; */
  /* double object_max_valid_altitude; */
  /* double object_filter_max_difference; */
  /* ros::Time object_altitude_last_update; */
  /* double objectQ; */
  /* double static_object_height_, dynamic_object_height_; */
  /* double object_altitude_max_down_difference_; */
  /* double object_altitude_max_abs_difference_; */
  /* double mobius_z_offset_; */

  /* ros::Subscriber object_altitude_sub; */
  /* bool callbackToggleObjectHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); */
  /* ros::ServiceServer ser_object_altitude_; */
};

//}

/* //{ onInit() */

void Odometry::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Odometry]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "Odometry");

  param_loader.load_param("uav_name", uav_name);

  odometry_published    = false;
  got_odom              = false;
  got_optflow           = false;
  got_vio               = false;
  got_rtk               = false;
  got_rtk_fix           = false;
  got_global_position   = false;
  got_tracker_status    = false;
  got_home_position_fix = false;
  got_altitude_sensors  = false;
  got_lateral_sensors   = false;
  got_rtk_counter       = 0;

  // got_object_altitude = false;

#if USE_RANGEFINDER == 1
  got_range = false;
#else
  got_range = true;
#endif

  utm_position_x = 0;
  utm_position_y = 0;

  // --------------------------------------------------------------
  // |                        odometry mode                       |
  // --------------------------------------------------------------

  // prepare the array of names
  // IMPORTANT, update this with each update of the OdometryMode message
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::OTHER));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::OPTFLOW));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::GPS));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::OPTFLOWGPS));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::RTK));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::ICP));
  _odometry_mode_names.push_back(NAME_OF(mrs_msgs::OdometryMode::VIO));

  ROS_WARN("[Odometry]: SAFETY Checking the OdometryMode2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::OdometryMode::MODE_COUNT; i++) {
    std::size_t found       = _odometry_mode_names[i].find_last_of(":");
    _odometry_mode_names[i] = _odometry_mode_names[i].substr(found + 1);
    ROS_INFO("[Odometry]: _odometry_mode[%d]=%s", i, _odometry_mode_names[i].c_str());
  }

  param_loader.load_param("rate", rate_);

  param_loader.load_param("simulation", simulation_);
  param_loader.load_param("slow_odom_rate", slow_odom_rate);
  param_loader.load_param("diag_rate", diag_rate);
  param_loader.load_param("max_altitude_rate", max_altitude_rate);
  param_loader.load_param("lkf_states_rate", lkf_states_rate);
  param_loader.load_param("trgFilterBufferSize", trg_filter_buffer_size);
  param_loader.load_param("trgFilterMaxValidAltitude", trg_max_valid_altitude);
  param_loader.load_param("trgFilterMaxDifference", trg_filter_max_difference);

  param_loader.load_param("garminFilterBufferSize", garmin_filter_buffer_size);
  param_loader.load_param("garminFilterMaxValidAltitude", garmin_max_valid_altitude);
  param_loader.load_param("garminFilterMaxDifference", garmin_filter_max_difference);

  /* param_loader.load_param("objectAltitudeBufferSize", object_filter_buffer_size); */
  /* param_loader.load_param("objectAltitudeValidAltitude", object_max_valid_altitude); */
  /* param_loader.load_param("objectAltitudeMaxDifference", object_filter_max_difference); */
  /* param_loader.load_param("altitude/objectQ", objectQ); */
  /* param_loader.load_param("staticObjectHeight", static_object_height_); */
  /* param_loader.load_param("dynamicObjectHeight", dynamic_object_height_); */
  /* param_loader.load_param("mobius_z_offset_", trg_z_offset_); */

  param_loader.load_param("trg_z_offset", trg_z_offset_);
  param_loader.load_param("garmin_z_offset", garmin_z_offset_);

  param_loader.load_param("home_utm_x", home_utm_x);
  param_loader.load_param("home_utm_y", home_utm_y);

  if (home_utm_x == 0 || home_utm_y == 0) {

    set_home_on_start = false;

  } else {

    param_loader.load_param("use_home_position", set_home_on_start);

    if (set_home_on_start)
      ROS_INFO("[Odometry]: SetHomeOnStart enabled");
    else
      ROS_INFO("[Odometry]: SetHomeOnStart disabled");
  }

  if (set_home_on_start)
    ROS_INFO("[Odometry]: Setting home position on %.5f %.5f", home_utm_x, home_utm_y);

  averaging = false;

  local_origin_offset_x = 0;
  local_origin_offset_y = 0;

  // averaging
  param_loader.load_param("averagingNumSamples", averaging_num_samples);
  averaging_started     = false;
  done_averaging        = false;
  averaging_got_samples = 0;

  // load init pose from config file
  std::string path = ros::package::getPath("mrs_odometry");
  path += "/config/init_pose/init_pose.csv";
  std::ifstream init_pose_file(path, std::ifstream::in);
  if (init_pose_file.is_open()) {
    std::string s0, s1, s2, s3;
    std::getline(init_pose_file, s0, ',');
    std::getline(init_pose_file, s1, ',');
    init_pose_x = std::atof(s1.c_str());
    std::getline(init_pose_file, s2, ',');
    init_pose_y = std::atof(s2.c_str());
    init_pose_file.close();
  } else {
    ROS_ERROR("[Odometry]: Error opening file");
  }

  // Optic flow
  param_loader.load_param("max_optflow_altitude", _max_optflow_altitude);
  param_loader.load_param("max_default_altitude", _max_default_altitude);
  param_loader.load_param("lateral/dynamic_optflow_cov", _dynamic_optflow_cov);
  optflow_stddev.x = 1.0;
  optflow_stddev.y = 1.0;
  optflow_stddev.z = 1.0;

  // Localization sources availability
  param_loader.load_param("min_satellites", _min_satellites);
  param_loader.load_param("gps_available", _gps_available);
  param_loader.load_param("vio_available", _vio_available);
  param_loader.load_param("optflow_available", _optflow_available);
  param_loader.load_param("rtk_available", _rtk_available);
  param_loader.load_param("lidar_available", _lidar_available);
  gps_reliable = _gps_available;

  // Takeoff mode
  std::string takeoff_mode;
  param_loader.load_param("takeoff_mode", takeoff_mode);
  std::transform(takeoff_mode.begin(), takeoff_mode.end(), takeoff_mode.begin(), ::toupper);
  size_t pos                  = std::distance(_odometry_mode_names.begin(), find(_odometry_mode_names.begin(), _odometry_mode_names.end(), takeoff_mode));
  _odometry_mode_takeoff.name = takeoff_mode;
  _odometry_mode_takeoff.mode = (int)pos;

  //{ altitude kalman init
  // declare and initialize variables for the altitude KF
  param_loader.load_param("altitude/numberOfVariables", altitude_n);
  param_loader.load_param("altitude/numberOfInputs", altitude_m);
  param_loader.load_param("altitude/numberOfMeasurements", altitude_p);

  A1 = Eigen::MatrixXd::Zero(altitude_n, altitude_n);
  B1 = Eigen::MatrixXd::Zero(altitude_n, altitude_m);
  R1 = Eigen::MatrixXd::Zero(altitude_n, altitude_n);
  Q1 = Eigen::MatrixXd::Zero(altitude_p, altitude_p);
  Q3 = Eigen::MatrixXd::Zero(altitude_p, altitude_p);
  P1 = Eigen::MatrixXd::Zero(altitude_p, altitude_n);

  param_loader.load_matrix_static("altitude/A", A1, altitude_n, altitude_n);
  param_loader.load_matrix_static("altitude/B", B1, altitude_n, altitude_m);

  param_loader.load_matrix_static("altitude/R", R1, altitude_n, altitude_n);

  param_loader.load_matrix_static("altitude/Q", Q1, altitude_p, altitude_p);

  Q3 = Q1;

  param_loader.load_matrix_static("altitude/P", P1, altitude_p, altitude_n);

  param_loader.load_param("altitude/TrgMaxQ", TrgMaxQ);
  param_loader.load_param("altitude/TrgMinQ", TrgMinQ);
  param_loader.load_param("altitude/TrgQChangeRate", TrgQChangeRate);

  param_loader.load_param("altitude/GarminMaxQ", GarminMaxQ);
  param_loader.load_param("altitude/GarminMinQ", GarminMinQ);
  param_loader.load_param("altitude/GarminQChangeRate", GarminQChangeRate);

  param_loader.load_param("altitude/rtkQ", rtkQ);

  // failsafes for altitude fusion
  param_loader.load_param("altitude/rtk_max_down_difference", rtk_max_down_difference_);
  param_loader.load_param("altitude/rtk_max_abs_difference", rtk_max_abs_difference_);

  /* param_loader.load_param("altitude/object_altitude_max_down_difference", object_altitude_max_down_difference_); */
  /* param_loader.load_param("altitude/object_altitude_max_abs_difference", object_altitude_max_abs_difference_); */

  terarangerFilter = new RangeFilter(trg_filter_buffer_size, trg_max_valid_altitude, trg_filter_max_difference);
  garminFilter     = new RangeFilter(garmin_filter_buffer_size, garmin_max_valid_altitude, garmin_filter_max_difference);

  ROS_INFO("[Odometry]: Garmin max valid altitude: %2.2f", garmin_max_valid_altitude);
  /* objectAltitudeFilter = new RangeFilter(object_filter_buffer_size, 0, false, object_max_valid_altitude, object_filter_max_difference); */

  main_altitude_kalman       = new mrs_lib::Lkf(altitude_n, altitude_m, altitude_p, A1, B1, R1, Q1, P1);
  failsafe_teraranger_kalman = new mrs_lib::Lkf(altitude_n, altitude_m, altitude_p, A1, B1, R1, Q1, P1);

  // initialize the altitude for standing uav
  main_altitude_kalman->setState(0, 0.3);
  failsafe_teraranger_kalman->setState(0, 0.3);

  ROS_INFO_STREAM("[Odometry]: Altitude Kalman Filter was initiated with following parameters: n: "
                  << altitude_n << ", m: " << altitude_m << ", p: " << altitude_p << ", A: " << A1 << ", B: " << B1 << ", R: " << R1 << ", Q: " << Q1
                  << ", P: " << P1 << ", TrgMaxQ: " << TrgMaxQ << ", TrgMinQ: " << TrgMinQ << ", TrgQChangeRate: " << TrgQChangeRate);

  ROS_INFO("[Odometry]: Altitude kalman prepared");
  //}

  //{ lateral kalman init
  // declare and initialize variables for the lateral KF
  param_loader.load_param("lateral/numberOfVariables", lateral_n);
  param_loader.load_param("lateral/numberOfInputs", lateral_m);
  param_loader.load_param("lateral/numberOfMeasurements", lateral_p);
  param_loader.load_param("lateral/max_pos_correction", max_pos_correction);


  A2 = Eigen::MatrixXd::Zero(lateral_n, lateral_n);
  if (lateral_m > 0)
    B2 = Eigen::MatrixXd::Zero(lateral_n, lateral_m);
  R2            = Eigen::MatrixXd::Zero(lateral_n, lateral_n);
  P_ang         = Eigen::MatrixXd::Zero(lateral_p, lateral_n);
  P_vel         = Eigen::MatrixXd::Zero(lateral_p, lateral_n);
  P_pos         = Eigen::MatrixXd::Zero(lateral_p, lateral_n);
  Q_vel_optflow = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_tilt        = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_vel_mavros  = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_pos_mavros  = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_pos_rtk     = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_vel_icp     = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_pos_icp     = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_vel_vio     = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  Q_pos_vio     = Eigen::MatrixXd::Zero(lateral_p, lateral_p);

  // mapping of measurements to states
  P_pos(0, 0) = 1;
  P_vel(0, 1) = 1;
  P_ang(0, 5) = 1;

  param_loader.load_param("lateral/Q_pos_mavros", Q_pos_mavros(0, 0));
  param_loader.load_param("lateral/Q_pos_vio", Q_pos_vio(0, 0));
  param_loader.load_param("lateral/Q_pos_icp", Q_pos_icp(0, 0));
  param_loader.load_param("lateral/Q_pos_rtk", Q_pos_rtk(0, 0));

  param_loader.load_param("lateral/Q_vel_mavros", Q_vel_mavros(0, 0));
  param_loader.load_param("lateral/Q_vel_vio", Q_vel_vio(0, 0));
  param_loader.load_param("lateral/Q_vel_icp", Q_vel_icp(0, 0));
  param_loader.load_param("lateral/Q_vel_optflow", Q_vel_optflow(0, 0));

  param_loader.load_param("lateral/Q_tilt", Q_tilt(0, 0));

  param_loader.load_matrix_static("lateral/A", A2, lateral_n, lateral_n);

  if (lateral_m > 0) {
    param_loader.load_matrix_static("lateral/B", B2, lateral_n, lateral_m);
  }

  param_loader.load_matrix_static("lateral/R", R2, lateral_n, lateral_n);

  lateralKalmanX = new mrs_lib::Lkf(lateral_n, lateral_m, lateral_p, A2, B2, R2, Q_tilt, P_ang);
  lateralKalmanY = new mrs_lib::Lkf(lateral_n, lateral_m, lateral_p, A2, B2, R2, Q_tilt, P_ang);

  ROS_INFO("[Odometry]: Lateral Kalman prepared");
  //}

  // use differential gps
  param_loader.load_param("use_differential_gps", use_differential_gps);
  param_loader.load_param("publish_fused_odom", _publish_fused_odom);
  param_loader.load_param("pass_rtk_as_new_odom", pass_rtk_as_new_odom);
  param_loader.load_param("max_altitude_correction", max_altitude_correction_);

  if (pass_rtk_as_new_odom && !use_differential_gps) {
    ROS_ERROR("[Odometry]: cant have pass_rtk_as_new_odom TRUE when use_differential_gps FALSE");
    ros::shutdown();
  }

  ROS_INFO("[Odometry]: Differential GPS %s", use_differential_gps ? "enabled" : "disabled");

  trg_last_update          = ros::Time::now();
  odom_pixhawk_last_update = ros::Time::now();

  teraranger_enabled   = true;
  garmin_enabled       = true;
  rtk_altitude_enabled = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  last_drs_config.R_pos   = R2(0, 0);
  last_drs_config.R_vel   = R2(1, 1);
  last_drs_config.R_acc   = R2(2, 2);
  last_drs_config.R_acc_i = R2(3, 3);
  last_drs_config.R_acc_d = R2(4, 4);
  last_drs_config.R_tilt  = R2(5, 5);

  last_drs_config.Q_pos_mavros = Q_pos_mavros(0, 0);
  last_drs_config.Q_pos_rtk    = Q_pos_rtk(0, 0);
  last_drs_config.Q_pos_icp    = Q_pos_icp(0, 0);
  last_drs_config.Q_pos_vio    = Q_pos_vio(0, 0);

  last_drs_config.Q_vel_mavros  = Q_vel_mavros(0, 0);
  last_drs_config.Q_vel_optflow = Q_vel_optflow(0, 0);
  last_drs_config.Q_vel_icp     = Q_vel_icp(0, 0);
  last_drs_config.Q_vel_vio     = Q_vel_vio(0, 0);

  last_drs_config.Q_tilt = Q_tilt(0, 0);

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(last_drs_config);
  ReconfigureServer::CallbackType f = boost::bind(&Odometry::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                  = new mrs_lib::Profiler(nh_, "Odometry");
  routine_main_timer        = profiler->registerRoutine("main", rate_, 0.002);
  routine_odometry_callback = profiler->registerRoutine("callbackOdometry");
  routine_optflow_callback  = profiler->registerRoutine("callbackOptflowTwist");
  routine_vio_callback      = profiler->registerRoutine("callbackVioOdometry");
  routine_garmin_callback   = profiler->registerRoutine("callbackGarmin");
  routine_rtk_callback      = profiler->registerRoutine("callbackRtk");

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  //{ publishers
  // publisher for new odometry
  pub_odom_          = nh_.advertise<nav_msgs::Odometry>("new_odom_out", 1);
  pub_slow_odom_     = nh_.advertise<nav_msgs::Odometry>("slow_odom_out", 1);
  pub_odometry_diag_ = nh_.advertise<mrs_msgs::OdometryDiag>("odometry_diag_out", 1);
  pub_altitude_      = nh_.advertise<mrs_msgs::Float64Stamped>("altitude_out", 1);
  pub_max_altitude_  = nh_.advertise<mrs_msgs::Float64Stamped>("max_altitude_out", 1);
  pub_lkf_states_    = nh_.advertise<mrs_msgs::LkfStates>("lkf_states_out", 1);

  // republisher for rtk local
  pub_rtk_local = nh_.advertise<mrs_msgs::RtkGps>("rtk_local_out", 1);

  // republisher for rtk local odometry (e.g. for rviz)
  pub_rtk_local_odom = nh_.advertise<nav_msgs::Odometry>("rtk_local_odom_out", 1);

  // republisher for gps local odometry (e.g. for rviz)
  pub_gps_local_odom = nh_.advertise<nav_msgs::Odometry>("gps_local_odom_out", 1);

  // publisher for tf
  broadcaster_ = new tf::TransformBroadcaster();

  // publishers for roll pitch yaw orientations in local_origin frame
  pub_target_attitude_global_ = nh_.advertise<geometry_msgs::Vector3Stamped>("target_attitude_global", 1);
  pub_orientation_gt_         = nh_.advertise<geometry_msgs::Vector3Stamped>("orientation_gt", 1);
  pub_orientation_mavros_     = nh_.advertise<geometry_msgs::Vector3Stamped>("orientation_mavros", 1);
  //}

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  //{ subscribers
  // subsribe to target attitude
  sub_target_attitude_ = nh_.subscribe("target_attitude_in", 1, &Odometry::callbackTargetAttitude, this, ros::TransportHints().tcpNoDelay());

  // subscriber to mavros odometry
  sub_pixhawk_ = nh_.subscribe("pixhawk_odom_in", 1, &Odometry::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());

  // subscriber to optflow odometry
  sub_optflow_ = nh_.subscribe("optflow_in", 1, &Odometry::callbackOptflowTwist, this, ros::TransportHints().tcpNoDelay());

  // subscriber to optflow odometry
  sub_vio_ = nh_.subscribe("vio_in", 1, &Odometry::callbackVioOdometry, this, ros::TransportHints().tcpNoDelay());

  // subscriber to optflow odometry
  sub_optflow_stddev_ = nh_.subscribe("optflow_stddev_in", 1, &Odometry::callbackOptflowStddev, this, ros::TransportHints().tcpNoDelay());

  // subscriber for terarangers range
  sub_terarangerone_ = nh_.subscribe("teraranger_in", 1, &Odometry::callbackTeraranger, this, ros::TransportHints().tcpNoDelay());

  // subscriber for garmin range
  sub_garmin_ = nh_.subscribe("garmin_in", 1, &Odometry::callbackGarmin, this, ros::TransportHints().tcpNoDelay());

  // subscriber for differential gps
  if (use_differential_gps) {
    rtk_gps_sub_ = nh_.subscribe("rtk_gps_in", 1, &Odometry::callbackRtkGps, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for ground truth
  sub_ground_truth_ = nh_.subscribe("ground_truth_in", 1, &Odometry::callbackGroundTruth, this, ros::TransportHints().tcpNoDelay());

  // subscriber for icp relative odometry
  sub_icp_relative__ = nh_.subscribe("icp_relative_in", 1, &Odometry::callbackIcpRelative, this, ros::TransportHints().tcpNoDelay());

  // subscriber for icp global odometry
  sub_icp_global__ = nh_.subscribe("icp_absolute_in", 1, &Odometry::callbackIcpAbsolute, this, ros::TransportHints().tcpNoDelay());

  // subscribe for utm coordinates
  sub_global_position_ = nh_.subscribe("global_position_in", 1, &Odometry::callbackGlobalPosition, this, ros::TransportHints().tcpNoDelay());

  // subscribe for tracker status
  sub_tracker_status_ = nh_.subscribe("tracker_status_in", 1, &Odometry::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());

  // subscribe for mavros diagnostic
  sub_mavros_diagnostic_ = nh_.subscribe("mavros_diagnostic_in", 1, &Odometry::callbackMavrosDiag, this, ros::TransportHints().tcpNoDelay());
  //}

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  //{ services
  // subscribe for averaging service
  ser_averaging_ = nh_.advertiseService("average_current_position_in", &Odometry::callbackAveraging, this);

  // subscribe for reset kalman service
  ser_reset_lateral_kalman_ = nh_.advertiseService("reset_lateral_kalman_in", &Odometry::callbackResetKalman, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh_.advertiseService("toggle_garmin_in", &Odometry::callbackToggleGarmin, this);

  // toggling fusing of rtk altitude
  ser_toggle_rtk_altitude = nh_.advertiseService("toggle_rtk_altitude_in", &Odometry::callbackToggleRtkHeight, this);

  // toggling fusing of rtk position
  ser_toggle_rtk_pos_fusion = nh_.advertiseService("toggle_rtk_pos_fusion_in", &Odometry::callbackToggleRtkPosition, this);

  // toggling fusing of vio position
  ser_toggle_vio_pos_fusion = nh_.advertiseService("toggle_vio_pos_fusion_in", &Odometry::callbackToggleVioPosition, this);

  // toggling fusing of vio velocity
  ser_toggle_vio_vel_fusion = nh_.advertiseService("toggle_vio_vel_fusion_in", &Odometry::callbackToggleVioVelocity, this);

  // toggling fusing of icp position
  ser_toggle_icp_pos_fusion = nh_.advertiseService("toggle_icp_pos_fusion_in", &Odometry::callbackToggleIcpPosition, this);

  // toggling fusing of icp velocity
  ser_toggle_icp_vel_fusion = nh_.advertiseService("toggle_icp_vel_fusion_in", &Odometry::callbackToggleIcpVelocity, this);

  // toggling fusing of mavros velocity
  ser_toggle_mavros_pos_fusion = nh_.advertiseService("toggle_mavros_pos_fusion_in", &Odometry::callbackToggleMavrosPosition, this);

  // toggling fusing of mavros velocity
  ser_toggle_mavros_vel_fusion = nh_.advertiseService("toggle_mavros_vel_fusion_in", &Odometry::callbackToggleMavrosVelocity, this);

  // toggling fusing of mavros tilts
  ser_toggle_mavros_tilts_fusion = nh_.advertiseService("toggle_mavros_tilts_fusion_in", &Odometry::callbackToggleMavrosTilts, this);

  // toggling fusing of icp velocity
  ser_toggle_optflow_vel_fusion = nh_.advertiseService("toggle_optflow_vel_fusion_in", &Odometry::callbackToggleOptflowVelocity, this);

  // change fusion mode of odometry
  ser_change_odometry_mode = nh_.advertiseService("change_odometry_mode_in", &Odometry::callbackChangeOdometryMode, this);

  // change fusion mode of odometry
  ser_change_odometry_mode_string = nh_.advertiseService("change_odometry_mode_string_in", &Odometry::callbackChangeOdometryModeString, this);
  //}

  // subscriber for object altitude
  /* object_altitude_sub = nh_.subscribe("object_altitude", 1, &Odometry::callbackObjectHeight, this, ros::TransportHints().tcpNoDelay()); */
  // subscribe for object_altitude toggle service
  /* ser_object_altitude_ = nh_.advertiseService("toggle_object_altitude", &Odometry::callbackToggleObjectHeight, this); */

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer          = nh_.createTimer(ros::Rate(rate_), &Odometry::mainTimer, this);
  slow_odom_timer     = nh_.createTimer(ros::Rate(slow_odom_rate), &Odometry::slowOdomTimer, this);
  rtk_rate_timer      = nh_.createTimer(ros::Rate(1), &Odometry::rtkRateTimer, this);
  diag_timer          = nh_.createTimer(ros::Rate(diag_rate), &Odometry::diagTimer, this);
  lkf_states_timer    = nh_.createTimer(ros::Rate(lkf_states_rate), &Odometry::lkfStatesTimer, this);
  max_altitude_timer  = nh_.createTimer(ros::Rate(max_altitude_rate), &Odometry::maxAltitudeTimer, this);
  topic_watcher_timer = nh_.createTimer(ros::Rate(topic_watcher_rate), &Odometry::topicWatcherTimer, this);


  // Decide the initial odometry mode based on sensors availability
  /* mrs_msgs::OdometryMode target_mode; */
  /* if (_gps_available) { */
  /*   if (_rtk_available) { */
  /*     target_mode.mode = mrs_msgs::OdometryMode::RTK; */
  /*     ROS_WARN("[Odometry]: Launching odometry in RTK mode."); */
  /*   } else if (_optflow_available) { */
  /*     target_mode.mode = mrs_msgs::OdometryMode::OPTFLOWGPS; */
  /*     ROS_WARN("[Odometry]: Launching odometry in OPTFLOWGPS mode."); */
  /*   } else { */
  /*     target_mode.mode = mrs_msgs::OdometryMode::GPS; */
  /*     ROS_WARN("[Odometry]: Launching odometry in GPS mode."); */
  /*   } */
  /* } else if (_vio_available) { */
  /*   target_mode.mode = mrs_msgs::OdometryMode::VIO; */
  /*   ROS_WARN("[Odometry]: Launching odometry in VIO mode."); */
  /* } else if (_optflow_available) { */
  /*   target_mode.mode = mrs_msgs::OdometryMode::OPTFLOW; */
  /*   ROS_WARN("[Odometry]: Launching odometry in OPTFLOW mode."); */
  /* } else if (_lidar_available) { */
  /*   target_mode.mode = mrs_msgs::OdometryMode::ICP; */
  /*   ROS_WARN("[Odometry]: Launching odometry in ICP mode."); */
  /* } else { */
  /*   is_initialized = false; */
  /*   ROS_ERROR("[Odometry]: Neither GPS, OPTFLOW, RTK, LIDAR localization method is available. Cannot launch odometry. Shutting down."); */
  /*   ros::shutdown(); */
  /*   return; */
  /* } */

  // Check validity of takeoff mode
  ROS_INFO("[Odometry]: Requested %s mode for takeoff.", _odometry_mode_takeoff.name.c_str());
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::OPTFLOW && !_optflow_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. Optflow localization not available. Shutting down.",
              _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::GPS && !_gps_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. GPS localization not available. Shutting down.", _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::OPTFLOWGPS && !_optflow_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. Optflow localization not available. Shutting down.",
              _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::RTK && !_rtk_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. RTK localization not available. Shutting down.", _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::ICP && !_lidar_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. Lidar localization not available. Shutting down.",
              _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_odometry_mode_takeoff.mode == mrs_msgs::OdometryMode::VIO && !_vio_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. Visual odometry localization not available. Shutting down.",
              _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }

  bool success = setOdometryModeTo(_odometry_mode_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff odometry mode %s could not be set. Shutting down.", _odometry_mode_takeoff.name.c_str());
    ros::shutdown();
  }
  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[Odometry]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[Odometry]: initialized");
}

//}

/* //{ isUavFlying() */

bool Odometry::isUavFlying() {

  if (got_tracker_status) {

    if (std::string(tracker_status.tracker).compare("mrs_mav_manager/NullTracker") == STRING_EQUAL) {

      return false;
    } else {

      return true;
    }

  } else {

    return false;
  }
}

//}

/* //{ startAveraging() */

void Odometry::startAveraging() {

  ROS_INFO("[Odometry]: startAveraging() called");

  averaging             = true;
  gpos_average_x        = utm_position_x;
  gpos_average_y        = utm_position_y;
  averaging_got_samples = 1;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* //{ mainTimer() */

void Odometry::mainTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  // --------------------------------------------------------------
  // |              publish the new altitude message              |
  // --------------------------------------------------------------

  if (!got_odom || !got_range) {
    ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for altitude data from sensors - received? pixhawk: %s, ranger: %s", got_odom ? "TRUE" : "FALSE",
                      got_range ? "TRUE" : "FALSE");
    return;
  }

  got_altitude_sensors = true;

  mrs_msgs::Float64Stamped new_altitude;
  mutex_odom.lock();
  {
    new_altitude.header = odom_pixhawk.header;
    new_altitude.value  = odom_pixhawk.pose.pose.position.z;
  }
  mutex_odom.unlock();

  new_altitude.header.frame_id = "local_origin";

#if USE_RANGEFINDER == 1
  // update the altitude state
  mutex_main_altitude_kalman.lock();
  { new_altitude.value = main_altitude_kalman->getState(0); }
  mutex_main_altitude_kalman.unlock();
#endif

  if (fabs(main_altitude_kalman->getState(0) - failsafe_teraranger_kalman->getState(0)) > 0.5) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Main altitude: %2.2f, Failsafe altitude: %2.2f", main_altitude_kalman->getState(0),
                      failsafe_teraranger_kalman->getState(0));
  }
  try {

    pub_altitude_.publish(mrs_msgs::Float64StampedConstPtr(new mrs_msgs::Float64Stamped(new_altitude)));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_.getTopic().c_str());
  }

  // --------------------------------------------------------------
  // |              publish the new odometry message              |
  // --------------------------------------------------------------


  // if there are some data missing, return
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::RTK) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW mode.");
      mrs_msgs::OdometryMode optflow_mode;
      optflow_mode.mode = mrs_msgs::OdometryMode::OPTFLOW;
      setOdometryModeTo(optflow_mode);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_rtk) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s",
                        got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_rtk ? "TRUE" : "FALSE");
      return;
    }
  } else if (_odometry_mode.mode == mrs_msgs::OdometryMode::GPS) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW mode.");
      mrs_msgs::OdometryMode optflow_mode;
      optflow_mode.mode = mrs_msgs::OdometryMode::OPTFLOW;
      setOdometryModeTo(optflow_mode);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position)) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE");
      return;
    }
  } else if (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW mode.");
      mrs_msgs::OdometryMode optflow_mode;
      optflow_mode.mode = mrs_msgs::OdometryMode::OPTFLOW;
      setOdometryModeTo(optflow_mode);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_optflow) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                        got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
      return;
    }
  } else if (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW) {
    if (gps_reliable) {
      if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                          got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
        return;
      }
    } else {
      if (!got_odom || !got_range || !got_optflow) {
        ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, optflow: %s", got_odom ? "TRUE" : "FALSE",
                          got_range ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
        return;
      }
    }
  } else if (_odometry_mode.mode == mrs_msgs::OdometryMode::ICP) {
    if (!got_odom || !got_range || !got_icp) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, icp: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_icp ? "TRUE" : "FALSE");
      return;
    }
  } else if (_odometry_mode.mode == mrs_msgs::OdometryMode::VIO) {
    if (!got_odom || !got_range || !got_vio) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vio: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_vio ? "TRUE" : "FALSE");
      return;
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: OTHER odometry mode. Not checking sensors.");
  }

  got_lateral_sensors = true;

  routine_main_timer->start(event);

  // --------------------------------------------------------------
  // |           check if the odometry is still comming           |
  // --------------------------------------------------------------

  mutex_odom.lock();
  {
    if ((ros::Time::now() - odom_pixhawk_last_update).toSec() > 0.1) {

      ROS_ERROR("[Odometry]: mavros odometry has not come for > 0.1 s, interrupting");
      got_odom = false;
      mutex_odom.unlock();

      routine_main_timer->end();
      return;
    }
  }
  mutex_odom.unlock();

  if (!got_home_position_fix &&
      (_odometry_mode.mode == mrs_msgs::OdometryMode::RTK || _odometry_mode.mode == mrs_msgs::OdometryMode::GPS ||
       _odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS || (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW && gps_reliable))) {

    if (!averaging && !averaging_started) {

      startAveraging();
      averaging_started = true;

      routine_main_timer->end();
      return;

    } else if (done_averaging) {

      start_position_average_x = gpos_average_x;
      start_position_average_y = gpos_average_y;
      got_home_position_fix    = true;
      ROS_INFO("[Odometry]: Finished averaging of home position.");

      // when we have defined our home position, set local origin offset
      if (set_home_on_start) {

        mutex_odom.lock();
        {
          local_origin_offset_x = (start_position_average_x - odom_pixhawk.pose.pose.position.x) - home_utm_x;
          local_origin_offset_y = (start_position_average_y - odom_pixhawk.pose.pose.position.y) - home_utm_y;
        }
        mutex_odom.unlock();

        // when we have not define our home position, define it as our averaged home position
      } else {

        home_utm_x = start_position_average_x;
        home_utm_y = start_position_average_y;

        // just to be sure, set local_origin_offset to 0
        local_origin_offset_x = 0;
        local_origin_offset_y = 0;
      }
      init_pose_set = true;
    } else {

      routine_main_timer->end();
      return;
    }
  } else if (!init_pose_set && _odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW &&
             !gps_reliable) {  // Odometry mode without GPS -> use start position from config file
    ROS_INFO("[Odometry]: Setting initial position x: %f y: %f from config file.", init_pose_x, init_pose_y);
    local_origin_offset_x = init_pose_x;
    local_origin_offset_y = init_pose_y;
    got_home_position_fix = true;
    init_pose_set         = true;
  }

  nav_msgs::Odometry new_odom;
  mutex_odom.lock();
  { new_odom = odom_pixhawk; }
  mutex_odom.unlock();

  new_odom.header.frame_id = "local_origin";
  new_odom.child_frame_id  = std::string("fcu_") + uav_name;

  geometry_msgs::PoseStamped newPose;
  newPose.header = new_odom.header;

#if USE_RANGEFINDER == 1
  // update the altitude state
  mutex_main_altitude_kalman.lock();
  { new_odom.pose.pose.position.z = main_altitude_kalman->getState(0); }
  mutex_main_altitude_kalman.unlock();
#endif

  // if odometry has not been published yet, initialize lateralKF
  if (!odometry_published) {

    mutex_lateral_kalman_x.lock();
    {
      if (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW && !gps_reliable) {
        lateralKalmanX->setState(0, local_origin_offset_x);
      } else {
        lateralKalmanX->setState(0, new_odom.pose.pose.position.x + local_origin_offset_x);
      }
    }
    mutex_lateral_kalman_x.unlock();
    mutex_lateral_kalman_y.lock();
    {
      if (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW && !gps_reliable) {
        lateralKalmanY->setState(0, local_origin_offset_y);
      } else {
        lateralKalmanY->setState(0, new_odom.pose.pose.position.y + local_origin_offset_y);
      }
    }
    mutex_lateral_kalman_y.unlock();

    odometry_published = true;
  }

  // when using differential gps, get the position states from lateralKalman
  if (_publish_fused_odom) {

    mutex_lateral_kalman_x.lock();
    {
      new_odom.pose.pose.position.x = lateralKalmanX->getState(0);
      new_odom.twist.twist.linear.x = lateralKalmanX->getState(1);
    }
    mutex_lateral_kalman_x.unlock();

    mutex_lateral_kalman_y.lock();
    {
      new_odom.pose.pose.position.y = lateralKalmanY->getState(0);
      new_odom.twist.twist.linear.y = lateralKalmanY->getState(1);
    }
    mutex_lateral_kalman_y.unlock();

  } else {

    new_odom.pose.pose.position.x += local_origin_offset_x;
    new_odom.pose.pose.position.y += local_origin_offset_y;
  }

  // publish the odometry
  if (pass_rtk_as_new_odom) {
    mutex_rtk_local_odom.lock();
    { new_odom = rtk_local_odom; }
    mutex_rtk_local_odom.unlock();
  }

  mutex_shared_odometry.lock();
  { shared_odom = new_odom; }
  mutex_shared_odometry.unlock();

  try {
    pub_odom_.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(new_odom)));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_.getTopic().c_str());
  }


  // publish TF
  geometry_msgs::Quaternion orientation = new_odom.pose.pose.orientation;
  geometry_msgs::Point      position    = new_odom.pose.pose.position;
  try {
    broadcaster_->sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w), tf::Vector3(position.x, position.y, position.z)),
        new_odom.header.stamp, "local_origin", std::string("fcu_") + uav_name));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing TF.");
  }

  routine_main_timer->end();
}

//}

/* //{ slowOdomTimer() */

void Odometry::slowOdomTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  nav_msgs::Odometry slow_odom;

  mutex_shared_odometry.lock();
  { slow_odom = shared_odom; }
  mutex_shared_odometry.unlock();

  try {
    pub_slow_odom_.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(slow_odom)));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_slow_odom_.getTopic().c_str());
  }
}

//}

/* //{ diagTimer() */

void Odometry::diagTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_msgs::OdometryDiag odometry_diag;

  odometry_diag.header.stamp = ros::Time::now();

  odometry_diag.odometry_mode = _odometry_mode;

  odometry_diag.max_altitude      = max_altitude;
  odometry_diag.gps_reliable      = gps_reliable;
  odometry_diag.gps_available     = _gps_available;
  odometry_diag.optflow_available = _optflow_available;
  odometry_diag.rtk_available     = _rtk_available;
  odometry_diag.lidar_available   = _lidar_available;

  odometry_diag.optflow_vel.model_state.state = mrs_msgs::ModelState::VELOCITY;
  odometry_diag.optflow_vel.is_fusing         = _fuse_optflow_velocity;
  odometry_diag.optflow_vel.covariance        = Q_vel_optflow(0, 0);

  odometry_diag.mavros_tilts.model_state.state = mrs_msgs::ModelState::TILT;
  odometry_diag.mavros_tilts.is_fusing         = _fuse_mavros_tilts;
  odometry_diag.mavros_tilts.covariance        = Q_tilt(0, 0);

  odometry_diag.mavros_vel.model_state.state = mrs_msgs::ModelState::VELOCITY;
  odometry_diag.mavros_vel.is_fusing         = _fuse_mavros_velocity;
  odometry_diag.mavros_vel.covariance        = Q_vel_mavros(0, 0);

  odometry_diag.mavros_pos.model_state.state = mrs_msgs::ModelState::POSITION;
  odometry_diag.mavros_pos.is_fusing         = _fuse_mavros_position;
  odometry_diag.mavros_pos.covariance        = Q_pos_mavros(0, 0);

  odometry_diag.rtk_pos.model_state.state = mrs_msgs::ModelState::POSITION;
  odometry_diag.rtk_pos.is_fusing         = _fuse_rtk_position;
  odometry_diag.rtk_pos.covariance        = Q_pos_rtk(0, 0);

  odometry_diag.vio_vel.model_state.state = mrs_msgs::ModelState::VELOCITY;
  odometry_diag.vio_vel.is_fusing         = _fuse_vio_velocity;
  odometry_diag.vio_vel.covariance        = Q_vel_vio(0, 0);

  odometry_diag.vio_pos.model_state.state = mrs_msgs::ModelState::POSITION;
  odometry_diag.vio_pos.is_fusing         = _fuse_vio_position;
  odometry_diag.vio_pos.covariance        = Q_pos_vio(0, 0);

  odometry_diag.icp_vel.model_state.state = mrs_msgs::ModelState::VELOCITY;
  odometry_diag.icp_vel.is_fusing         = _fuse_icp_velocity;
  odometry_diag.icp_vel.covariance        = Q_vel_icp(0, 0);

  odometry_diag.icp_pos.model_state.state = mrs_msgs::ModelState::POSITION;
  odometry_diag.icp_pos.is_fusing         = _fuse_icp_position;
  odometry_diag.icp_pos.covariance        = Q_pos_icp(0, 0);

  try {
    pub_odometry_diag_.publish(odometry_diag);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odometry_diag_.getTopic().c_str());
  }
}

//}

/* //{ lkfStatesTimer() */

void Odometry::lkfStatesTimer(const ros::TimerEvent &event) {

  Eigen::VectorXd states_vec;
  Eigen::MatrixXd cov_mat;

  // get states and covariances from lateral kalman
  mutex_lateral_kalman_x.lock();
  {
    states_vec = lateralKalmanY->getStates();
    cov_mat    = lateralKalmanY->getCovariance();
  }
  mutex_lateral_kalman_x.unlock();

  // convert eigen matrix to std::vector
  std::vector<double> cov_vec(cov_mat.data(), cov_mat.data() + cov_mat.rows() * cov_mat.cols());

  Eigen::EigenSolver<Eigen::MatrixXd> es(cov_mat);

  // fill the message
  mrs_msgs::LkfStates lkf_states;
  for (int i = 0; i < cov_mat.rows(); ++i) {
    lkf_states.eigenvalues[i] = (es.eigenvalues().col(0)[i].real());
  }

  for (int i = 0; i < cov_mat.rows(); i++) {
    lkf_states.covariance[i] = cov_mat(i, i);
  }

  lkf_states.header.stamp = ros::Time::now();
  lkf_states.pos          = states_vec(0);
  lkf_states.vel          = states_vec(1);
  lkf_states.acc          = states_vec(2);
  lkf_states.acc_i        = states_vec(3);
  lkf_states.acc_d        = states_vec(4);
  lkf_states.tilt         = states_vec(5);

  try {
    pub_lkf_states_.publish(lkf_states);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_lkf_states_.getTopic().c_str());
  }
}

//}

/* //{ maxAltitudeTimer() */

void Odometry::maxAltitudeTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_msgs::Float64Stamped max_altitude_msg;
  max_altitude_msg.header.frame_id = "local_origin";
  max_altitude_msg.header.stamp    = ros::Time::now();

  max_altitude_msg.value = max_altitude;

  try {
    pub_max_altitude_.publish(max_altitude_msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_max_altitude_.getTopic().c_str());
  }
}

//}

/* //{ rtkRateTimer() */

void Odometry::rtkRateTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

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

  if (!is_initialized)
    return;

  ros::Duration interval;

  // pixhawk odometry
  interval = ros::Time::now() - odom_pixhawk_last_update;
  if (got_odom && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Pixhawk odometry not received for %f seconds.", interval.toSec());
    got_odom = false;
  }

  // optflow velocities
  interval = ros::Time::now() - optflow_twist_last_update;
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOW && got_optflow && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Optflow twist not received for %f seconds.", interval.toSec());
    got_optflow = false;
  }

  //  target attitude
  interval = ros::Time::now() - target_attitude_last_update;
  if (got_target_attitude && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Target attitude not received for %f seconds.", interval.toSec());
    got_target_attitude = false;
  }

  //  vio odometry
  interval = ros::Time::now() - odom_vio_last_update;
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::VIO && got_vio && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: VIO odometry not received for %f seconds.", interval.toSec());
    got_vio = false;
  }

  //  icp velocities
  interval = ros::Time::now() - icp_odom_last_update;
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::ICP && got_icp && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP velocities not received for %f seconds.", interval.toSec());
    got_icp = false;
  }

  //  icp position
  interval = ros::Time::now() - icp_global_odom_last_update;
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::ICP && got_icp_global && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP position not received for %f seconds.", interval.toSec());
    got_icp_global = false;
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackTargetAttitude() */
void Odometry::callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg) {

  if (got_target_attitude) {

    mutex_target_attitude.lock();
    {
      target_attitude_previous     = target_attitude;
      target_attitude              = *msg;
      target_attitude.header.stamp = ros::Time::now();
    }
    mutex_target_attitude.unlock();

  } else {

    mutex_target_attitude.lock();
    {
      target_attitude              = *msg;
      target_attitude.header.stamp = ros::Time::now();
      target_attitude_previous     = target_attitude;
    }
    mutex_target_attitude.unlock();

    got_target_attitude = true;
    return;
  }

  target_attitude_last_update = ros::Time::now();

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_target_attitude.lock();
  { interval2 = target_attitude.header.stamp - target_attitude_previous.header.stamp; }
  mutex_target_attitude.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("Attitude messages came within 0.001 s");
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  // rotations in inertial frame
  double rot_x, rot_y, rot_z;
  mutex_target_attitude.lock();
  { getGlobalRot(target_attitude.orientation, rot_x, rot_y, rot_z); }
  // For model testing
  /* { getGlobalRot(ground_truth.pose.pose.orientation, rot_x, rot_y, rot_z); } */
  mutex_target_attitude.unlock();
  target_attitude_global.header   = target_attitude.header;
  target_attitude_global.vector.x = rot_x;
  target_attitude_global.vector.y = rot_y;
  target_attitude_global.vector.z = rot_z;
  pub_target_attitude_global_.publish(target_attitude_global);

  if (!std::isfinite(rot_x)) {
    rot_x = 0;
    ROS_ERROR("NaN detected in Mavros variable \"rot_x\", setting it to 0 and returning!!!");
    return;
  } else if (rot_x > 1.57) {
    rot_x = 1.57;
  } else if (rot_x < -1.57) {
    rot_x = -1.57;
  }

  if (!std::isfinite(rot_y)) {
    rot_y = 0;
    ROS_ERROR("NaN detected in Mavros variable \"rot_y\", setting it to 0 and returning!!!");
    return;
  } else if (rot_y > 1.57) {
    rot_y = 1.57;
  } else if (rot_y < -1.57) {
    rot_y = -1.57;
  }

  double dt = interval2.toSec();

  if (!std::isfinite(dt)) {
    dt = 0;
    ROS_ERROR("NaN detected in Mavros variable \"dt\", setting it to 0 and returning!!!");
    return;
  } else if (dt > 1) {
    ROS_ERROR("Mavros variable \"dt\" > 1, setting it to 1 and returning!!!");
    dt = 1;
    return;
  } else if (dt < -1) {
    ROS_ERROR("Mavros variable \"dt\" < -1, setting it to -1 and returning!!!");
    dt = -1;
    return;
  }

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing target attitude. Waiting for other sensors.");
    return;
  }

  // x prediction
  mutex_lateral_kalman_x.lock();
  {
    // set the input vector
    Eigen::VectorXd input;
    input = Eigen::VectorXd::Zero(lateral_m);
    input << rot_y;

    Eigen::MatrixXd newA = lateralKalmanX->getA();
    newA(0, 1)           = dt;
    newA(1, 2)           = dt;
    lateralKalmanX->setA(newA);

    lateralKalmanX->setInput(input);

    /* Eigen::MatrixXd cov_mat = lateralKalmanX->getCovariance(); */
    /* ROS_WARN("[Odometry]: LKF Covariance before prediction:"); */
    /* for (int i = 0; i < cov_mat.rows(); i++) { */
    /*   std::cout << " " << cov_mat(i, i); */
    /* } */
    /* std::cout << std::endl; */
    lateralKalmanX->iterateWithoutCorrection();
  }
  mutex_lateral_kalman_x.unlock();

  // y prediction
  mutex_lateral_kalman_y.lock();
  {
    // set the input vector
    Eigen::VectorXd input;
    input = Eigen::VectorXd::Zero(lateral_m);
    input << -rot_x;

    Eigen::MatrixXd newA = lateralKalmanY->getA();
    newA(0, 1)           = dt;
    newA(1, 2)           = dt;
    lateralKalmanY->setA(newA);

    lateralKalmanY->setInput(input);

    lateralKalmanY->iterateWithoutCorrection();
  }
  mutex_lateral_kalman_y.unlock();

  ROS_INFO_ONCE("[Odometry]: Iterating lateral Kalman filter with target_attitude at input");
}
//}

/* //{ callbackGlobalPosition() */

void Odometry::callbackGlobalPosition(const sensor_msgs::NavSatFixConstPtr &msg) {

  if (!is_initialized)
    return;

  double out_x;
  double out_y;

  mrs_lib::UTM(msg->latitude, msg->longitude, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    ROS_ERROR("[Odometry]: NaN detected in UTM variable \"out_x\"!!!");
    return;
  }

  if (!std::isfinite(out_y)) {
    ROS_ERROR("[Odometry]: NaN detected in UTM variable \"out_y\"!!!");
    return;
  }

  utm_position_x = out_x;
  utm_position_y = out_y;

  got_global_position = true;

  nav_msgs::Odometry gps_local_odom;
  gps_local_odom.header          = msg->header;
  gps_local_odom.header.frame_id = "local_origin";

  // | ------------- offset the gps to local_origin ------------- |
  gps_local_odom.pose.pose.position.x = utm_position_x - home_utm_x;
  gps_local_odom.pose.pose.position.y = utm_position_y - home_utm_y;

  mutex_odom.lock();
  {
    gps_local_odom.pose.pose.position.z = odom_pixhawk.pose.pose.position.z;
    gps_local_odom.twist                = odom_pixhawk.twist;
  }
  mutex_odom.unlock();


  // | ------------- publish the gps local odometry ------------- |
  mutex_gps_local_odom.lock();
  {
    try {
      pub_gps_local_odom.publish(gps_local_odom);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_gps_local_odom.getTopic().c_str());
    }
  }
  mutex_gps_local_odom.unlock();

  if (averaging) {

    gpos_average_x = gpos_average_x + (utm_position_x - gpos_average_x) / (averaging_num_samples + 1);
    gpos_average_y = gpos_average_y + (utm_position_y - gpos_average_y) / (averaging_num_samples + 1);

    // stop averaging
    if (averaging_got_samples++ >= averaging_num_samples) {

      averaging = false;
      ROS_INFO("[Odometry]: Averaged current position is: %.5f %.5f", gpos_average_x, gpos_average_y);
      done_averaging = true;

    } else {

      ROS_INFO("[Odometry]: Sample %d of %d of GPS: %.5f %.5f", averaging_got_samples, averaging_num_samples, gpos_average_x, gpos_average_y);
    }
  }
}

//}

/* //{ callbackTeraranger() */

void Odometry::callbackTeraranger(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized)
    return;

  range_terarangerone_ = *msg;

  if (!got_odom) {

    return;
  }

  // getting roll, pitch, yaw
  double                    roll, pitch, yaw;
  geometry_msgs::Quaternion quat;
  mutex_odom.lock();
  { quat = odom_pixhawk.pose.pose.orientation; }
  mutex_odom.unlock();
  tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

  double measurement = 0;
  // compensate for tilting of the sensor
  measurement = range_terarangerone_.range * cos(roll) * cos(pitch) + trg_z_offset_;

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Teraranger variable \"measurement\" (teraranger)!!!");
    return;
  }

  got_range = true;

  // deside on measurement's covariance
  Eigen::MatrixXd mesCov;
  mesCov = Eigen::MatrixXd::Zero(altitude_p, altitude_p);

  //////////////////// Filter out terarange measurement ////////////////////
  mutex_failsafe_altitude_kalman.lock();
  {
    // filter out outliers in the case of hight alttide
    if (failsafe_teraranger_kalman->getState(0) > 8 && measurement < 6) {

      measurement = 0;
    }

    // teraranger filtration
    if (isUavFlying()) {

      ros::Duration interval;
      interval    = ros::Time::now() - range_terarangerone_.header.stamp;
      measurement = terarangerFilter->getValue(measurement, interval);
    }
  }
  mutex_failsafe_altitude_kalman.unlock();

  {  // Update variance of Kalman measurement
    // set the default covariance
    mesCov << Q1(0, 0);

    if (measurement <= 0 || measurement > trg_max_valid_altitude) {
      // enlarge the measurement covariance
      Q1(0, 0) = Q1(0, 0) + TrgQChangeRate;

    } else {
      // ensmall the measurement covariance
      Q1(0, 0) = Q1(0, 0) - TrgQChangeRate;
    }

    // saturate the measurement covariance
    if (Q1(0, 0) > TrgMaxQ) {
      Q1(0, 0) = TrgMaxQ;
    } else if (Q1(0, 0) < TrgMinQ) {
      Q1(0, 0) = TrgMinQ;
    }
  }

  //////////////////// Fuse failsafe altitude kalman ////////////////////
  // fuse the measurement only when terarangerFilter produced positive value, i.e. feasible value
  if (measurement > 0.2) {

    mutex_failsafe_altitude_kalman.lock();
    {
      // create a correction value
      double correction = 0;
      correction        = measurement - failsafe_teraranger_kalman->getState(0);

      // saturate the correction
      if (!std::isfinite(correction)) {
        correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in Teraranger variable \"correction\", setting it to 0!!!");
      }

      // set the measurement vector
      Eigen::VectorXd mes(1);
      mes << failsafe_teraranger_kalman->getState(0) + correction;

      failsafe_teraranger_kalman->setMeasurement(mes, mesCov);
      failsafe_teraranger_kalman->doCorrection();
    }
    mutex_failsafe_altitude_kalman.unlock();
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (teraranger_enabled) {

    // fuse the measurement only when terarangerFilter produced positive value, i.e. feasible value
    if (measurement > 0.2) {

      // create a correction value
      double correction;
      correction = measurement - main_altitude_kalman->getState(0);

      // saturate the correction
      if (!std::isfinite(correction)) {
        correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in Teraranger variable \"correction\", setting it to 0!!!");
      } else if (correction > max_altitude_correction_) {
        correction = max_altitude_correction_;
      } else if (correction < -max_altitude_correction_) {
        correction = -max_altitude_correction_;
      }

      // set the measurement vector
      Eigen::VectorXd mes(1);
      mes << main_altitude_kalman->getState(0) + correction;

      mutex_main_altitude_kalman.lock();
      {
        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_WARN_ONCE("[Odometry]: Fusing Teraranger");
    }
  }
}

//}

/* //{ callbackGarmin() */

void Odometry::callbackGarmin(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized)
    return;

  range_garmin_ = *msg;

  if (!got_odom) {

    return;
  }

  routine_garmin_callback->start();

  // getting roll, pitch, yaw
  double                    roll, pitch, yaw;
  geometry_msgs::Quaternion quat;
  mutex_odom.lock();
  { quat = odom_pixhawk.pose.pose.orientation; }
  mutex_odom.unlock();
  tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

  double measurement = 0;
  // compensate for tilting of the sensor
  measurement = range_garmin_.range * cos(roll) * cos(pitch) + garmin_z_offset_;

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Garmin variable \"measurement\" (garmin)!!!");

    routine_garmin_callback->end();
    return;
  }

  got_range = true;

  // deside on measurement's covariance
  Eigen::MatrixXd mesCov;
  mesCov = Eigen::MatrixXd::Zero(altitude_p, altitude_p);

  //////////////////// Filter out garmin measurement ////////////////////
  // garmin filtration
  if (isUavFlying()) {

    ros::Duration interval;
    interval    = ros::Time::now() - range_garmin_.header.stamp;
    measurement = garminFilter->getValue(measurement, interval);
  }

  {  // Update variance of Kalman measurement
    // set the default covariance
    mesCov << Q1(0, 0);

    if (measurement <= 0 || measurement > garmin_max_valid_altitude) {
      // enlarge the measurement covariance
      Q1(0, 0) = Q1(0, 0) + GarminQChangeRate;

    } else {
      // ensmall the measurement covariance
      Q1(0, 0) = Q1(0, 0) - GarminQChangeRate;
    }

    // saturate the measurement covariance
    if (Q1(0, 0) > GarminMaxQ) {
      Q1(0, 0) = GarminMaxQ;
    } else if (Q1(0, 0) < GarminMinQ) {
      Q1(0, 0) = GarminMinQ;
    }
  }

  //////////////////// Fuse main altitude kalman ////////////////////
  if (garmin_enabled) {

    // fuse the measurement only when garminFilter produced positive value, i.e. feasible value
    if (measurement > 0.01) {

      // create a correction value
      double correction;
      correction = measurement - main_altitude_kalman->getState(0);

      // saturate the correction
      if (!std::isfinite(correction)) {
        correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in Garmin variable \"correction\", setting it to 0!!!");
      } else if (correction > max_altitude_correction_) {
        correction = max_altitude_correction_;
      } else if (correction < -max_altitude_correction_) {
        correction = -max_altitude_correction_;
      }

      // set the measurement vector
      Eigen::VectorXd mes(1);
      mes << main_altitude_kalman->getState(0) + correction;

      mutex_main_altitude_kalman.lock();
      {
        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_WARN_ONCE("[Odometry]: fusing Garmin rangefinder");
    }
  }

  routine_garmin_callback->end();
}

//}

/* //{ callbackObjectHeight() */

/*
void Odometry::callbackObjectHeight(const object_detection::ObjectWithTypeConstPtr &msg) {

  mutex_object_altitude.lock();
  {
    object_altitude = *msg;
  }
  mutex_object_altitude.unlock();

  got_object_altitude = true;

  ROS_INFO_THROTTLE(1, "[Odometry]: Receiving object altitude");

  // ALTITUDE KALMAN FILTER
  // deside on measurement's covariance
  Eigen::MatrixXd mesCov;
  mesCov = Eigen::MatrixXd::Zero(altitude_p, altitude_p);

  // getting roll, pitch, yaw
  double roll, pitch, yaw;
  geometry_msgs::Quaternion quat;
  mutex_odom.lock();
  {
    quat = odom_pixhawk.pose.pose.orientation;
  }
  mutex_odom.unlock();
  tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);

  double measurement = 0;
  // compensate for tilting of the sensor

  if (msg->type == 2) { // moving object
    measurement = msg->z*cos(roll)*cos(pitch) + mobius_z_offset_ + dynamic_object_height_;
  } else { // static object
    measurement = msg->z*cos(roll)*cos(pitch) + mobius_z_offset_ + static_object_height_;
  }

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"measurement\" (object)!!!");
    return;
  }

  ros::Duration interval;

  // object altitude filtration
  if (isUavFlying()) {

    interval = ros::Time::now() - object_altitude.stamp;
    mutex_main_altitude_kalman.lock();
    {
      measurement = objectAltitudeFilter->getValue(measurement, main_altitude_kalman->getState(0), interval);
    }
    mutex_main_altitude_kalman.unlock();
  }

  mesCov << objectQ;

  //////////////////// Fuse object altitude ////////////////////
  if (object_altitude_enabled) {

    // if rtk integral is too above failsafe kalman, switch to fusing teraranger
    if ((measurement - failsafe_teraranger_kalman->getState(0)) > object_altitude_max_down_difference_) {

      object_altitude_enabled = false;
      teraranger_enabled = true;
      ROS_ERROR("[Odometry]: Object altitude is above failsafe kalman by more than %2.2f m!", object_altitude_max_down_difference_);
      ROS_ERROR("[Odometry]: Switching back to fusing teraranger!");
      return;
    }

    // if object altitude is too above failsafe kalman, switch to fusing teraranger
    if (fabs(failsafe_teraranger_kalman->getState(0) - measurement) > object_altitude_max_abs_difference_) {

      object_altitude_enabled = false;
      teraranger_enabled = true;
      ROS_ERROR("[Odometry]: Object altitude differs from Failsafe kalman by more than %2.2f m!", rtk_max_abs_difference_);
      ROS_ERROR("[Odometry]: Switching back to fusing teraranger!");
      return;
    }

    if (measurement > 0.2) {

      mutex_main_altitude_kalman.lock();
      {

        double correction = 0;

        correction = measurement - main_altitude_kalman->getState(0);

        // saturate the correction
        if (!std::isfinite(correction)) {
          correction = 0;
          ROS_ERROR("[Odometry]: NaN detected in variable \"correction\", setting it to 0!!!");
        } else if (correction > max_altitude_correction_) {
          correction = max_altitude_correction_;
        } else if (correction < -max_altitude_correction_) {
          correction = -max_altitude_correction_;
        }

        // set the measurement vector
        Eigen::VectorXd mes(1);
        mes << main_altitude_kalman->getState(0) + correction;

        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_INFO_THROTTLE(1, "[Odometry]: Fusing object altitude");
    }
  }
}
*/

//}

/* //{ callbackAveraging() */

bool Odometry::callbackAveraging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  startAveraging();

  res.success = true;
  res.message = "Started averaging";

  return true;
}

//}

/* //{ callbackRtkGps() */

void Odometry::callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_msgs::RtkGps rtk_utm;

  if (msg->header.frame_id.compare("gps") == STRING_EQUAL) {

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
    rtk_utm.header.frame_id      = "utm";
    rtk_utm.pose.pose.position.z = msg->gps.altitude;
    // | ----------------------- #TODO fixme ---------------------- |
    /* rtk_utm.pose.covariance      = msg->gps.covariance; */
    rtk_utm.fix_type = msg->fix_type;
    rtk_utm.status   = msg->status;

  } else if (msg->header.frame_id.compare("utm") == STRING_EQUAL) {

    rtk_utm = *msg;

  } else {

    ROS_INFO_THROTTLE(1.0, "[Odometry]: RTK message has unknown frame_id: '%s'", msg->header.frame_id.c_str());
  }

  mutex_rtk.lock();
  {
    rtk_local_previous = rtk_local;
    rtk_local          = rtk_utm;

    if (++got_rtk_counter > 2) {

      got_rtk         = true;
      rtk_last_update = ros::Time::now();
    }
  }
  mutex_rtk.unlock();

  if (!got_odom || !got_rtk) {

    return;
  }

  routine_rtk_callback->start();

  // | ------------- offset the rtk to local_origin ------------- |
  rtk_local.pose.pose.position.x -= home_utm_x;
  rtk_local.pose.pose.position.y -= home_utm_y;

  rtk_local.header.frame_id = "local_origin";

  // | ------------------ publish the rtk local ----------------- |
  mrs_msgs::RtkGps rtk_local_out = rtk_local;

  try {
    pub_rtk_local.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_local_out)));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local.getTopic().c_str());
  }
  // | ------------- publish the rtk local odometry ------------- |
  mutex_rtk_local_odom.lock();
  {
    rtk_local_odom.header = rtk_local.header;
    rtk_local_odom.pose   = rtk_local.pose;
    rtk_local_odom.twist  = rtk_local.twist;

    try {
      pub_rtk_local_odom.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(rtk_local_odom)));
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_rtk_local_odom.getTopic().c_str());
    }
  }
  mutex_rtk_local_odom.unlock();

  // | ----------------------------- --------------------------- |

  // check whether we have rtk fix
  got_rtk_fix = (rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FLOAT || rtk_local.fix_type.fix_type == mrs_msgs::RtkFixType::RTK_FIX) ? true : false;

  // continue to lateral and altitude fusion only when we got a fix
  if (!got_rtk_fix) {

    routine_rtk_callback->end();
    return;
  }

  if (!std::isfinite(rtk_local.pose.pose.position.x) || !std::isfinite(rtk_local.pose.pose.position.y)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local.pose.pose.position.x\" or \"rtk_local.pose.pose.position.y\" (rtk)!!!");

    routine_rtk_callback->end();
    return;
  }

  if (_fuse_rtk_position) {


    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing RTK position. Waiting for other sensors.");
      return;
    }

    // fuse rtk gps x position
    mutex_lateral_kalman_x.lock();
    {
      // fill the measurement vector
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

      double x_correction = 0;

      mutex_rtk.lock();
      { x_correction = rtk_local.pose.pose.position.x - lateralKalmanX->getState(0); }
      mutex_rtk.unlock();

      // saturate the x_correction
      if (!std::isfinite(x_correction)) {
        x_correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in RTK variable \"x_correction\", setting it to 0!!!");
      } else if (x_correction > max_pos_correction) {
        x_correction = max_pos_correction;
      } else if (x_correction < -max_pos_correction) {
        x_correction = -max_pos_correction;
      }

      mes_lat << lateralKalmanX->getState(0) + x_correction;  // apply offsetting from desired center

      lateralKalmanX->setP(P_pos);

      // set the measurement to kalman filter
      lateralKalmanX->setMeasurement(mes_lat, Q_pos_rtk);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // fuse rtk gps y position
    mutex_lateral_kalman_y.lock();
    {
      // fill the measurement vector
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

      double y_correction = 0;

      mutex_rtk.lock();
      { y_correction = rtk_local.pose.pose.position.y - lateralKalmanY->getState(0); }
      mutex_rtk.unlock();

      // saturate the y_correction
      if (!std::isfinite(y_correction)) {
        y_correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in RTK variable \"y_correction\", setting it to 0!!!");
      } else if (y_correction > max_pos_correction) {
        y_correction = max_pos_correction;
      } else if (y_correction < -max_pos_correction) {
        y_correction = -max_pos_correction;
      }

      mes_lat << lateralKalmanY->getState(0) + y_correction;  // apply offsetting from desired center

      lateralKalmanY->setP(P_pos);
      // set the measurement to kalman filter
      lateralKalmanY->setMeasurement(mes_lat, Q_pos_rtk);

      lateralKalmanY->doCorrection();

      ROS_WARN_ONCE("[Odometry]: fusing RTK GPS");
    }
    mutex_lateral_kalman_y.unlock();
  }

  if (rtk_altitude_enabled) {

    if (!got_rtk_fix) {

      rtk_altitude_enabled = false;
      teraranger_enabled   = true;
      garmin_enabled       = true;
      ROS_WARN("[Odometry]: We lost RTK fix, switching back to fusing teraranger and garmin.");

      routine_rtk_callback->end();
      return;
    }

    // ALTITUDE KALMAN FILTER
    // deside on measurement's covariance
    Eigen::MatrixXd mesCov;
    mesCov = Eigen::MatrixXd::Zero(altitude_p, altitude_p);

    if (!std::isfinite(rtk_local.pose.pose.position.z)) {

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in RTK variable \"rtk_local.position.position.z\" (rtk_altitude)!!!");

      routine_rtk_callback->end();
      return;
    }

    //////////////////// update rtk integral ////////////////////
    // compute the difference
    double difference = rtk_local.pose.pose.position.z - rtk_local_previous.pose.pose.position.z;

    rtk_altitude_integral += difference;

    //////////////////// Compare integral against failsafe kalman ////////////////////
    if (failsafe_teraranger_kalman->getState(0) < 5) {  // only when near to the ground

      // if rtk integral is too above failsafe kalman, switch to fusing teraranger
      if ((rtk_altitude_integral - failsafe_teraranger_kalman->getState(0)) > rtk_max_down_difference_) {

        rtk_altitude_enabled = false;
        teraranger_enabled   = true;
        ROS_ERROR("[Odometry]: RTK kalman is above failsafe kalman by more than %2.2f m!", rtk_max_down_difference_);
        ROS_ERROR("[Odometry]: Switching back to fusing teraranger!");

        routine_rtk_callback->end();
        return;
      }
    }

    // if rtk integral is too above failsafe kalman, switch to fusing teraranger
    if (fabs(failsafe_teraranger_kalman->getState(0) - rtk_altitude_integral) > rtk_max_abs_difference_) {

      rtk_altitude_enabled = false;
      teraranger_enabled   = true;
      ROS_ERROR("[Odometry]: RTK kalman differs from Failsafe kalman by more than %2.2f m!", rtk_max_abs_difference_);
      ROS_ERROR("[Odometry]: Switching back to fusing teraranger!");

      routine_rtk_callback->end();
      return;
    }

    // set the measurement vector
    Eigen::VectorXd mes(1);
    mes << rtk_altitude_integral;

    // set variance of gps measurement
    mesCov << rtkQ;

    mutex_main_altitude_kalman.lock();
    {
      main_altitude_kalman->setMeasurement(mes, mesCov);
      main_altitude_kalman->doCorrection();
    }
    mutex_main_altitude_kalman.unlock();

    ROS_WARN_ONCE("[Odometry]: Fusing rtk altitude");
  }

  routine_rtk_callback->end();
}

//}

/* //{ callbackIcpRelative() */

void Odometry::callbackIcpRelative(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  icp_odom_last_update = ros::Time::now();

  if (got_icp) {

    mutex_icp.lock();
    {
      icp_odom_previous = icp_odom;
      icp_odom          = *msg;
    }
    mutex_icp.unlock();

  } else {

    mutex_icp.lock();
    {
      icp_odom_previous = *msg;
      icp_odom          = *msg;
    }
    mutex_icp.unlock();

    got_icp = true;
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  routine_icp_callback->start();

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_icp.lock();
  { interval2 = icp_odom.header.stamp - icp_odom.header.stamp; }
  mutex_icp.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: ICP relative messages came within %1.8f s", interval2.toSec());

    routine_icp_callback->end();
    return;
  }


  //////////////////// Fuse Lateral Kalman ////////////////////

  if (_fuse_icp_velocity) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP velocity. Waiting for other sensors.");
      return;
    }

    // set the measurement vector
    Eigen::VectorXd mes_icp = Eigen::VectorXd::Zero(lateral_p);
    // fuse lateral kalman x velocity
    mutex_icp.lock();
    { mes_icp << icp_odom.twist.twist.linear.x; }
    mutex_icp.unlock();

    mutex_lateral_kalman_x.lock();
    {
      lateralKalmanX->setP(P_vel);

      lateralKalmanX->setMeasurement(mes_icp, Q_vel_icp);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // fuse lateral kalman y velocity
    mutex_icp.lock();
    { mes_icp << icp_odom.twist.twist.linear.y; }
    mutex_icp.unlock();

    mutex_lateral_kalman_y.lock();
    {
      lateralKalmanY->setP(P_vel);

      lateralKalmanY->setMeasurement(mes_icp, Q_vel_icp);

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    ROS_WARN_ONCE("[Odometry]: fusing ICP velocity");
  }
}
//}

/* //{ callbackIcpAbsolute() */

void Odometry::callbackIcpAbsolute(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  icp_global_odom_last_update = ros::Time::now();

  if (got_icp_global) {

    mutex_icp_global.lock();
    {
      icp_global_odom_previous = icp_global_odom;
      icp_global_odom          = *msg;
    }
    mutex_icp_global.unlock();

  } else {

    mutex_icp_global.lock();
    {
      icp_global_odom_previous = *msg;
      icp_global_odom          = *msg;
    }
    mutex_icp_global.unlock();

    got_icp_global = true;
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  routine_icp_global_callback->start();

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_icp_global.lock();
  { interval2 = icp_global_odom.header.stamp - icp_global_odom_previous.header.stamp; }
  mutex_icp_global.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: ICP relative messages came within %1.8f s", interval2.toSec());

    routine_icp_global_callback->end();
    return;
  }


  //////////////////// Fuse Lateral Kalman ////////////////////

  if (_fuse_icp_position) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP position. Waiting for other sensors.");
      return;
    }

    // set the measurement vector
    Eigen::VectorXd mes_icp_global = Eigen::VectorXd::Zero(lateral_p);

    double x_correction = 0;

    // fuse lateral kalman x position
    mutex_icp_global.lock();
    { x_correction = icp_global_odom.pose.pose.position.x - lateralKalmanX->getState(0); }
    mutex_icp_global.unlock();

    // saturate the x_correction
    if (!std::isfinite(x_correction)) {
      x_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in ICP variable \"x_correction\", setting it to 0!!!");
    } else if (x_correction > max_pos_correction) {
      x_correction = max_pos_correction;
    } else if (x_correction < -max_pos_correction) {
      x_correction = -max_pos_correction;
    }

    mutex_lateral_kalman_x.lock();
    {
      mes_icp_global << lateralKalmanX->getState(0) + x_correction;  // apply offsetting from desired center

      lateralKalmanX->setP(P_pos);

      lateralKalmanX->setMeasurement(mes_icp_global, Q_pos_icp);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    double y_correction = 0;

    // fuse lateral kalman x position
    mutex_icp_global.lock();
    { y_correction = icp_global_odom.pose.pose.position.y - lateralKalmanY->getState(0); }
    mutex_icp_global.unlock();

    // saturate the x_correction
    if (!std::isfinite(y_correction)) {
      y_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in ICP variable \"y_correction\", setting it to 0!!!");
    } else if (y_correction > max_pos_correction) {
      y_correction = max_pos_correction;
    } else if (y_correction < -max_pos_correction) {
      y_correction = -max_pos_correction;
    }

    mutex_lateral_kalman_y.lock();
    {
      mes_icp_global << lateralKalmanY->getState(0) + y_correction;  // apply offsetting from desired center

      lateralKalmanY->setP(P_pos);

      lateralKalmanY->setMeasurement(mes_icp_global, Q_pos_icp);

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    ROS_WARN_ONCE("[Odometry]: fusing ICP position");
  }
}
//}

/* //{ callbackMavrosOdometry() */

void Odometry::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  if (got_odom) {

    mutex_odom.lock();
    {
      odom_pixhawk_previous = odom_pixhawk;
      odom_pixhawk          = *msg;
    }
    mutex_odom.unlock();

  } else {

    mutex_odom.lock();
    {
      odom_pixhawk_previous = *msg;
      odom_pixhawk          = *msg;
    }
    mutex_odom.unlock();

    got_odom                 = true;
    odom_pixhawk_last_update = ros::Time::now();
    return;
  }

  odom_pixhawk_last_update = ros::Time::now();

  if (!got_range) {

    mutex_odom.unlock();
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  routine_odometry_callback->start();

  // use our ros::Time as a time stamp for simulation, fixes problems
  /* if (simulation_) { */
  /* mutex_odom.lock(); */
  /* { odom_pixhawk.header.stamp = ros::Time::now(); } */
  /* mutex_odom.unlock(); */
  /* } */

  // set the input vector
  Eigen::VectorXd input;
  input = Eigen::VectorXd::Zero(altitude_p);

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_odom.lock();
  { interval2 = odom_pixhawk.header.stamp - odom_pixhawk_previous.header.stamp; }
  mutex_odom.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: Odometry messages came within %1.8f s", interval2.toSec());

    routine_odometry_callback->end();
    return;
  }

  // set the input computed from two consecutive positions
  mutex_odom.lock();
  { input << (odom_pixhawk.pose.pose.position.z - odom_pixhawk_previous.pose.pose.position.z) / interval2.toSec(); }
  mutex_odom.unlock();

  //////////////////// Fuse MAIN ALT. KALMAN ////////////////////
  //{ fuse main altitude kalman
  mutex_main_altitude_kalman.lock();
  {
    main_altitude_kalman->setInput(input);

    Eigen::MatrixXd newB = Eigen::MatrixXd::Zero(altitude_n, altitude_p);
    newB << interval2.toSec();
    main_altitude_kalman->setB(newB);

    main_altitude_kalman->iterateWithoutCorrection();

    // bottom saturate the altitude
    if (main_altitude_kalman->getState(0) < 0) {

      main_altitude_kalman->setState(0, 0);
    }
  }
  mutex_main_altitude_kalman.unlock();
  //}

  //////////////////// Fuse FAILSAFE ALT. KALMAN ////////////////////
  //{ fuse failsafe altitude kalman
  mutex_failsafe_altitude_kalman.lock();
  {
    failsafe_teraranger_kalman->setInput(input);

    Eigen::MatrixXd newB = Eigen::MatrixXd::Zero(altitude_n, altitude_p);
    newB << interval2.toSec();
    failsafe_teraranger_kalman->setB(newB);

    failsafe_teraranger_kalman->iterateWithoutCorrection();

    // bottom saturate the altitude
    if (failsafe_teraranger_kalman->getState(0) < 0) {

      failsafe_teraranger_kalman->setState(0, 0);
    }
  }
  mutex_failsafe_altitude_kalman.unlock();
  //}

  //////////////////// Fuse Lateral Kalman ////////////////////

  // rotations in inertial frame
  double rot_x, rot_y, rot_z;
  mutex_odom.lock();
  { getGlobalRot(odom_pixhawk.pose.pose.orientation, rot_x, rot_y, rot_z); }
  mutex_odom.unlock();

  // publish orientation for debugging
  orientation_mavros.header   = odom_pixhawk.header;
  orientation_mavros.vector.x = rot_x;
  orientation_mavros.vector.y = rot_y;
  orientation_mavros.vector.z = rot_z;
  pub_orientation_mavros_.publish(orientation_mavros);

  //{ if (_fuse_mavros_tilts)
  if (_fuse_mavros_tilts) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros tilts. Waiting for other sensors.");
      return;
    }

    // x correction
    mutex_lateral_kalman_x.lock();
    {
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

      if (!std::isfinite(rot_y)) {
        rot_y = 0;
        ROS_ERROR("NaN detected in Mavros variable \"rot_y\", setting it to 0 and returning!!!");
        return;
      } else if (rot_y > 1.57) {
        rot_y = 1.57;
      } else if (rot_y < -1.57) {
        rot_y = -1.57;
      }

      mes_lat << rot_y;  // rotation around y-axis corresponds to acceleration in x-axis

      lateralKalmanX->setP(P_ang);

      // set the measurement to kalman filter
      lateralKalmanX->setMeasurement(mes_lat, Q_tilt);

      /* lateralKalmanX->iterate(); */
      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // y correction
    mutex_lateral_kalman_y.lock();
    {
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);
      mes_lat << -rot_x;  // rotation around x-axis corresponds to negative acceleration in y-axis

      lateralKalmanY->setP(P_ang);

      // set the measurement to kalman filter
      lateralKalmanY->setMeasurement(mes_lat, Q_tilt);

      /* Eigen::MatrixXd cov_mat = lateralKalmanX->getCovariance(); */
      /* ROS_WARN("[Odometry]: LKF Covariance before tilt correction:"); */
      /* for (int i = 0; i < cov_mat.rows(); i++) { */
      /*   std::cout << " " << cov_mat(i, i); */
      /* } */
      /* std::cout << std::endl; */
      /* lateralKalmanY->iterate(); */
      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();


    ROS_WARN_ONCE("[Odometry]: fusing Mavros tilts");
  }
  //}

  //{ if (_fuse_mavros_velocity)
  if (_fuse_mavros_velocity) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros velocity. Waiting for other sensors.");
      return;
    }
    // set the measurement vector
    Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

    // fuse lateral kalman x velocity
    double x_twist = 0.0;

    mutex_odom.lock();
    { x_twist = (odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous.pose.pose.position.x) / interval2.toSec(); }
    /* { mes_lat << odom_pixhawk.twist.twist.linear.x; } */
    mutex_odom.unlock();

    // check nans and saturate
    if (!std::isfinite(x_twist)) {
      x_twist = 0;
      ROS_ERROR("NaN detected in Mavros variable \"x_twist\", setting it to 0 and returning!!!");
      return;
    } else if (x_twist > 10) {
      x_twist = 10;
    } else if (x_twist < -10) {
      x_twist = -10;
    }

    mes_lat << x_twist;

    mutex_lateral_kalman_x.lock();
    {

      lateralKalmanX->setP(P_vel);

      lateralKalmanX->setMeasurement(mes_lat, Q_vel_mavros);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // fuse lateral kalman y velocity
    double y_twist = 0.0;

    mutex_odom.lock();
    { y_twist = (odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous.pose.pose.position.y) / interval2.toSec(); }
    /* { mes_lat << odom_pixhawk.twist.twist.linear.y; } */
    mutex_odom.unlock();

    // check nans and saturate
    if (!std::isfinite(y_twist)) {
      y_twist = 0;
      ROS_ERROR("NaN detected in Mavros variable \"y_twist\", setting it to 0 and returning!!!");
      return;
    } else if (y_twist > 10) {
      y_twist = 10;
    } else if (y_twist < -10) {
      y_twist = -10;
    }

    mes_lat << y_twist;

    mutex_lateral_kalman_y.lock();
    {
      lateralKalmanY->setP(P_vel);

      lateralKalmanY->setMeasurement(mes_lat, Q_vel_mavros);

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    trg_last_update = ros::Time::now();

    ROS_WARN_ONCE("[Odometry]: fusing Mavros velocity");
  }
  //}

  //{ if (_fuse_mavros_position)
  if (_fuse_mavros_position) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros position. Waiting for other sensors.");
      return;
    }
    if (!got_odom || (set_home_on_start && !got_global_position)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Mavros position. Global position not averaged.");
      routine_odometry_callback->end();
      return;
    }

    if (!std::isfinite(odom_pixhawk.pose.pose.position.x) || !std::isfinite(odom_pixhawk.pose.pose.position.y)) {
      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local.pose.pose.position.x\" or \"rtk_local.pose.pose.position.y\" (rtk)!!!");
      routine_odometry_callback->end();
      return;
    }
    mutex_lateral_kalman_x.lock();
    {
      // fill the measurement vector
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

      double pos_x = odom_pixhawk.pose.pose.position.x + local_origin_offset_x;

      double x_correction = 0;

      mutex_odom.lock();
      { x_correction = pos_x - lateralKalmanX->getState(0); }
      mutex_odom.unlock();

      // saturate the x_correction
      if (!std::isfinite(x_correction)) {
        x_correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in Mavros variable \"x_correction\", setting it to 0!!!");
      } else if (x_correction > max_pos_correction) {
        x_correction = max_pos_correction;
      } else if (x_correction < -max_pos_correction) {
        x_correction = -max_pos_correction;
      }

      mes_lat << lateralKalmanX->getState(0) + x_correction;  // apply offsetting from desired center

      lateralKalmanX->setP(P_pos);

      // set the measurement to kalman filter
      lateralKalmanX->setMeasurement(mes_lat, Q_pos_mavros);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // fuse rtk gps y position
    mutex_lateral_kalman_y.lock();
    {
      // fill the measurement vector
      Eigen::VectorXd mes_lat = Eigen::VectorXd::Zero(lateral_p);

      double pos_y = odom_pixhawk.pose.pose.position.y + local_origin_offset_y;

      double y_correction = 0;

      mutex_odom.lock();
      { y_correction = pos_y - lateralKalmanY->getState(0); }
      mutex_odom.unlock();

      // saturate the y_correction
      if (!std::isfinite(y_correction)) {
        y_correction = 0;
        ROS_ERROR("[Odometry]: NaN detected in Mavros variable \"y_correction\", setting it to 0!!!");
      } else if (y_correction > max_pos_correction) {
        y_correction = max_pos_correction;
      } else if (y_correction < -max_pos_correction) {
        y_correction = -max_pos_correction;
      }

      mes_lat << lateralKalmanY->getState(0) + y_correction;  // apply offsetting from desired center

      lateralKalmanY->setP(P_pos);
      // set the measurement to kalman filter
      lateralKalmanY->setMeasurement(mes_lat, Q_pos_mavros);

      lateralKalmanY->doCorrection();

      ROS_WARN_ONCE("[Odometry]: fusing Mavros GPS");
    }
    mutex_lateral_kalman_y.unlock();
  }
  //}

  routine_odometry_callback->end();
}

//}

/* //{ callbackVioOdometry() */

void Odometry::callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  if (got_vio) {

    mutex_odom_vio.lock();
    {
      odom_vio_previous = odom_vio;
      odom_vio          = *msg;
    }
    mutex_odom_vio.unlock();

  } else {

    mutex_odom_vio.lock();
    {
      odom_vio_previous = *msg;
      odom_vio          = *msg;
    }
    mutex_odom_vio.unlock();

    got_vio              = true;
    odom_vio_last_update = ros::Time::now();
    return;
  }

  odom_vio_last_update = ros::Time::now();

  if (!got_range) {

    mutex_odom_vio.unlock();
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  routine_vio_callback->start();

  // use our ros::Time as a time stamp for simulation, fixes problems
  /* if (simulation_) { */
  /* mutex_odom.lock(); */
  /* { odom_pixhawk.header.stamp = ros::Time::now(); } */
  /* mutex_odom.unlock(); */
  /* } */

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_odom_vio.lock();
  { interval2 = odom_vio.header.stamp - odom_vio_previous.header.stamp; }
  mutex_odom_vio.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: Vio odometry messages came within %1.8f s", interval2.toSec());

    routine_vio_callback->end();
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  //{ if (_fuse_vio_velocity)
  if (_fuse_vio_velocity) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing vio velocity. Waiting for other sensors.");
      return;
    }
    // set the measurement vector
    Eigen::VectorXd mes_vio_vel_x = Eigen::VectorXd::Zero(lateral_p);

    // fuse lateral kalman x velocity
    double x_twist = 0.0;

    mutex_odom_vio.lock();
    { x_twist = odom_vio.twist.twist.linear.x; }
    mutex_odom_vio.unlock();

    // check nans and saturate
    if (!std::isfinite(x_twist)) {
      x_twist = 0;
      ROS_ERROR("NaN detected in vio variable \"x_twist\", setting it to 0 and returning!!!");
      return;
    } else if (x_twist > 10) {
      x_twist = 10;
    } else if (x_twist < -10) {
      x_twist = -10;
    }

    mes_vio_vel_x << x_twist;

    mutex_lateral_kalman_x.lock();
    {

      lateralKalmanX->setP(P_vel);

      lateralKalmanX->setMeasurement(mes_vio_vel_x, Q_vel_vio);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // set the measurement vector
    Eigen::VectorXd mes_vio_vel_y = Eigen::VectorXd::Zero(lateral_p);

    // fuse lateral kalman y velocity
    double y_twist = 0.0;

    mutex_odom_vio.lock();
    { y_twist = odom_vio.twist.twist.linear.y; }
    mutex_odom_vio.unlock();

    // check nans and saturate
    if (!std::isfinite(y_twist)) {
      y_twist = 0;
      ROS_ERROR("NaN detected in vio variable \"y_twist\", setting it to 0 and returning!!!");
      return;
    } else if (y_twist > 10) {
      y_twist = 10;
    } else if (y_twist < -10) {
      y_twist = -10;
    }

    mes_vio_vel_y << y_twist;

    mutex_lateral_kalman_y.lock();
    {
      lateralKalmanY->setP(P_vel);

      lateralKalmanY->setMeasurement(mes_vio_vel_y, Q_vel_vio);

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    ROS_WARN_ONCE("[Odometry]: fusing vio velocity");
  }
  //}

  //{ if (_fuse_vio_position)
  if (_fuse_vio_position) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing vio position. Waiting for other sensors.");
      return;
    }

    if (!std::isfinite(odom_vio.pose.pose.position.x) || !std::isfinite(odom_vio.pose.pose.position.y)) {
      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"odom_vio.pose.pose.position.x\" or \"odom_vio.pose.pose.position.y\" (vio)!!!");
      routine_vio_callback->end();
      return;
    }
    // fill the measurement vector
    Eigen::VectorXd mes_vio_pos_x = Eigen::VectorXd::Zero(lateral_p);
    double          pos_x         = 0;

    mutex_odom_vio.lock();
    { pos_x = odom_vio.pose.pose.position.x + local_origin_offset_x; }
    mutex_odom_vio.unlock();

    double x_correction = 0;

    mutex_lateral_kalman_x.lock();
    { x_correction = pos_x - lateralKalmanX->getState(0); }
    mutex_lateral_kalman_x.unlock();

    // saturate the x_correction
    if (!std::isfinite(x_correction)) {
      x_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in vio variable \"x_correction\", setting it to 0!!!");
    } else if (x_correction > max_pos_correction) {
      x_correction = max_pos_correction;
    } else if (x_correction < -max_pos_correction) {
      x_correction = -max_pos_correction;
    }

    mutex_lateral_kalman_x.lock();
    {
      mes_vio_pos_x << lateralKalmanX->getState(0) + x_correction;  // apply offsetting from desired center

      lateralKalmanX->setP(P_pos);

      // set the measurement to kalman filter
      lateralKalmanX->setMeasurement(mes_vio_pos_x, Q_pos_vio);

      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    // fuse vio y position
    // fill the measurement vector
    Eigen::VectorXd mes_vio_pos_y = Eigen::VectorXd::Zero(lateral_p);
    double          pos_y         = 0;

    mutex_odom_vio.lock();
    { pos_y = odom_vio.pose.pose.position.y + local_origin_offset_y; }
    mutex_odom_vio.unlock();

    double y_correction = 0;

    mutex_lateral_kalman_y.lock();
    { y_correction = pos_y - lateralKalmanY->getState(0); }
    mutex_lateral_kalman_y.unlock();

    // saturate the y_correction
    if (!std::isfinite(y_correction)) {
      y_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in vio variable \"y_correction\", setting it to 0!!!");
    } else if (y_correction > max_pos_correction) {
      y_correction = max_pos_correction;
    } else if (y_correction < -max_pos_correction) {
      y_correction = -max_pos_correction;
    }

    mutex_lateral_kalman_y.lock();
    {
      mes_vio_pos_y << lateralKalmanY->getState(0) + y_correction;  // apply offsetting from desired center

      lateralKalmanY->setP(P_pos);
      // set the measurement to kalman filter
      lateralKalmanY->setMeasurement(mes_vio_pos_y, Q_pos_vio);

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    ROS_WARN_ONCE("[Odometry]: fusing vio position");
  }
  //}

  routine_vio_callback->end();
}

//}

/* //{ callbackOptflowTwist() */

void Odometry::callbackOptflowTwist(const geometry_msgs::TwistStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  optflow_twist_last_update = ros::Time::now();

  if (got_optflow) {

    mutex_optflow.lock();
    {
      optflow_twist_previous = optflow_twist;
      optflow_twist          = *msg;
    }
    mutex_optflow.unlock();

  } else {

    mutex_optflow.lock();
    {
      optflow_twist_previous = *msg;
      optflow_twist          = *msg;
    }
    mutex_optflow.unlock();

    got_optflow = true;

    return;
  }

  if (!got_range) {

    mutex_optflow.unlock();
    return;
  }

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

  routine_optflow_callback->start();

  // use our ros::Time as a time stamp for simulation, fixes problems
  /* if (simulation_) { */
  /* mutex_odom.lock(); */
  /* { odom_pixhawk.header.stamp = ros::Time::now(); } */
  /* mutex_odom.unlock(); */
  /* } */

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_odom.lock();
  { interval2 = odom_pixhawk.header.stamp - odom_pixhawk_previous.header.stamp; }
  mutex_odom.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: Odometry messages came within %1.8f s", interval2.toSec());

    routine_odometry_callback->end();
    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (_fuse_optflow_velocity) {

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow velocity. Waiting for other sensors.");
      return;
    }
    // set the measurement vector
    Eigen::VectorXd mes_optflow_x = Eigen::VectorXd::Zero(lateral_p);

    // fuse lateral kalman x velocity
    double x_twist = 0;

    mutex_optflow.lock();
    { x_twist = optflow_twist.twist.linear.x; }
    mutex_optflow.unlock();

    if (!std::isfinite(x_twist)) {
      x_twist = 0;
      ROS_ERROR("NaN detected in Optflow variable \"x_twist\", setting it to 0 and returning!!!");
      return;
    } else if (x_twist > 10) {
      x_twist = 10;
    } else if (x_twist < -10) {
      x_twist = -10;
    }

    mes_optflow_x << x_twist;

    // calculate the dynamic covariance based on altitude and stddev
    Q_vel_optflow_dynamic_x = Q_vel_optflow;

    mutex_main_altitude_kalman.lock();
    { Q_vel_optflow_dynamic_x *= main_altitude_kalman->getState(0); }
    mutex_main_altitude_kalman.unlock();

    mutex_optflow_stddev.lock();
    { Q_vel_optflow_dynamic_x *= optflow_stddev.x; }
    mutex_optflow_stddev.unlock();

    /* ROS_INFO_THROTTLE(1.0, "[Odometry]: optflow cov x: %f", Q_vel_optflow_dynamic_x(0, 0)); */

    mutex_lateral_kalman_x.lock();
    {
      lateralKalmanX->setP(P_vel);

      if (_dynamic_optflow_cov) {
        lateralKalmanX->setMeasurement(mes_optflow_x, Q_vel_optflow_dynamic_x);
      } else {
        lateralKalmanX->setMeasurement(mes_optflow_x, Q_vel_optflow);
      }

      /* Eigen::MatrixXd cov_mat = lateralKalmanX->getCovariance(); */
      /* ROS_WARN("[Odometry]: LKF Covariance before optflow correction:"); */
      /* for (int i = 0; i < cov_mat.rows(); i++) { */
      /*   std::cout << " " << cov_mat(i, i); */
      /* } */
      /* std::cout << std::endl; */
      lateralKalmanX->doCorrection();
    }
    mutex_lateral_kalman_x.unlock();

    Eigen::VectorXd mes_optflow_y = Eigen::VectorXd::Zero(lateral_p);

    // fuse lateral kalman y velocity
    double y_twist = 0;

    mutex_optflow.lock();
    { y_twist = optflow_twist.twist.linear.y; }
    mutex_optflow.unlock();

    if (!std::isfinite(y_twist)) {
      y_twist = 0;
      ROS_ERROR("NaN detected in Optflow variable \"y_twist\", setting it to 0 and returning!!!");
      return;
    } else if (y_twist > 10) {
      y_twist = 10;
    } else if (y_twist < -10) {
      y_twist = -10;
    }

    mes_optflow_y << y_twist;

    // calculate the dynamic covariance based on altitude and stddev
    Q_vel_optflow_dynamic_y = Q_vel_optflow;

    mutex_main_altitude_kalman.lock();
    { Q_vel_optflow_dynamic_y *= main_altitude_kalman->getState(0); }
    mutex_main_altitude_kalman.unlock();

    mutex_optflow_stddev.lock();
    { Q_vel_optflow_dynamic_y *= optflow_stddev.y; }
    mutex_optflow_stddev.unlock();

    /* ROS_INFO_THROTTLE(1.0, "[Odometry]: optflow cov y: %f", Q_vel_optflow_dynamic_y(0, 0)); */

    mutex_lateral_kalman_y.lock();
    {
      lateralKalmanY->setP(P_vel);

      if (_dynamic_optflow_cov) {
        lateralKalmanY->setMeasurement(mes_optflow_y, Q_vel_optflow_dynamic_y);
      } else {
        lateralKalmanY->setMeasurement(mes_optflow_y, Q_vel_optflow);
      }

      lateralKalmanY->doCorrection();
    }
    mutex_lateral_kalman_y.unlock();

    ROS_WARN_ONCE("[Odometry]: fusing optical flow velocity");
  }

  routine_optflow_callback->end();
}

//}

/* //{ callbackOptflowStddev() */
void Odometry::callbackOptflowStddev(const geometry_msgs::Vector3ConstPtr &msg) {

  mutex_optflow_stddev.lock();
  { optflow_stddev = *msg; }
  mutex_optflow_stddev.unlock();
}
//}

/* //{ callbackTrackerStatus() */

void Odometry::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  tracker_status     = *msg;
  got_tracker_status = true;
}

//}

/* //{ callbackMavrosDiag() */
void Odometry::callbackMavrosDiag(const mrs_msgs::MavrosDiagnosticsConstPtr &msg) {

  if (!is_initialized)
    return;

  mutex_mavros_diag.lock();
  { mavros_diag.gps.satellites_visible = msg->gps.satellites_visible; }
  mutex_mavros_diag.unlock();

  if (max_altitude != _max_optflow_altitude && mavros_diag.gps.satellites_visible < _min_satellites) {
    max_altitude = _max_optflow_altitude;
    gps_reliable = false;
    ROS_WARN("[Odometry]: GPS unreliable. %d satellites visible. Setting max altitude to max optflow altitude %d.", mavros_diag.gps.satellites_visible,
             max_altitude);
  } else if (_gps_available && max_altitude != _max_default_altitude && mavros_diag.gps.satellites_visible >= _min_satellites) {
    max_altitude = _max_default_altitude;
    gps_reliable = true;
    ROS_WARN("[Odometry]: GPS reliable. %d satellites visible. Setting max altitude to max allowed altitude %d.", mavros_diag.gps.satellites_visible,
             max_altitude);
  }
}
//}

/* //{ callbackToggleRtkHeight() */

bool Odometry::callbackToggleRtkHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  // set the intergrated altitude to the current altitude from kalman
  mutex_main_altitude_kalman.lock();
  { rtk_altitude_integral = main_altitude_kalman->getState(0); }
  mutex_main_altitude_kalman.unlock();

  rtk_altitude_enabled = req.data;

  res.success = true;
  res.message = (rtk_altitude_enabled ? "RTK altitude enabled" : "RTK altitude disabled");

  if (rtk_altitude_enabled) {

    ROS_INFO("[Odometry]: Rtk altitude enabled.");
    teraranger_enabled = false;
    garmin_enabled     = false;
    /* object_altitude_enabled = false; */

  } else {

    ROS_INFO("[Odometry]: Rtk altitude disabled.");
  }

  return true;
}

//}

/* //{ callbackToggleRtkPosition() */

bool Odometry::callbackToggleRtkPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_rtk_position = req.data;

  if (_fuse_rtk_position) {

    ROS_INFO("[Odometry]: Fusing RTK position enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing RTK position disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_rtk_position ? "Fusing RTK position enabled" : "Fusing RTK position disabled");

  return true;
}

//}

/* //{ callbackToggleVioPosition() */

bool Odometry::callbackToggleVioPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_vio_position = req.data;

  if (_fuse_vio_position) {

    ROS_INFO("[Odometry]: Fusing vio position enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing vio position disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_vio_position ? "Fusing vio position enabled" : "Fusing vio position disabled");

  return true;
}

//}

/* //{ callbackToggleVioVelocity() */

bool Odometry::callbackToggleVioVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_vio_velocity = req.data;

  if (_fuse_vio_velocity) {

    ROS_INFO("[Odometry]: Fusing vio velocity enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing vio velocity disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_vio_velocity ? "Fusing vio velocity enabled" : "Fusing vio velocity disabled");

  return true;
}

//}

/* //{ callbackToggleIcpPosition() */

bool Odometry::callbackToggleIcpPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_icp_position = req.data;

  if (_fuse_icp_position) {

    ROS_INFO("[Odometry]: Fusing ICP position enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing ICP position disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_icp_position ? "Fusing ICP position enabled" : "Fusing ICP position disabled");

  return true;
}

//}

/* //{ callbackToggleIcpVelocity() */

bool Odometry::callbackToggleIcpVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_icp_velocity = req.data;

  if (_fuse_icp_velocity) {

    ROS_INFO("[Odometry]: Fusing ICP velocity enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing ICP velocity disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_icp_velocity ? "Fusing ICP velocity enabled" : "Fusing ICP velocity disabled");

  return true;
}

//}

/* //{ callbackToggleMavrosPosition() */

bool Odometry::callbackToggleMavrosPosition(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_mavros_position = req.data;

  if (_fuse_mavros_position) {

    ROS_INFO("[Odometry]: Fusing Mavros position enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing Mavros position disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_mavros_position ? "Fusing Mavros position enabled" : "Fusing Mavros position disabled");

  return true;
}

//}

/* //{ callbackToggleMavrosVelocity() */

bool Odometry::callbackToggleMavrosVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_mavros_velocity = req.data;

  if (_fuse_mavros_velocity) {

    ROS_INFO("[Odometry]: Fusing Mavros velocity enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing Mavros velocity disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_mavros_velocity ? "Fusing Mavros velocity enabled" : "Fusing Mavros velocity disabled");

  return true;
}

//}

/* //{ callbackToggleMavrosTilts() */

bool Odometry::callbackToggleMavrosTilts(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_mavros_tilts = req.data;

  if (_fuse_mavros_tilts) {

    ROS_INFO("[Odometry]: Fusing Mavros tilts enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing Mavros tilts disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_mavros_tilts ? "Fusing Mavros tilts enabled" : "Fusing Mavros tilts disabled");

  return true;
}

//}

/* //{ callbackToggleOptflowVelocity() */

bool Odometry::callbackToggleOptflowVelocity(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  _fuse_optflow_velocity = req.data;

  if (_fuse_optflow_velocity) {

    ROS_INFO("[Odometry]: Fusing optflow velocity enabled.");

  } else {

    ROS_INFO("[Odometry]: Fusing optflow velocity disabled.");
  }

  mutex_odometry_mode.lock();
  { _odometry_mode.mode = mrs_msgs::OdometryMode::OTHER; }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = true;
  res.message = (_fuse_optflow_velocity ? "Fusing opflow velocity enabled" : "Fusing optflow velocity disabled");

  return true;
}

//}

/* //{ callbackChangeOdometryMode() */

bool Odometry::callbackChangeOdometryMode(mrs_msgs::ChangeOdometryMode::Request &req, mrs_msgs::ChangeOdometryMode::Response &res) {

  if (!is_initialized)
    return false;

  // Check whether a valid mode was requested
  if (!isValidMode(req.odometry_mode)) {
    ROS_ERROR("[Odometry]: %d is not a valid odometry mode", req.odometry_mode.mode);
    res.success = false;
    res.message = ("Not a valid odometry mode");
    mutex_odometry_mode.lock();
    { res.odometry_mode.mode = _odometry_mode.mode; }
    mutex_odometry_mode.unlock();
    return true;
  }

  // Check whether OTHER mode was chosen manually
  if (req.odometry_mode.mode == mrs_msgs::OdometryMode::OTHER) {
    ROS_ERROR("[Odometry]: OTHER mode cannot be chosen manually.");
    res.success = false;
    res.message = ("OTHER mode cannot be chosen manually");
    mutex_odometry_mode.lock();
    { res.odometry_mode.mode = _odometry_mode.mode; }
    mutex_odometry_mode.unlock();
    return true;
  }

  bool success = false;
  mutex_odometry_mode.lock();
  {
    mrs_msgs::OdometryMode target_mode;
    target_mode.mode = req.odometry_mode.mode;
    success          = setOdometryModeTo(target_mode);
  }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  mutex_odometry_mode.lock();
  { res.odometry_mode.mode = _odometry_mode.mode; }
  mutex_odometry_mode.unlock();

  return true;
}

//}

/* //{ callbackChangeOdometryModeString() */

bool Odometry::callbackChangeOdometryModeString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  mrs_msgs::OdometryMode target_mode;

  std::string mode = req.value;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
  if (std::strcmp(mode.c_str(), "OPTFLOW") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::OPTFLOW;
  } else if (std::strcmp(mode.c_str(), "GPS") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::GPS;
  } else if (std::strcmp(mode.c_str(), "OPTFLOWGPS") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::OPTFLOWGPS;
  } else if (std::strcmp(mode.c_str(), "RTK") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::RTK;
  } else if (std::strcmp(mode.c_str(), "ICP") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::ICP;
  } else if (std::strcmp(mode.c_str(), "VIO") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::VIO;
  } else if (std::strcmp(mode.c_str(), "OTHER") == 0) {
    target_mode.mode = mrs_msgs::OdometryMode::OTHER;
  } else {
    ROS_WARN("[Odometry]: Invalid mode %s requested", mode.c_str());
    res.success = false;
    res.message = ("Not a valid odometry mode");
    return true;
  }


  // Check whether a valid mode was requested
  if (!isValidMode(target_mode)) {
    ROS_ERROR("[Odometry]: %d is not a valid odometry mode", target_mode.mode);
    res.success = false;
    res.message = ("Not a valid odometry mode");
    return true;
  }

  // Check whether OTHER mode was chosen manually
  if (req.value == "OTHER") {
    ROS_ERROR("[Odometry]: OTHER mode cannot be chosen manually.");
    res.success = false;
    res.message = ("OTHER mode cannot be chosen manually");
    return true;
  }

  bool success = false;
  mutex_odometry_mode.lock();
  { success = setOdometryModeTo(target_mode); }
  mutex_odometry_mode.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());

  return true;
}  // namespace mrs_odometry

//}

/* //{ callbackToggleObjectHeight() */

/*
bool Odometry::callbackToggleObjectHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  object_altitude_enabled = req.data;

  res.success = true;
  res.message = (object_altitude_enabled ? "Object altitude enabled" : "Object altitude disabled");

  if (object_altitude_enabled) {

    ROS_INFO("[Odometry]: Object altitude enabled.");
    teraranger_enabled = false;
    rtk_altitude_enabled = false;

  } else {

    ROS_INFO("[Odometry]: Object altitude disabled.");
  }

  return true;
}
*/

//}

/* //{ callbackToggleTeraranger() */

bool Odometry::callbackToggleTeraranger(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  teraranger_enabled = req.data;

  res.success = true;
  res.message = (teraranger_enabled ? "Teraranger enabled" : "Teraranger disabled");

  if (teraranger_enabled) {

    ROS_INFO("[Odometry]: Teraranger enabled.");
    rtk_altitude_enabled = false;

  } else {

    ROS_INFO("[Odometry]: Teraranger disabled");
  }

  return true;
}

//}

/* //{ callbackToggleGarmin() */

bool Odometry::callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  if (!is_initialized)
    return false;

  garmin_enabled = req.data;

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

/* //{ callbackResetKalman() */

bool Odometry::callbackResetKalman(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  Eigen::VectorXd states_x = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd states_y = Eigen::VectorXd::Zero(6);

  // Delay to be sure that UAV is in the air
  /* ros::Duration(0.1).sleep(); */

  // reset lateral kalman x
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::RTK || _odometry_mode.mode == mrs_msgs::OdometryMode::GPS ||
      _odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS) {
    mutex_lateral_kalman_x.lock();
    { states_x = lateralKalmanX->getStates(); }
    mutex_lateral_kalman_x.unlock();
  } else {
    states_x(0) = init_pose_x;
  }

  states_x(1) = 0.0;
  states_x(2) = 0.0;
  states_x(3) = 0.0;
  states_x(4) = 0.0;
  states_x(5) = 0.0;

  mutex_lateral_kalman_x.lock();
  { lateralKalmanX->reset(states_x); }
  mutex_lateral_kalman_x.unlock();

  // reset lateral kalman y
  if (_odometry_mode.mode == mrs_msgs::OdometryMode::RTK || _odometry_mode.mode == mrs_msgs::OdometryMode::GPS ||
      _odometry_mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS) {
    mutex_lateral_kalman_y.lock();
    { states_y = lateralKalmanY->getStates(); }
    mutex_lateral_kalman_y.unlock();
  } else {
    states_y(0) = init_pose_y;
  }
  states_y(1) = 0.0;
  states_y(2) = 0.0;
  states_y(3) = 0.0;
  states_y(4) = 0.0;
  states_y(5) = 0.0;


  mutex_lateral_kalman_y.lock();
  { lateralKalmanY->reset(states_y); }
  mutex_lateral_kalman_y.unlock();


  ROS_WARN("[Odometry]: Lateral kalman states and covariance reset. Setting position to: x: %f y: %f", init_pose_x, init_pose_y);

  res.success = true;
  res.message = "Reset of lateral kalman successful";

  return true;
}
//}

/* //{ callbackGroundTruth() */
void Odometry::callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg) {

  mutex_ground_truth.lock();
  { ground_truth = *msg; }
  mutex_ground_truth.unlock();

  // rotations in inertial frame
  double rot_x, rot_y, rot_z;
  getGlobalRot(msg->pose.pose.orientation, rot_x, rot_y, rot_z);

  orientation_gt.header   = odom_pixhawk.header;
  orientation_gt.vector.x = rot_x;
  orientation_gt.vector.y = rot_y;
  orientation_gt.vector.z = rot_z;
  pub_orientation_gt_.publish(orientation_gt);
}
//}

/* //{ callbackReconfigure() */
void Odometry::callbackReconfigure(mrs_odometry::lkfConfig &config, uint32_t level) {
  ROS_INFO(
      "Reconfigure Request: Q_pos_mavros: %f, Q_pos_vio: %f, Q_pos_icp: %f, Q_pos_rtk: %f\nQ_vel_mavros: %f, Q_vel_vio: %f, Q_vel_icp: %f, Q_vel_optflow: "
      "%f\nQ_tilt:%f\nR_pos: %f, R_vel: "
      "%f, R_acc: %f, R_acc_i: %f, R_acc_d: %f, R_tilt: %f",
      config.Q_pos_mavros, config.Q_pos_vio, config.Q_pos_icp, config.Q_pos_rtk, config.Q_vel_mavros, config.Q_vel_vio, config.Q_vel_icp, config.Q_vel_optflow,
      config.Q_tilt, config.R_pos, config.R_vel, config.R_acc, config.R_acc_i, config.R_acc_d, config.R_tilt);

  Q_pos_mavros(0, 0) = config.Q_pos_mavros;
  Q_pos_vio(0, 0)    = config.Q_pos_vio;
  Q_pos_icp(0, 0)    = config.Q_pos_icp;
  Q_pos_rtk(0, 0)    = config.Q_pos_rtk;

  Q_vel_mavros(0, 0)  = config.Q_vel_mavros;
  Q_vel_vio(0, 0)     = config.Q_vel_vio;
  Q_vel_icp(0, 0)     = config.Q_vel_icp;
  Q_vel_optflow(0, 0) = config.Q_vel_optflow;

  Q_tilt(0, 0) = config.Q_tilt;

  R2(0, 0) = config.R_pos;
  R2(1, 1) = config.R_vel;
  R2(2, 2) = config.R_acc;
  R2(3, 3) = config.R_acc_i;
  R2(4, 4) = config.R_acc_d;
  R2(5, 5) = config.R_tilt;
}
//}

/* //{ getGlobalRot() */
void Odometry::getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz) {

  tf::Quaternion q_orig, q_rot, q_new;

  quaternionMsgToTF(q_msg, q_orig);

  // Get roll, pitch, yaw in body frame
  double r, p, y, r_new, p_new;
  tf::Matrix3x3(q_orig).getRPY(r, p, y);

  p_new = p * cos(-y) - r * sin(-y);
  r_new = r * cos(-y) + p * sin(-y);

  // Get quaternion of yaw rotation
  /* q_rot = tf::createQuaternionFromRPY(0.0, 0.0, y); */

  // Apply yaw rotation
  /* q_new = q_rot * q_orig; */
  /* q_new.normalize(); */

  // Get quaternion of yaw rotation
  /* q_new = tf::createQuaternionFromRPY(r_new, p_new, y); */

  // Get rotations in inertial frame
  /* tf::Matrix3x3(q_new).getRPY(rx, ry, rz); */

  rx = r_new;
  ry = p_new;
  rz = y;
}
//}

/* //{ setOdometryModeTo() */
bool Odometry::setOdometryModeTo(const mrs_msgs::OdometryMode &target_mode) {

  // Optic flow mode
  if (target_mode.mode == mrs_msgs::OdometryMode::OPTFLOW) {

    if (!_optflow_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW mode. Optic flow not available in this world.");
      return false;
    }

    if (main_altitude_kalman->getState(0) > _max_optflow_altitude) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW mode. Current altitude %f. Must descend to %f.", main_altitude_kalman->getState(0),
                _max_optflow_altitude);
      return false;
    }

    _fuse_optflow_velocity = true;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = false;
    _fuse_mavros_position = false;

    _fuse_icp_velocity = false;
    _fuse_icp_position = false;

    _fuse_vio_velocity = false;
    _fuse_vio_position = false;

    _fuse_rtk_position = false;

    max_altitude = _max_optflow_altitude;

    // Mavros GPS mode
  } else if (target_mode.mode == mrs_msgs::OdometryMode::GPS) {

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS mode. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS mode. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible, _min_satellites);
      return false;
    }

    _fuse_optflow_velocity = false;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = true;
    _fuse_mavros_position = true;

    _fuse_icp_velocity = false;
    _fuse_icp_position = false;

    _fuse_vio_velocity = false;
    _fuse_vio_position = false;

    _fuse_rtk_position = false;

    // Optic flow + Mavros GPS mode
  } else if (target_mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS) {

    if (!_optflow_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS mode. Optic flow not available in this world.");
      return false;
    }

    if (main_altitude_kalman->getState(0) > _max_optflow_altitude) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS mode. Current altitude %f. Must descend to %f.", main_altitude_kalman->getState(0),
                _max_optflow_altitude);
      return false;
    }

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS mode. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS mode. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible,
                _min_satellites);
      return false;
    }

    _fuse_optflow_velocity = true;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = true;
    _fuse_mavros_position = true;

    _fuse_icp_velocity = false;
    _fuse_icp_position = false;

    _fuse_vio_velocity = false;
    _fuse_vio_position = false;

    _fuse_rtk_position = false;

    // RTK GPS mode
  } else if (target_mode.mode == mrs_msgs::OdometryMode::RTK) {

    if (!_rtk_available) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK mode. RTK signal not available in this world.");
      return false;
    }

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK mode. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK mode. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible, _min_satellites);
      return false;
    }

    _fuse_optflow_velocity = false;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = true;
    _fuse_mavros_position = false;

    _fuse_icp_velocity = false;
    _fuse_icp_position = false;

    _fuse_vio_velocity = false;
    _fuse_vio_position = false;

    _fuse_rtk_position = true;

    // LIDAR localization mode
  } else if (target_mode.mode == mrs_msgs::OdometryMode::ICP) {

    if (!_lidar_available) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP mode. Lidar localization not available in this world.");
      return false;
    }

    _fuse_optflow_velocity = false;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = false;
    _fuse_mavros_position = false;

    _fuse_icp_velocity = true;
    _fuse_icp_position = true;

    _fuse_vio_velocity = false;
    _fuse_vio_position = false;

    _fuse_rtk_position = false;

    // Vio localization mode
  } else if (target_mode.mode == mrs_msgs::OdometryMode::VIO) {

    if (!_vio_available) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO mode. Visual odometry not available in this world.");
      return false;
    }

    _fuse_optflow_velocity = false;

    _fuse_mavros_tilts    = true;
    _fuse_mavros_velocity = false;
    _fuse_mavros_position = false;

    _fuse_icp_velocity = false;
    _fuse_icp_position = false;

    _fuse_vio_velocity = false;
    _fuse_vio_position = true;

    _fuse_rtk_position = false;

    _odometry_mode = target_mode;

  } else {

    ROS_ERROR("[Odometry]: Rejected transition to invalid mode %s.", target_mode.name.c_str());
    return false;
  }
  _odometry_mode      = target_mode;
  _odometry_mode.name = _odometry_mode_names[_odometry_mode.mode];
  return true;
}

//}

/* //{ isValidMode() */
bool Odometry::isValidMode(const mrs_msgs::OdometryMode &mode) {

  if (mode.mode == mrs_msgs::OdometryMode::OTHER || mode.mode == mrs_msgs::OdometryMode::OPTFLOW || mode.mode == mrs_msgs::OdometryMode::GPS ||
      mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS || mode.mode == mrs_msgs::OdometryMode::RTK || mode.mode == mrs_msgs::OdometryMode::ICP ||
      mode.mode == mrs_msgs::OdometryMode::VIO) {
    return true;
  }

  return false;
}

//}

/* //{ printOdometryDiag() */
std::string Odometry::printOdometryDiag() {

  mrs_msgs::OdometryMode mode;

  mutex_odometry_mode.lock();
  { mode.mode = _odometry_mode.mode; }
  mutex_odometry_mode.unlock();

  std::string s_diag;

  s_diag += "Odometry Mode: ";
  s_diag += std::to_string(mode.mode);
  s_diag += " - ";

  if (mode.mode == mrs_msgs::OdometryMode::OTHER) {
    s_diag += "OTHER";
  } else if (mode.mode == mrs_msgs::OdometryMode::OPTFLOW) {
    s_diag += "OPTFLOW";
  } else if (mode.mode == mrs_msgs::OdometryMode::GPS) {
    s_diag += "GPS";
  } else if (mode.mode == mrs_msgs::OdometryMode::OPTFLOWGPS) {
    s_diag += "OPTFLOWGPS";
  } else if (mode.mode == mrs_msgs::OdometryMode::RTK) {
    s_diag += "RTK";
  } else if (mode.mode == mrs_msgs::OdometryMode::ICP) {
    s_diag += "ICP";
  } else if (mode.mode == mrs_msgs::OdometryMode::VIO) {
    s_diag += "VIO";
  } else {
    s_diag += "UNKNOWN";
  }

  s_diag += ", optflow_vel: ";
  s_diag += btoa(_fuse_optflow_velocity);

  s_diag += ", mavros_tilts: ";
  s_diag += btoa(_fuse_mavros_tilts);

  s_diag += ", mavros_vel: ";
  s_diag += btoa(_fuse_mavros_velocity);

  s_diag += ", mavros_pos: ";
  s_diag += btoa(_fuse_mavros_position);

  s_diag += ", rtk_pos: ";
  s_diag += btoa(_fuse_rtk_position);

  s_diag += ", vio_vel: ";
  s_diag += btoa(_fuse_vio_velocity);

  s_diag += ", vio_pos: ";
  s_diag += btoa(_fuse_vio_position);

  s_diag += ", icp_vel: ";
  s_diag += btoa(_fuse_icp_velocity);

  s_diag += ", icp_pos: ";
  s_diag += btoa(_fuse_icp_position);

  return s_diag;
}

//}

}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::Odometry, nodelet::Nodelet)
