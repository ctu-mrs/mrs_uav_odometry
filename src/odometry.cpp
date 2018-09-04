/* includes //{ */

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
#include <mrs_msgs/EstimatorType.h>
#include <mrs_msgs/ChangeEstimator.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/LkfStates.h>
#include <mrs_msgs/MavrosDiagnostics.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/OffsetOdom.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/Lkf.h>
#include <mrs_lib/GpsConversions.h>
#include <mrs_lib/ParamLoader.h>

#include <range_filter.h>
#include <StateEstimator.h>
#include <mrs_odometry/lkfConfig.h>

#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <locale>
#include <Eigen/Eigen>
#include <math.h>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <fstream>

//}

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

  bool _publish_fused_odom;
  bool _dynamic_optflow_cov = false;

  double _max_optflow_altitude;
  double _max_default_altitude;
  int    _min_satellites;

  ros::NodeHandle nh_;

private:
  ros::Publisher pub_odom_main_;  // the main fused odometry
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
  ros::Publisher pub_lkf_states_x_;
  ros::Publisher pub_lkf_states_y_;

private:
  ros::Subscriber sub_global_position_;
  ros::Subscriber sub_tracker_status_;

  // Pixhawk odometry subscriber and callback
  ros::Subscriber sub_pixhawk_;
  ros::Subscriber sub_optflow_;
  ros::Subscriber sub_optflow_stddev_;
  ros::Subscriber sub_vio_;
  ros::Subscriber rtk_gps_sub_;
  ros::Subscriber sub_icp_relative_;
  ros::Subscriber sub_icp_global_;
  ros::Subscriber sub_target_attitude_;
  ros::Subscriber sub_ground_truth_;
  ros::Subscriber sub_mavros_diagnostic_;

private:
  ros::ServiceServer ser_reset_lateral_kalman_;
  ros::ServiceServer ser_offset_odom_;
  ros::ServiceServer ser_averaging_;
  ros::ServiceServer ser_teraranger_;
  ros::ServiceServer ser_garmin_;
  ros::ServiceServer ser_toggle_rtk_altitude;
  ros::ServiceServer ser_change_estimator_type;
  ros::ServiceServer ser_change_estimator_type_string;

private:
  tf::TransformBroadcaster *broadcaster_;

  /* dynamic_reconfigure::Server<mrs_odometry::lkfConfig>               server; */
  /* dynamic_reconfigure::Server<mrs_odometry::lkfConfig>::CallbackType f; */

  nav_msgs::Odometry odom_pixhawk;
  std::mutex         mutex_odom;
  nav_msgs::Odometry odom_pixhawk_previous;
  ros::Time          odom_pixhawk_last_update;
  std::mutex         mutex_gps_local_odom;

  std::vector<nav_msgs::Odometry> vec_odom_aux;

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

  mrs_msgs::EstimatorType  _estimator_type;
  mrs_msgs::EstimatorType  _estimator_type_takeoff;
  std::vector<std::string> _estimator_type_names;
  std::mutex               mutex_estimator_type;

  std::string child_frame_id;
  std::mutex  mutex_child_frame_id;


  // | -------------------- message callbacks ------------------- |
  void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackOptflowTwist(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackOptflowStddev(const geometry_msgs::Vector3ConstPtr &msg);
  void callbackGlobalPosition(const sensor_msgs::NavSatFixConstPtr &msg);
  void callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg);
  void callbackIcpRelative(const nav_msgs::OdometryConstPtr &msg);
  void callbackIcpAbsolute(const nav_msgs::OdometryConstPtr &msg);
  void callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg);
  void callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg);
  void callbackReconfigure(mrs_odometry::lkfConfig &config, uint32_t level);
  void callbackMavrosDiag(const mrs_msgs::MavrosDiagnosticsConstPtr &msg);

  // | ------------------- service callbacks ------------------- |
  bool callbackToggleTeraranger(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleRtkHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackChangeEstimator(mrs_msgs::ChangeEstimator::Request &req, mrs_msgs::ChangeEstimator::Response &res);
  bool callbackChangeEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res);
  bool callbackResetEstimator(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // | --------------------- helper methods --------------------- |
  void        stateEstimatorsPrediction(double x, double y, double dt);
  void        stateEstimatorsCorrection(double x, double y, const std::string &measurement_name);
  void        getGlobalRot(const geometry_msgs::Quaternion &q_msg, double &rx, double &ry, double &rz);
  bool        changeCurrentEstimator(const mrs_msgs::EstimatorType &desired_estimator);
  bool        isValidType(const mrs_msgs::EstimatorType &type);
  std::string printOdometryDiag();
  bool        stringInVector(const std::string &value, const std::vector<std::string> &vector);

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

  // State estimation
  int                                             _n_model_states;
  std::vector<std::string>                        _state_estimators_names;
  std::vector<std::string>                        _model_state_names;
  std::vector<std::string>                        _measurement_names;
  std::map<std::string, std::vector<std::string>> map_estimator_measurement;
  std::map<std::string, Eigen::MatrixXd>          map_measurement_covariance;
  std::map<std::string, std::string>              map_measurement_state;
  std::map<std::string, int>                      map_measurement_name_id;
  std::map<std::string, Eigen::MatrixXd>          map_states;
  std::map<std::string, nav_msgs::Odometry>       map_estimator_odom;
  std::map<std::string, ros::Publisher>           map_estimator_pub;
  std::map<std::string, std::shared_ptr<StateEstimator>>    m_state_estimators;
  std::shared_ptr<StateEstimator>                 current_estimator;
  std::mutex                                      mutex_current_estimator;
  std::string                                     current_estimator_name;

  int             lateral_n, lateral_m, lateral_p;
  Eigen::MatrixXd A_lat, B_lat, R_lat;

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
  bool   pass_rtk_as_odom     = false;
  double max_pos_correction_rate;
  double max_altitude_correction_;

  // disabling teraranger on the flight
  bool teraranger_enabled;
  bool garmin_enabled;

  ros::Timer slow_odom_timer;
  ros::Timer diag_timer;
  ros::Timer lkf_states_timer;
  ros::Timer max_altitude_timer;
  ros::Timer topic_watcher_timer;
  int        slow_odom_rate_;
  int        aux_rate_;
  int        diag_rate_;
  int        lkf_states_rate_;
  int        max_altitude_rate_;
  int        topic_watcher_rate_ = 1;
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
  ros::Timer main_timer, aux_timer;
  void       mainTimer(const ros::TimerEvent &event);
  void       auxTimer(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler *profiler;
  bool               profiler_enabled_ = false;

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

  param_loader.load_param("enable_profiler", profiler_enabled_);

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
  // IMPORTANT, update this with each update of the EstimatorType message
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOW));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::GPS));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::OPTFLOWGPS));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::RTK));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::ICP));
  _estimator_type_names.push_back(NAME_OF(mrs_msgs::EstimatorType::VIO));

  ROS_WARN("[Odometry]: SAFETY Checking the EstimatorType2Name conversion. If it fails here, you should update the code above this ROS_INFO");
  for (int i = 0; i < mrs_msgs::EstimatorType::TYPE_COUNT; i++) {
    std::size_t found        = _estimator_type_names[i].find_last_of(":");
    _estimator_type_names[i] = _estimator_type_names[i].substr(found + 1);
    ROS_INFO("[Odometry]: _estimator_type[%d]=%s", i, _estimator_type_names[i].c_str());
  }

  param_loader.load_param("rate", rate_);

  param_loader.load_param("simulation", simulation_);
  param_loader.load_param("slow_odom_rate", slow_odom_rate_);
  param_loader.load_param("diag_rate", diag_rate_);
  param_loader.load_param("aux_rate", aux_rate_);
  param_loader.load_param("max_altitude_rate", max_altitude_rate_);
  param_loader.load_param("lkf_states_rate", lkf_states_rate_);
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

  // Takeoff type
  std::string takeoff_estimator;
  param_loader.load_param("takeoff_estimator", takeoff_estimator);
  std::transform(takeoff_estimator.begin(), takeoff_estimator.end(), takeoff_estimator.begin(), ::toupper);
  size_t pos = std::distance(_estimator_type_names.begin(), find(_estimator_type_names.begin(), _estimator_type_names.end(), takeoff_estimator));
  _estimator_type_takeoff.name = takeoff_estimator;
  _estimator_type_takeoff.type = (int)pos;

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

  //}

  /*  //{ load parameters of state estimators */

  param_loader.load_param("state_estimators/n_model_states", _n_model_states);
  param_loader.load_param("state_estimators/state_estimators", _state_estimators_names);
  param_loader.load_param("state_estimators/model_states", _model_state_names);
  param_loader.load_param("state_estimators/measurements", _measurement_names);
  param_loader.load_matrix_dynamic("state_estimators/A", A_lat, _n_model_states, _n_model_states);
  param_loader.load_matrix_dynamic("state_estimators/B", B_lat, _n_model_states, 1);
  param_loader.load_matrix_dynamic("state_estimators/R", R_lat, _n_model_states, _n_model_states);

  // Load the measurements fused by each state estimator
  for (std::vector<std::string>::iterator it = _state_estimators_names.begin(); it != _state_estimators_names.end(); ++it) {

    std::vector<std::string> temp_vector;
    param_loader.load_param("state_estimators/fused_measurements/" + *it, temp_vector);

    for (std::vector<std::string>::iterator it2 = temp_vector.begin(); it2 != temp_vector.end(); ++it2) {
      if (!stringInVector(*it2, _measurement_names)) {
        ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid measurement name!", it2->c_str(), it->c_str());
        ros::shutdown();
      }
    }

    map_estimator_measurement.insert(std::pair<std::string, std::vector<std::string>>(*it, temp_vector));
  }

  // Load the model state of each measurement
  for (std::vector<std::string>::iterator it = _measurement_names.begin(); it != _measurement_names.end(); ++it) {

    std::string temp_value;
    param_loader.load_param("state_estimators/measurement_states/" + *it, temp_value);

    if (!stringInVector(temp_value, _model_state_names)) {
      ROS_ERROR("[Odometry]: the element '%s' of %s is not a valid model_state name!", temp_value.c_str(), it->c_str());
      ros::shutdown();
    }

    map_measurement_state.insert(std::pair<std::string, std::string>(*it, temp_value));
  }

  // Load the model state mapping
  for (std::vector<std::string>::iterator it = _model_state_names.begin(); it != _model_state_names.end(); ++it) {

    Eigen::MatrixXd temp_P = Eigen::MatrixXd::Zero(1, _n_model_states);
    param_loader.load_matrix_static("state_estimators/state_mapping/" + *it, temp_P, 1, _n_model_states);

    map_states.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_P));
  }

  // Load the covariances of each measurement
  for (std::vector<std::string>::iterator it = _measurement_names.begin(); it != _measurement_names.end(); ++it) {

    Eigen::MatrixXd temp_matrix;
    param_loader.load_matrix_static("state_estimators/Q/" + *it, temp_matrix, 1, 1);

    map_measurement_covariance.insert(std::pair<std::string, Eigen::MatrixXd>(*it, temp_matrix));
  }

  for (std::vector<std::string>::iterator it = _measurement_names.begin(); it < _measurement_names.end(); it++) {
    map_measurement_name_id.insert(std::pair<std::string, int>(*it, (int)std::distance(_measurement_names.begin(), it)));
  }

  //}

  /*  //{ Create state estimators*/
  for (std::vector<std::string>::iterator it = _state_estimators_names.begin(); it != _state_estimators_names.end(); ++it) {

    std::vector<bool>            fusing_measurement;
    std::vector<Eigen::MatrixXd> P_arr, Q_arr;

    // Find measurements fused by the estimator
    std::map<std::string, std::vector<std::string>>::iterator temp_vec = map_estimator_measurement.find(*it);

    // Loop through all measurements
    for (std::vector<std::string>::iterator it2 = _measurement_names.begin(); it2 != _measurement_names.end(); ++it2) {

      // Check whether measurement is fused by the estimator
      if (stringInVector(*it2, temp_vec->second)) {
        fusing_measurement.push_back(true);
      } else {
        fusing_measurement.push_back(false);
      }

      // Find state name
      std::map<std::string, std::string>::iterator pair_measurement_state = map_measurement_state.find(*it2);

      // Find state mapping
      std::map<std::string, Eigen::MatrixXd>::iterator pair_state_matrix = map_states.find(pair_measurement_state->second);
      P_arr.push_back(pair_state_matrix->second);

      // Find measurement covariance
      std::map<std::string, Eigen::MatrixXd>::iterator pair_measurement_covariance = map_measurement_covariance.find(*it2);
      Q_arr.push_back(pair_measurement_covariance->second);
    }

    // Add pointer to state estimator to array
    // this is how to create shared pointers!!! the correct way
    m_state_estimators.insert(std::pair<std::string, std::shared_ptr<StateEstimator>>(*it, std::make_shared<StateEstimator>(*it, fusing_measurement, P_arr, Q_arr, A_lat, B_lat, R_lat)));

    // Map odometry to estimator name
    nav_msgs::Odometry odom;
    std::string        estimator_name = *it;
    std::transform(estimator_name.begin(), estimator_name.end(), estimator_name.begin(), ::tolower);
    odom.child_frame_id = estimator_name;
    map_estimator_odom.insert(std::pair<std::string, nav_msgs::Odometry>(*it, odom));

    // Map publisher to estimator name
    ros::Publisher pub = nh_.advertise<nav_msgs::Odometry>("odom_" + estimator_name + "_out", 1);
    map_estimator_pub.insert(std::pair<std::string, ros::Publisher>(*it, pub));
  }
  //}

  // use differential gps
  param_loader.load_param("use_differential_gps", use_differential_gps);
  param_loader.load_param("publish_fused_odom", _publish_fused_odom);
  param_loader.load_param("pass_rtk_as_odom", pass_rtk_as_odom);
  param_loader.load_param("max_altitude_correction", max_altitude_correction_);

  if (pass_rtk_as_odom && !use_differential_gps) {
    ROS_ERROR("[Odometry]: cant have pass_rtk_as_odom TRUE when use_differential_gps FALSE");
    ros::shutdown();
  }

  ROS_INFO("[Odometry]: Differential GPS %s", use_differential_gps ? "enabled" : "disabled");

  odom_pixhawk_last_update = ros::Time::now();

  teraranger_enabled   = true;
  garmin_enabled       = true;
  rtk_altitude_enabled = false;

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(last_drs_config);
  ReconfigureServer::CallbackType f = boost::bind(&Odometry::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler = new mrs_lib::Profiler(nh_, "Odometry", profiler_enabled_);

  // timer routines
  /* routine_main_timer = profiler->registerRoutine(); */
  /* routine_aux_timer  = profiler->registerRoutine("auxTimer", aux_rate_, 0.002); */


  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  //{ publishers
  // publisher for new odometry
  pub_odom_main_     = nh_.advertise<nav_msgs::Odometry>("odom_main_out", 1);
  pub_slow_odom_     = nh_.advertise<nav_msgs::Odometry>("slow_odom_out", 1);
  pub_odometry_diag_ = nh_.advertise<mrs_msgs::OdometryDiag>("odometry_diag_out", 1);
  pub_altitude_      = nh_.advertise<mrs_msgs::Float64Stamped>("altitude_out", 1);
  pub_max_altitude_  = nh_.advertise<mrs_msgs::Float64Stamped>("max_altitude_out", 1);
  pub_lkf_states_x_  = nh_.advertise<mrs_msgs::LkfStates>("lkf_states_x_out", 1);
  pub_lkf_states_y_  = nh_.advertise<mrs_msgs::LkfStates>("lkf_states_y_out", 1);

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

  // subscriber to optflow velocity
  if (_optflow_available) {
    sub_optflow_        = nh_.subscribe("optflow_in", 1, &Odometry::callbackOptflowTwist, this, ros::TransportHints().tcpNoDelay());
    sub_optflow_stddev_ = nh_.subscribe("optflow_stddev_in", 1, &Odometry::callbackOptflowStddev, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber to visual odometry
  if (_vio_available) {
    sub_vio_ = nh_.subscribe("vio_in", 1, &Odometry::callbackVioOdometry, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for differential gps
  if (use_differential_gps) {
    rtk_gps_sub_ = nh_.subscribe("rtk_gps_in", 1, &Odometry::callbackRtkGps, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for icp odometry
  if (_lidar_available) {
    sub_icp_relative_ = nh_.subscribe("icp_relative_in", 1, &Odometry::callbackIcpRelative, this, ros::TransportHints().tcpNoDelay());
    sub_icp_global_   = nh_.subscribe("icp_absolute_in", 1, &Odometry::callbackIcpAbsolute, this, ros::TransportHints().tcpNoDelay());
  }

  // subscriber for terarangers range
  sub_terarangerone_ = nh_.subscribe("teraranger_in", 1, &Odometry::callbackTeraranger, this, ros::TransportHints().tcpNoDelay());

  // subscriber for garmin range
  sub_garmin_ = nh_.subscribe("garmin_in", 1, &Odometry::callbackGarmin, this, ros::TransportHints().tcpNoDelay());

  // subscriber for ground truth
  sub_ground_truth_ = nh_.subscribe("ground_truth_in", 1, &Odometry::callbackGroundTruth, this, ros::TransportHints().tcpNoDelay());

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
  ser_reset_lateral_kalman_ = nh_.advertiseService("reset_lateral_kalman_in", &Odometry::callbackResetEstimator, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh_.advertiseService("toggle_garmin_in", &Odometry::callbackToggleGarmin, this);

  // toggling fusing of rtk altitude
  ser_toggle_rtk_altitude = nh_.advertiseService("toggle_rtk_altitude_in", &Odometry::callbackToggleRtkHeight, this);

  // change current estimator
  ser_change_estimator_type = nh_.advertiseService("change_estimator_type_in", &Odometry::callbackChangeEstimator, this);

  // change current estimator
  ser_change_estimator_type_string = nh_.advertiseService("change_estimator_type_string_in", &Odometry::callbackChangeEstimatorString, this);
  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer          = nh_.createTimer(ros::Rate(rate_), &Odometry::mainTimer, this);
  aux_timer           = nh_.createTimer(ros::Rate(aux_rate_), &Odometry::auxTimer, this);
  slow_odom_timer     = nh_.createTimer(ros::Rate(slow_odom_rate_), &Odometry::slowOdomTimer, this);
  rtk_rate_timer      = nh_.createTimer(ros::Rate(1), &Odometry::rtkRateTimer, this);
  diag_timer          = nh_.createTimer(ros::Rate(diag_rate_), &Odometry::diagTimer, this);
  lkf_states_timer    = nh_.createTimer(ros::Rate(lkf_states_rate_), &Odometry::lkfStatesTimer, this);
  max_altitude_timer  = nh_.createTimer(ros::Rate(max_altitude_rate_), &Odometry::maxAltitudeTimer, this);
  topic_watcher_timer = nh_.createTimer(ros::Rate(topic_watcher_rate_), &Odometry::topicWatcherTimer, this);

  // Check validity of takeoff type
  ROS_INFO("[Odometry]: Requested %s type for takeoff.", _estimator_type_takeoff.name.c_str());
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOW && !_optflow_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Optflow localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::GPS && !_gps_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. GPS localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::OPTFLOWGPS && !_optflow_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Optflow localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::RTK && !_rtk_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. RTK localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::ICP && !_lidar_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Lidar localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  if (_estimator_type_takeoff.type == mrs_msgs::EstimatorType::VIO && !_vio_available) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Visual odometry localization not available. Shutting down.",
              _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }

  bool success = changeCurrentEstimator(_estimator_type_takeoff);
  if (!success) {
    ROS_ERROR("[Odometry]: The takeoff odometry type %s could not be set. Shutting down.", _estimator_type_takeoff.name.c_str());
    ros::shutdown();
  }
  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loaded_successfully()) {
    ROS_ERROR("[Odometry]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_ERROR("[Odometry]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  ROS_ERROR("[Odometry]: !!!   topic new_odom is deprecated, use odom_main   !!!");
  ROS_ERROR("[Odometry]: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
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

  mrs_lib::Routine profiler_routine = profiler->createRoutine("mainTimer", rate_, 0.004, event);

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
  new_altitude.header.stamp    = ros::Time::now();

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
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_altitude_.getTopic().c_str());
  }

  // --------------------------------------------------------------
  // |              publish the new odometry message              |
  // --------------------------------------------------------------


  // if there are some data missing, return
  if (_estimator_type.type == mrs_msgs::EstimatorType::RTK) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      changeCurrentEstimator(optflow_type);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_rtk) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s",
                        got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_rtk ? "TRUE" : "FALSE");
      return;
    }
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::GPS) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      changeCurrentEstimator(optflow_type);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position)) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE");
      return;
    }
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    if (!gps_reliable && _optflow_available && main_altitude_kalman->getState(0) < _max_optflow_altitude) {
      ROS_WARN("[Odometry]: GPS not reliable. Switching to OPTFLOW type.");
      mrs_msgs::EstimatorType optflow_type;
      optflow_type.type = mrs_msgs::EstimatorType::OPTFLOW;
      changeCurrentEstimator(optflow_type);
    }
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_optflow) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, optflow: %s",
                        got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_optflow ? "TRUE" : "FALSE");
      return;
    }
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW) {
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
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::ICP) {
    if (!got_odom || !got_range || !got_icp) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, icp: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_icp ? "TRUE" : "FALSE");
      return;
    }
  } else if (_estimator_type.type == mrs_msgs::EstimatorType::VIO) {
    if (!got_odom || !got_range || !got_vio) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, vio: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE", got_vio ? "TRUE" : "FALSE");
      return;
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Unknown odometry type. Not checking sensors.");
  }

  got_lateral_sensors = true;
  ROS_INFO_ONCE("[Odometry]: Lateral sensors ready");

  // --------------------------------------------------------------
  // |           check if the odometry is still comming           |
  // --------------------------------------------------------------

  mutex_odom.lock();
  {
    if ((ros::Time::now() - odom_pixhawk_last_update).toSec() > 0.1) {

      ROS_ERROR("[Odometry]: mavros odometry has not come for > 0.1 s, interrupting");
      got_odom = false;
      mutex_odom.unlock();

      return;
    }
  }
  mutex_odom.unlock();

  if (!got_home_position_fix &&
      (_estimator_type.type == mrs_msgs::EstimatorType::RTK || _estimator_type.type == mrs_msgs::EstimatorType::GPS ||
       _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS || (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW && gps_reliable))) {

    if (!averaging && !averaging_started) {

      startAveraging();
      averaging_started = true;

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

        // This is wroooong
        home_utm_x = start_position_average_x;
        home_utm_y = start_position_average_y;

        // just to be sure, set local_origin_offset to 0
        local_origin_offset_x = 0;
        local_origin_offset_y = 0;
      }
      init_pose_set = true;
    } else {

      return;
    }
  } else if (!init_pose_set && _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW &&
             !gps_reliable) {  // Odometry type without GPS -> use start position from config file
    ROS_INFO("[Odometry]: Setting initial position x: %f y: %f from config file.", init_pose_x, init_pose_y);
    local_origin_offset_x = init_pose_x;
    local_origin_offset_y = init_pose_y;
    got_home_position_fix = true;
    init_pose_set         = true;
  }

  nav_msgs::Odometry odom_main;
  mutex_odom.lock();
  { odom_main = odom_pixhawk; }
  mutex_odom.unlock();

  odom_main.header.frame_id = "local_origin";
  odom_main.header.stamp    = ros::Time::now();

  geometry_msgs::PoseStamped newPose;
  newPose.header = odom_main.header;

#if USE_RANGEFINDER == 1
  // update the altitude state
  mutex_main_altitude_kalman.lock();
  { odom_main.pose.pose.position.z = main_altitude_kalman->getState(0); }
  mutex_main_altitude_kalman.unlock();
#endif

  // if odometry has not been published yet, initialize lateralKF
  if (!odometry_published) {
    ROS_INFO("[Odometry]: Initializing the states of current estimator");
    if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW && !gps_reliable) {
      Eigen::VectorXd state(2);
      state << local_origin_offset_x, local_origin_offset_y;
      current_estimator->setState(0, state);
    } else {
      Eigen::VectorXd state(2);
      double          pos_x = odom_main.pose.pose.position.x + local_origin_offset_x;
      double          pos_y = odom_main.pose.pose.position.y + local_origin_offset_y;
      state << pos_x, pos_y;
      current_estimator->setState(0, state);
    }
    ROS_INFO("[Odometry]: Initialized the states of current estimator");
    odometry_published = true;
  }

  if (_publish_fused_odom) {

    Eigen::VectorXd pos_vec(2);
    Eigen::VectorXd vel_vec(2);

    mutex_current_estimator.lock();
    {
      current_estimator->getState(0, pos_vec);
      current_estimator->getState(1, vel_vec);
      odom_main.child_frame_id = current_estimator->getName();
    }
    mutex_current_estimator.unlock();

    odom_main.pose.pose.position.x = pos_vec(0);
    odom_main.twist.twist.linear.x = vel_vec(0);
    odom_main.pose.pose.position.y = pos_vec(1);
    odom_main.twist.twist.linear.y = vel_vec(1);

  } else {

    odom_main.pose.pose.position.x += local_origin_offset_x;
    odom_main.pose.pose.position.y += local_origin_offset_y;
  }

  // publish the odometry
  if (pass_rtk_as_odom) {
    mutex_rtk_local_odom.lock();
    { odom_main = rtk_local_odom; }
    mutex_rtk_local_odom.unlock();
  }

  mutex_shared_odometry.lock();
  { shared_odom = odom_main; }
  mutex_shared_odometry.unlock();

  try {
    pub_odom_main_.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(odom_main)));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_main_.getTopic().c_str());
  }
  ROS_INFO_ONCE("[Odometry]: Publishing odometry");

  // publish TF
  geometry_msgs::Quaternion orientation = odom_main.pose.pose.orientation;
  geometry_msgs::Point      position    = odom_main.pose.pose.position;
  try {
    broadcaster_->sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w), tf::Vector3(position.x, position.y, position.z)),
        odom_main.header.stamp, "local_origin", std::string("fcu_") + uav_name));
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing TF.");
  }
}

//}

/* //{ auxTimer() */

void Odometry::auxTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("auxTimer", aux_rate_, 0.004, event);

  ros::Time t_pub = ros::Time::now();

  // Loop through each estimator
  for (auto &estimator : m_state_estimators) {
    std::map<std::string, nav_msgs::Odometry>::iterator odom_aux = map_estimator_odom.find(estimator.first);
    mutex_odom.lock();
    { odom_aux->second.pose = odom_pixhawk.pose; }
    mutex_odom.unlock();
    odom_aux->second.header.frame_id = "local_origin";
    odom_aux->second.header.stamp    = t_pub;
#if USE_RANGEFINDER == 1
    // update the altitude state
    mutex_main_altitude_kalman.lock();
    { odom_aux->second.pose.pose.position.z = main_altitude_kalman->getState(0); }
    mutex_main_altitude_kalman.unlock();
#endif

    Eigen::VectorXd pos_vec(2);
    Eigen::VectorXd vel_vec(2);

    estimator.second->getState(0, pos_vec);
    estimator.second->getState(1, vel_vec);

    odom_aux->second.pose.pose.position.x = pos_vec(0);
    odom_aux->second.twist.twist.linear.x = vel_vec(0);
    odom_aux->second.pose.pose.position.y = pos_vec(1);
    odom_aux->second.twist.twist.linear.y = vel_vec(1);


    std::map<std::string, ros::Publisher>::iterator pub_odom_aux = map_estimator_pub.find(estimator.second->getName());

    try {
      pub_odom_aux->second.publish(nav_msgs::OdometryConstPtr(new nav_msgs::Odometry(odom_aux->second)));
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub_odom_aux->second.getTopic().c_str());
    }
  }
  ROS_INFO_ONCE("[Odometry]: Publishing auxiliary odometry");
}

//}

/* //{ slowOdomTimer() */

void Odometry::slowOdomTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("slowOdomTimer", slow_odom_rate_, 0.01, event);

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

  mrs_lib::Routine profiler_routine = profiler->createRoutine("diagTimer", diag_rate_, 0.01, event);

  mrs_msgs::OdometryDiag odometry_diag;

  odometry_diag.header.stamp = ros::Time::now();

  odometry_diag.estimator_type = _estimator_type;

  odometry_diag.max_altitude      = max_altitude;
  odometry_diag.gps_reliable      = gps_reliable;
  odometry_diag.gps_available     = _gps_available;
  odometry_diag.optflow_available = _optflow_available;
  odometry_diag.rtk_available     = _rtk_available;
  odometry_diag.lidar_available   = _lidar_available;


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

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("lkfStatesTimer", lkf_states_rate_, 0.01, event);

  Eigen::MatrixXd states_mat = Eigen::MatrixXd::Zero(lateral_n, 2);
  /* Eigen::MatrixXd cov_mat; */

  // get states and covariances from lateral kalman
  mutex_current_estimator.lock();
  {
    current_estimator->getStates(states_mat);
    /* cov_mat    = current_estimator->getCovariance(); */
  }
  mutex_current_estimator.unlock();

  // convert eigen matrix to std::vector
  /* std::vector<double> cov_vec(cov_mat.data(), cov_mat.data() + cov_mat.rows() * cov_mat.cols()); */

  /* Eigen::EigenSolver<Eigen::MatrixXd> es(cov_mat); */

  /* // fill the message */
  mrs_msgs::LkfStates lkf_states_x;
  mrs_msgs::LkfStates lkf_states_y;
  /* for (int i = 0; i < cov_mat.rows(); ++i) { */
  /*   lkf_states.eigenvalues[i] = (es.eigenvalues().col(0)[i].real()); */
  /* } */

  /* for (int i = 0; i < cov_mat.rows(); i++) { */
  /*   lkf_states.covariance[i] = cov_mat(i, i); */
  /* } */

  lkf_states_x.header.stamp = ros::Time::now();
  lkf_states_x.pos          = states_mat(0, 0);
  lkf_states_x.vel          = states_mat(1, 0);
  lkf_states_x.acc          = states_mat(2, 0);
  lkf_states_x.acc_i        = states_mat(3, 0);
  lkf_states_x.acc_d        = states_mat(4, 0);
  lkf_states_x.tilt         = states_mat(5, 0);

  lkf_states_y.header.stamp = ros::Time::now();
  lkf_states_y.pos          = states_mat(0, 1);
  lkf_states_y.vel          = states_mat(1, 1);
  lkf_states_y.acc          = states_mat(2, 1);
  lkf_states_y.acc_i        = states_mat(3, 1);
  lkf_states_y.acc_d        = states_mat(4, 1);
  lkf_states_y.tilt         = states_mat(5, 1);

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
}

//}

/* //{ maxAltitudeTimer() */

void Odometry::maxAltitudeTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("maxAltitudeTimer", max_altitude_rate_, 0.01, event);

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

  mrs_lib::Routine profiler_routine = profiler->createRoutine("rtkRateTimer", 1, 0.01, event);

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

  mrs_lib::Routine profiler_routine = profiler->createRoutine("topicWatcherTimer", topic_watcher_rate_, 0.01, event);

  ros::Duration interval;

  // pixhawk odometry
  interval = ros::Time::now() - odom_pixhawk_last_update;
  if (got_odom && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: Pixhawk odometry not received for %f seconds.", interval.toSec());
    got_odom = false;
  }

  // optflow velocities
  interval = ros::Time::now() - optflow_twist_last_update;
  if (_estimator_type.type == mrs_msgs::EstimatorType::OPTFLOW && got_optflow && interval.toSec() > 1.0) {
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
  if (_estimator_type.type == mrs_msgs::EstimatorType::VIO && got_vio && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: VIO odometry not received for %f seconds.", interval.toSec());
    got_vio = false;
  }

  //  icp velocities
  interval = ros::Time::now() - icp_odom_last_update;
  if (_estimator_type.type == mrs_msgs::EstimatorType::ICP && got_icp && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP velocities not received for %f seconds.", interval.toSec());
    got_icp = false;
  }

  //  icp position
  interval = ros::Time::now() - icp_global_odom_last_update;
  if (_estimator_type.type == mrs_msgs::EstimatorType::ICP && got_icp_global && interval.toSec() > 1.0) {
    ROS_WARN("[Odometry]: ICP position not received for %f seconds.", interval.toSec());
    got_icp_global = false;
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | ------------------ subscriber callbacks ------------------ |

/* //{ callbackTargetAttitude() */
void Odometry::callbackTargetAttitude(const mavros_msgs::AttitudeTargetConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTargetAttitude");

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

  // --------------------------------------------------------------
  // |                        callback body                       |
  // --------------------------------------------------------------

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

  // Apply prediction step to all state estimators
  stateEstimatorsPrediction(rot_y, -rot_x, dt);

  ROS_INFO_ONCE("[Odometry]: Prediction step of all state estimators running.");
}

//}

/* //{ callbackMavrosOdometry() */

void Odometry::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOdometry");

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


  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing mavros odom. Waiting for other sensors.");
    return;
  }

  //{ fuse mavros tilts

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(rot_y, -rot_x, "tilt_mavros");

  ROS_WARN_ONCE("[Odometry]: Fusing mavros tilts");

  //}

  //{ fuse mavros velocity

  if (_gps_available) {

    double vel_mavros_x, vel_mavros_y;
    mutex_odom.lock();
    {
      vel_mavros_x = (odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous.pose.pose.position.x) / interval2.toSec();
      /*  vel_mavros_x << odom_pixhawk.twist.twist.linear.x;  */
      vel_mavros_y = (odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous.pose.pose.position.y) / interval2.toSec();
      /*  vel_mavros_y << odom_pixhawk.twist.twist.linear.y;  */
    }
    mutex_odom.unlock();

    // Apply correction step to all state estimators
    stateEstimatorsCorrection(vel_mavros_x, vel_mavros_y, "vel_mavros");

    ROS_WARN_ONCE("[Odometry]: Fusing mavros velocity");
  }

  //}

  //{ fuse mavros position

  if (_gps_available) {

    if (!got_odom || (set_home_on_start && !got_global_position)) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing Mavros position. Global position not averaged.");
      return;
    }

    double pos_mavros_x, pos_mavros_y;

    mutex_odom.lock();
    {
      pos_mavros_x = odom_pixhawk.pose.pose.position.x + local_origin_offset_x;
      pos_mavros_y = odom_pixhawk.pose.pose.position.y + local_origin_offset_y;
    }
    mutex_odom.unlock();

    // Apply correction step to all state estimators
    stateEstimatorsCorrection(pos_mavros_x, pos_mavros_y, "pos_mavros");

    ROS_WARN_ONCE("[Odometry]: Fusing mavros position");
  }

  //}
}

//}

/* //{ callbackOptflowTwist() */

void Odometry::callbackOptflowTwist(const geometry_msgs::TwistStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOptflowTwist");

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

  // use our ros::Time as a time stamp for simulation, fixes problems
  /* if (simulation_) { */
  /* mutex_odom.lock(); */
  /* { odom_pixhawk.header.stamp = ros::Time::now(); } */
  /* mutex_odom.unlock(); */
  /* } */

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_optflow.lock();
  { interval2 = optflow_twist.header.stamp - optflow_twist_previous.header.stamp; }
  mutex_optflow.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: Odometry messages came within %1.8f s", interval2.toSec());

    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing optflow velocity. Waiting for other sensors.");
    return;
  }

  double optflow_vel_x, optflow_vel_y;
  mutex_optflow.lock();
  {
    optflow_vel_x = optflow_twist.twist.linear.x;
    optflow_vel_y = optflow_twist.twist.linear.y;
  }
  mutex_optflow.unlock();

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(optflow_vel_x, optflow_vel_y, "vel_optflow");


  ROS_WARN_ONCE("[Odometry]: Fusing optflow velocity");
}

//}

/* //{ callbackRtkGps() */

void Odometry::callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackRtk");

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

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_rtk.lock();
  { interval2 = rtk_local.header.stamp - rtk_local_previous.header.stamp; }
  mutex_rtk.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: RTK messages came within %1.8f s", interval2.toSec());

    return;
  }


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

    return;
  }

  if (_rtk_available) {
    if (!std::isfinite(rtk_local.pose.pose.position.x) || !std::isfinite(rtk_local.pose.pose.position.y)) {

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local.pose.pose.position.x\" or \"rtk_local.pose.pose.position.y\" (rtk)!!!");

      return;
    }

    if (!got_lateral_sensors) {
      ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing RTK position. Waiting for other sensors.");
      return;
    }

    double x_rtk, y_rtk;
    mutex_rtk.lock();
    {
      x_rtk = rtk_local.pose.pose.position.x;
      y_rtk = rtk_local.pose.pose.position.y;
    }
    mutex_rtk.unlock();

    if (!std::isfinite(x_rtk)) {
      ROS_ERROR("NaN detected in variable \"x_rtk\" (callbackRtk)!!!");
      return;
    }

    if (!std::isfinite(y_rtk)) {
      ROS_ERROR("NaN detected in variable \"y_rtk\" (callbackRtk)!!!");
      return;
    }

    // Apply correction step to all state estimators
    stateEstimatorsCorrection(x_rtk, y_rtk, "pos_rtk");
  }


  if (rtk_altitude_enabled) {

    if (!got_rtk_fix) {

      rtk_altitude_enabled = false;
      teraranger_enabled   = true;
      garmin_enabled       = true;
      ROS_WARN("[Odometry]: We lost RTK fix, switching back to fusing teraranger and garmin.");

      return;
    }

    // ALTITUDE KALMAN FILTER
    // deside on measurement's covariance
    Eigen::MatrixXd mesCov;
    mesCov = Eigen::MatrixXd::Zero(altitude_p, altitude_p);

    if (!std::isfinite(rtk_local.pose.pose.position.z)) {

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in RTK variable \"rtk_local.position.position.z\" (rtk_altitude)!!!");

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

        return;
      }
    }

    // if rtk integral is too above failsafe kalman, switch to fusing teraranger
    if (fabs(failsafe_teraranger_kalman->getState(0) - rtk_altitude_integral) > rtk_max_abs_difference_) {

      rtk_altitude_enabled = false;
      teraranger_enabled   = true;
      ROS_ERROR("[Odometry]: RTK kalman differs from Failsafe kalman by more than %2.2f m!", rtk_max_abs_difference_);
      ROS_ERROR("[Odometry]: Switching back to fusing teraranger!");

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
}

//}

/* //{ callbackVioOdometry() */

void Odometry::callbackVioOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackVioOdometry");

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

    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  //{ fuse vio velocity

  double vel_vio_x, vel_vio_y;

  mutex_odom_vio.lock();
  {
    vel_vio_x = odom_vio.twist.twist.linear.x;
    vel_vio_y = odom_vio.twist.twist.linear.y;
  }
  mutex_odom_vio.unlock();

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vel_vio_x, vel_vio_y, "vel_vio");

  //}

  //{ fuse vio position

  double vio_pos_x, vio_pos_y;
  mutex_odom_vio.lock();
  {
    vio_pos_x = odom_vio.pose.pose.position.x + local_origin_offset_x;
    vio_pos_y = odom_vio.pose.pose.position.y + local_origin_offset_y;
  }
  mutex_odom_vio.unlock();

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vio_pos_x, vio_pos_x, "vio_pos");

  //}
}

//}

/* //{ callbackIcpRelative() */

void Odometry::callbackIcpRelative(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackIcpRelative");

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

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_icp.lock();
  { interval2 = icp_odom.header.stamp - icp_odom.header.stamp; }
  mutex_icp.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: ICP relative messages came within %1.8f s", interval2.toSec());

    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP velocity. Waiting for other sensors.");
    return;
  }

  double vel_icp_x, vel_icp_y;
  mutex_icp.lock();
  {
    vel_icp_x = icp_odom.twist.twist.linear.x;
    vel_icp_y = icp_odom.twist.twist.linear.y;
  }
  mutex_icp.unlock();

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(vel_icp_x, vel_icp_y, "vel_icp");
}
//}

/* //{ callbackIcpAbsolute() */

void Odometry::callbackIcpAbsolute(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackIcpAbsolute");

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

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_icp_global.lock();
  { interval2 = icp_global_odom.header.stamp - icp_global_odom_previous.header.stamp; }
  mutex_icp_global.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: ICP relative messages came within %1.8f s", interval2.toSec());

    return;
  }

  //////////////////// Fuse Lateral Kalman ////////////////////

  if (!got_lateral_sensors) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Not fusing ICP position. Waiting for other sensors.");
    return;
  }

  double pos_icp_x, pos_icp_y;

  mutex_icp_global.lock();
  {
    pos_icp_x = icp_global_odom.pose.pose.position.x;
    pos_icp_y = icp_global_odom.pose.pose.position.y;
  }
  mutex_icp_global.unlock();

  // Apply correction step to all state estimators
  stateEstimatorsCorrection(pos_icp_x, pos_icp_y, "pos_icp");
}
//}

/* //{ callbackOptflowStddev() */
void Odometry::callbackOptflowStddev(const geometry_msgs::Vector3ConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOptflowStd");

  mutex_optflow_stddev.lock();
  { optflow_stddev = *msg; }
  mutex_optflow_stddev.unlock();
}
//}

/* //{ callbackTeraranger() */

void Odometry::callbackTeraranger(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTeraranger");

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

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackGarmin");

  range_garmin_ = *msg;

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
  measurement = range_garmin_.range * cos(roll) * cos(pitch) + garmin_z_offset_;

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in Garmin variable \"measurement\" (garmin)!!!");

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
}

//}

/* //{ callbackGlobalPosition() */

void Odometry::callbackGlobalPosition(const sensor_msgs::NavSatFixConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackGlobalPosition");

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

/* //{ callbackTrackerStatus() */

void Odometry::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTrackerStatus");

  tracker_status     = *msg;
  got_tracker_status = true;
}
//}

/* //{ callbackMavrosDiag() */
void Odometry::callbackMavrosDiag(const mrs_msgs::MavrosDiagnosticsConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackMavrosDiag");

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

/* //{ callbackGroundTruth() */
void Odometry::callbackGroundTruth(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackGroundTruth");

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

// | -------------------- service callbacks ------------------- |

/* //{ callbackAveraging() */

bool Odometry::callbackAveraging([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  startAveraging();

  res.success = true;
  res.message = "Started averaging";

  return true;
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

/* //{ callbackChangeEstimator() */

bool Odometry::callbackChangeEstimator(mrs_msgs::ChangeEstimator::Request &req, mrs_msgs::ChangeEstimator::Response &res) {

  if (!is_initialized)
    return false;

  // Check whether a valid type was requested
  if (!isValidType(req.estimator_type)) {
    ROS_ERROR("[Odometry]: %d is not a valid odometry type", req.estimator_type.type);
    res.success = false;
    res.message = ("Not a valid odometry type");
    mutex_estimator_type.lock();
    { res.estimator_type.type = _estimator_type.type; }
    mutex_estimator_type.unlock();
    return true;
  }

  bool success = false;
  mutex_estimator_type.lock();
  {
    mrs_msgs::EstimatorType desired_estimator;
    desired_estimator.type = req.estimator_type.type;
    desired_estimator.name = _state_estimators_names[desired_estimator.type];
    success                = changeCurrentEstimator(desired_estimator);
  }
  mutex_estimator_type.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());
  mutex_estimator_type.lock();
  { res.estimator_type.type = _estimator_type.type; }
  mutex_estimator_type.unlock();

  return true;
}

//}

/* //{ callbackChangeEstimatorString() */

bool Odometry::callbackChangeEstimatorString(mrs_msgs::String::Request &req, mrs_msgs::String::Response &res) {

  if (!is_initialized)
    return false;

  mrs_msgs::EstimatorType desired_estimator;

  std::string type = req.value;
  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  if (std::strcmp(type.c_str(), "OPTFLOW") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::OPTFLOW;
  } else if (std::strcmp(type.c_str(), "GPS") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::GPS;
  } else if (std::strcmp(type.c_str(), "OPTFLOWGPS") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::OPTFLOWGPS;
  } else if (std::strcmp(type.c_str(), "RTK") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::RTK;
  } else if (std::strcmp(type.c_str(), "ICP") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::ICP;
  } else if (std::strcmp(type.c_str(), "VIO") == 0) {
    desired_estimator.type = mrs_msgs::EstimatorType::VIO;
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

  desired_estimator.name = _state_estimators_names[desired_estimator.type];

  bool success = false;
  mutex_estimator_type.lock();
  { success = changeCurrentEstimator(desired_estimator); }
  mutex_estimator_type.unlock();

  ROS_INFO("[Odometry]: %s", printOdometryDiag().c_str());

  res.success = success;
  res.message = (printOdometryDiag().c_str());

  return true;
}  // namespace mrs_odometry

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

/* //{ callbackResetEstimator() */

bool Odometry::callbackResetEstimator([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  Eigen::MatrixXd states  = Eigen::MatrixXd::Zero(lateral_n, 2);
  bool            success = false;

  // reset lateral kalman x
  if (_estimator_type.type == mrs_msgs::EstimatorType::RTK || _estimator_type.type == mrs_msgs::EstimatorType::GPS ||
      _estimator_type.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {
    mutex_current_estimator.lock();
    { success = current_estimator->getStates(states); }
    mutex_current_estimator.unlock();
  } else {
    states(0, 0) = init_pose_x;
    states(0, 1) = init_pose_y;
    success      = true;
  }

  if (!success) {

    ROS_ERROR("[Odometry]: Lateral kalman states and covariance reset failed.");

    res.success = false;
    res.message = "Reset of lateral kalman failed";

    return true;
  }

  states(1, 0) = 0.0;
  states(2, 0) = 0.0;
  states(3, 0) = 0.0;
  states(4, 0) = 0.0;
  states(5, 0) = 0.0;
  states(1, 1) = 0.0;
  states(2, 1) = 0.0;
  states(3, 1) = 0.0;
  states(4, 1) = 0.0;
  states(5, 1) = 0.0;

  mutex_current_estimator.lock();
  { current_estimator->reset(states); }
  mutex_current_estimator.unlock();

  ROS_WARN("[Odometry]: Lateral kalman states and covariance reset.");

  res.success = true;
  res.message = "Reset of lateral kalman successful";

  return true;
}
//}

/* //{ callbackReconfigure() */
void Odometry::callbackReconfigure([[maybe_unused]] mrs_odometry::lkfConfig &config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized)
    return;
  ROS_INFO("[Odometry]: TODO callbackReconfigure()");
  /* ROS_INFO( */
  /*     "Reconfigure Request: Q_pos_mavros: %f, Q_pos_vio: %f, Q_pos_icp: %f, Q_pos_rtk: %f\nQ_vel_mavros: %f, Q_vel_vio: %f, Q_vel_icp: %f, Q_vel_optflow: "
   */
  /*     "%f\nQ_tilt:%f\nR_pos: %f, R_vel: " */
  /*     "%f, R_acc: %f, R_acc_i: %f, R_acc_d: %f, R_tilt: %f", */
  /*     config.Q_pos_mavros, config.Q_pos_vio, config.Q_pos_icp, config.Q_pos_rtk, config.Q_vel_mavros, config.Q_vel_vio, config.Q_vel_icp,
   * config.Q_vel_optflow, */
  /*     config.Q_tilt, config.R_pos, config.R_vel, config.R_acc, config.R_acc_i, config.R_acc_d, config.R_tilt); */
}
//}

// --------------------------------------------------------------
// |                      helper functions                      |
// --------------------------------------------------------------

/*  //{ stateEstimatorsPrediction() */

void Odometry::stateEstimatorsPrediction(double x, double y, double dt) {

  if (!std::isfinite(x)) {
    ROS_ERROR("NaN detected in variable \"x\" (stateEstimatorsPrediction) !!!");
    return;
  }

  if (!std::isfinite(y)) {
    ROS_ERROR("NaN detected in variable \"y\" (stateEstimatorsPrediction) !!!");
    return;
  }

  Eigen::VectorXd input = Eigen::VectorXd::Zero(2);
  input << x, y;

  for (auto &estimator : m_state_estimators) {
    estimator.second->doPrediction(input, dt);
    /* Eigen::VectorXd pos_vec(2); */
    /* m_state_estimators[i]->getState(0, pos_vec); */
    /* ROS_INFO("[Odometry]: %s after prediction with input: %f, dt: %f x: %f", m_state_estimators[i]->getName().c_str(), input(0), dt, pos_vec(0)); */
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
    ROS_ERROR("NaN detected in variable \"x\" (stateEstimatorsCorrection) !!!");
    return;
  }

  if (!std::isfinite(y)) {
    ROS_ERROR("NaN detected in variable \"y\" (stateEstimatorsCorrection) !!!");
    return;
  }

  Eigen::VectorXd mes = Eigen::VectorXd::Zero(2);
  mes << x, y;

  for (auto &estimator : m_state_estimators) {
    estimator.second->doCorrection(mes, it_measurement_id->second);
    /* Eigen::VectorXd pos_vec(2); */
    /* estimator.second->getState(0, pos_vec); */
    /* ROS_INFO("[Odometry]: %s after %s correction: %f, x: %f", estimator.second->getName().c_str(), measurement_name.c_str(), mes(0), pos_vec(0)); */
  }
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

/* //{ changeCurrentEstimator() */
bool Odometry::changeCurrentEstimator(const mrs_msgs::EstimatorType &desired_estimator) {

  // Optic flow type
  if (desired_estimator.type == mrs_msgs::EstimatorType::OPTFLOW) {

    if (!_optflow_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. Optic flow not available in this world.");
      return false;
    }

    if (main_altitude_kalman->getState(0) > _max_optflow_altitude) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOW type. Current altitude %f. Must descend to %f.", main_altitude_kalman->getState(0),
                _max_optflow_altitude);
      return false;
    }

    max_altitude = _max_optflow_altitude;

    // Mavros GPS type
  } else if (desired_estimator.type == mrs_msgs::EstimatorType::GPS) {

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to GPS type. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible, _min_satellites);
      return false;
    }

    max_altitude = _max_default_altitude;

    // Optic flow + Mavros GPS type
  } else if (desired_estimator.type == mrs_msgs::EstimatorType::OPTFLOWGPS) {

    if (!_optflow_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Optic flow not available in this world.");
      return false;
    }

    if (main_altitude_kalman->getState(0) > _max_optflow_altitude) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Current altitude %f. Must descend to %f.", main_altitude_kalman->getState(0),
                _max_optflow_altitude);
      return false;
    }

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to OPTFLOWGPS type. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible,
                _min_satellites);
      return false;
    }

    max_altitude = _max_default_altitude;

    // RTK GPS type
  } else if (desired_estimator.type == mrs_msgs::EstimatorType::RTK) {

    if (!_rtk_available) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. RTK signal not available in this world.");
      return false;
    }

    if (!_gps_available) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. GPS signal not available in this world.");
      return false;
    }

    if (!gps_reliable) {
      ROS_ERROR("[Odometry]: Cannot transition to RTK type. Not enough satellites: %d. Required %d.", mavros_diag.gps.satellites_visible, _min_satellites);
      return false;
    }

    max_altitude = _max_default_altitude;

    // LIDAR localization type
  } else if (desired_estimator.type == mrs_msgs::EstimatorType::ICP) {

    if (!_lidar_available) {
      ROS_ERROR("[Odometry]: Cannot transition to ICP type. Lidar localization not available in this world.");
      return false;
    }

    ROS_ERROR("[Odometry]: TODO switching to ICP kalman.");

    // Vio localization type
  } else if (desired_estimator.type == mrs_msgs::EstimatorType::VIO) {

    if (!_vio_available) {
      ROS_ERROR("[Odometry]: Cannot transition to VIO type. Visual odometry not available in this world.");
      return false;
    }

    max_altitude = _max_default_altitude;

  } else {

    ROS_ERROR("[Odometry]: Rejected transition to invalid type %s.", desired_estimator.name.c_str());
    return false;
  }

  if (stringInVector(desired_estimator.name, _state_estimators_names)) {

    mutex_current_estimator.lock();
    {
      /* ROS_WARN_STREAM("[Odometry]: " << m_state_estimators.find(desired_estimator.name)->second->getName()); */
      current_estimator      = m_state_estimators.find(desired_estimator.name)->second;
      current_estimator_name = current_estimator->getName();
    }
    mutex_current_estimator.unlock();

    ROS_WARN("[Odometry]: Transition to %s state estimator successful", current_estimator_name.c_str());

  } else {
    ROS_WARN("[Odometry]: Requested transition to nonexistent state estimator %s", desired_estimator.name.c_str());
    return false;
  }

  _estimator_type      = desired_estimator;
  _estimator_type.name = _estimator_type_names[_estimator_type.type];
  return true;
}

//}

/* //{ isValidType() */
bool Odometry::isValidType(const mrs_msgs::EstimatorType &type) {

  if (type.type == mrs_msgs::EstimatorType::OPTFLOW || type.type == mrs_msgs::EstimatorType::GPS || type.type == mrs_msgs::EstimatorType::OPTFLOWGPS ||
      type.type == mrs_msgs::EstimatorType::RTK || type.type == mrs_msgs::EstimatorType::ICP || type.type == mrs_msgs::EstimatorType::VIO) {
    return true;
  }

  return false;
}

//}

/* //{ printOdometryDiag() */
std::string Odometry::printOdometryDiag() {

  mrs_msgs::EstimatorType type;

  mutex_estimator_type.lock();
  { type.type = _estimator_type.type; }
  mutex_estimator_type.unlock();

  std::string s_diag;

  s_diag += "Current estimator type: ";
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
  } else if (type.type == mrs_msgs::EstimatorType::ICP) {
    s_diag += "ICP";
  } else if (type.type == mrs_msgs::EstimatorType::VIO) {
    s_diag += "VIO";
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

}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::Odometry, nodelet::Nodelet)
