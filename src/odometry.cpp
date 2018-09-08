#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/TrackerStatus.h>
#include <mrs_msgs/RtkFixType.h>

#include <mrs_lib/Profiler.h>
#include <mrs_lib/Lkf.h>
#include <mrs_lib/GpsConversions.h>

#include <range_filter.h>

#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <Eigen/Eigen>
#include <math.h>
#include <cmath>
#include <mutex>
#include <stdexcept>

#define USE_TERARANGER 1
#define STRING_EQUAL 0

namespace mrs_odometry
{

//{ class Odometry

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

  ros::NodeHandle nh_;

private:
  ros::Publisher pub_odom_;       // the main fused odometry
  ros::Publisher pub_slow_odom_;  // the main fused odometry, just slow
  ros::Publisher pub_rtk_local;
  ros::Publisher pub_rtk_local_odom;

private:
  ros::Subscriber sub_global_position_;
  ros::Subscriber sub_tracker_status_;

  // Pixhawk odometry subscriber and callback
  ros::Subscriber sub_pixhawk_;
  ros::Subscriber rtk_gps_sub_;

private:
  ros::ServiceServer ser_reset_home_;
  ros::ServiceServer ser_averaging_;
  ros::ServiceServer ser_teraranger_;
  ros::ServiceServer ser_garmin_;
  ros::ServiceServer ser_toggle_rtk_altitude;

private:
  tf::TransformBroadcaster *broadcaster_;

  nav_msgs::Odometry odom_pixhawk;
  std::mutex         mutex_odom;
  nav_msgs::Odometry odom_pixhawk_previous_;
  ros::Time          odom_pixhawk_last_update;

  std::mutex       mutex_rtk;
  mrs_msgs::RtkGps rtk_local_previous;
  mrs_msgs::RtkGps rtk_local;

  void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackGlobalPosition(const sensor_msgs::NavSatFix &msg);
  bool callbackResetHome(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackToggleTeraranger(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleGarmin(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool callbackToggleRtkHeight(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg);

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

  bool got_odom, got_range, got_global_position, got_rtk;
  int  got_rtk_counter;
  bool got_rtk_fix;

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

  // altitude kalman
  int             altitude_n, altitude_m, altitude_p;
  Eigen::MatrixXd A1, B1, R1, Q1, Q3, P1;
  mrs_lib::Lkf *  main_altitude_kalman;
  mrs_lib::Lkf *  failsafe_teraranger_kalman;
  std::mutex      mutex_main_altitude_kalman;
  std::mutex      mutex_failsafe_altitude_kalman;

  // lateral kalman
  int             lateral_n, lateral_m, lateral_p;
  Eigen::MatrixXd A2, B2, R2, Q2, P2;
  mrs_lib::Lkf *  lateralKalman;
  std::mutex      mutex_lateral_kalman;

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

  // use differential gps
  bool   use_differential_gps = false;
  bool   pass_rtk_as_new_odom = false;
  double max_rtk_correction;
  double max_altitude_correction_;

  // disabling teraranger on the flight
  bool teraranger_enabled;
  bool garmin_enabled;

  ros::Timer slow_odom_timer;
  int        slow_odom_rate;
  void       slowOdomTimer(const ros::TimerEvent &event);
  void       rtkRateTimer(const ros::TimerEvent &event);

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
  mrs_lib::Routine * routine_garmin_callback;
  mrs_lib::Routine * routine_rtk_callback;

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

//{ onInit()

void Odometry::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Odometry]: initializing");

  nh_.param("uav_name", uav_name, std::string());

  if (uav_name.empty()) {

    ROS_ERROR("[Odometry]: UAV_NAME is empty");
    ros::shutdown();
  }

  odometry_published    = false;
  got_odom              = false;
  got_rtk               = false;
  got_rtk_fix           = false;
  got_global_position   = false;
  got_tracker_status    = false;
  got_home_position_fix = false;
  got_rtk_counter       = 0;

  // got_object_altitude = false;

#if USE_TERARANGER == 1
  got_range = false;
#else
  got_range = true;
#endif

  utm_position_x = 0;
  utm_position_y = 0;

  nh_.param("rate", rate_, -1);

  nh_.param("simulation", simulation_, false);
  nh_.param("slow_odom_rate", slow_odom_rate, 1);
  nh_.param("trgFilterBufferSize", trg_filter_buffer_size, 20);
  nh_.param("trgFilterMaxValidAltitude", trg_max_valid_altitude, 8.0);
  nh_.param("trgFilterMaxDifference", trg_filter_max_difference, 3.0);

  nh_.param("garminFilterBufferSize", garmin_filter_buffer_size, 20);
  nh_.param("garminFilterMaxValidAltitude", garmin_max_valid_altitude, 8.0);
  nh_.param("garminFilterMaxDifference", garmin_filter_max_difference, 3.0);

  /* nh_.param("objectAltitudeBufferSize", object_filter_buffer_size, 20); */
  /* nh_.param("objectAltitudeValidAltitude", object_max_valid_altitude, 8.0); */
  /* nh_.param("objectAltitudeMaxDifference", object_filter_max_difference, 3.0); */
  /* nh_.param("altitude/objectQ", objectQ, 1.0); */
  /* nh_.param("staticObjectHeight", static_object_height_, 0.2); */
  /* nh_.param("dynamicObjectHeight", dynamic_object_height_, 0.2); */
  /* nh_.param("mobius_z_offset_", trg_z_offset_, 0.0); */

  nh_.param("trg_z_offset", trg_z_offset_, 0.0);
  nh_.param("garmin_z_offset", garmin_z_offset_, 0.0);

  nh_.param("home_utm_x", home_utm_x, 0.0);
  nh_.param("home_utm_y", home_utm_y, 0.0);

  if (home_utm_x == 0 || home_utm_y == 0) {

    set_home_on_start = false;

  } else {

    nh_.param("use_home_position", set_home_on_start, false);

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
  nh_.param("averagingNumSamples", averaging_num_samples, 1);
  averaging_started     = false;
  done_averaging        = false;
  averaging_got_samples = 0;

  // declare and initialize variables for the altitude KF
  nh_.param("altitude/numberOfVariables", altitude_n, -1);
  nh_.param("altitude/numberOfInputs", altitude_m, -1);
  nh_.param("altitude/numberOfMeasurements", altitude_p, -1);

  A1 = Eigen::MatrixXd::Zero(altitude_n, altitude_n);
  B1 = Eigen::MatrixXd::Zero(altitude_n, altitude_m);
  R1 = Eigen::MatrixXd::Zero(altitude_n, altitude_n);
  Q1 = Eigen::MatrixXd::Zero(altitude_p, altitude_p);
  Q3 = Eigen::MatrixXd::Zero(altitude_p, altitude_p);
  P1 = Eigen::MatrixXd::Zero(altitude_p, altitude_n);

  std::vector<double> tempList;
  int                 tempIdx = 0;

  tempIdx = 0;
  nh_.getParam("altitude/A", tempList);
  for (int i = 0; i < altitude_n; i++) {
    for (int j = 0; j < altitude_n; j++) {
      A1(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  nh_.getParam("altitude/B", tempList);
  for (int i = 0; i < altitude_n; i++) {
    for (int j = 0; j < altitude_m; j++) {
      B1(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  nh_.getParam("altitude/R", tempList);
  for (int i = 0; i < altitude_n; i++) {
    for (int j = 0; j < altitude_n; j++) {
      R1(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  nh_.getParam("altitude/Q", tempList);
  for (int i = 0; i < altitude_p; i++) {
    for (int j = 0; j < altitude_p; j++) {
      Q1(i, j) = tempList[tempIdx++];
    }
  }

  Q3 = Q1;

  tempIdx = 0;
  nh_.getParam("altitude/P", tempList);
  for (int i = 0; i < altitude_p; i++) {
    for (int j = 0; j < altitude_n; j++) {
      P1(i, j) = tempList[tempIdx++];
    }
  }

  nh_.param("altitude/TrgMaxQ", TrgMaxQ, 1000.0);
  nh_.param("altitude/TrgMinQ", TrgMinQ, 1.0);
  nh_.param("altitude/TrgQChangeRate", TrgQChangeRate, 1.0);

  nh_.param("altitude/GarminMaxQ", GarminMaxQ, 1000.0);
  nh_.param("altitude/GarminMinQ", GarminMinQ, 1.0);
  nh_.param("altitude/GarminQChangeRate", GarminQChangeRate, 1.0);

  nh_.param("altitude/rtkQ", rtkQ, 1.0);

  // failsafes for altitude fusion
  nh_.param("altitude/rtk_max_down_difference", rtk_max_down_difference_, 1.0);
  nh_.param("altitude/rtk_max_abs_difference", rtk_max_abs_difference_, 5.0);

  /* nh_.param("altitude/object_altitude_max_down_difference", object_altitude_max_down_difference_, 1.0); */
  /* nh_.param("altitude/object_altitude_max_abs_difference", object_altitude_max_abs_difference_, 5.0); */

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

  // declare and initialize variables for the altitude KF
  nh_.param("lateral/numberOfVariables", lateral_n, -1);
  nh_.param("lateral/numberOfInputs", lateral_m, -1);
  nh_.param("lateral/numberOfMeasurements", lateral_p, -1);

  A2 = Eigen::MatrixXd::Zero(lateral_n, lateral_n);
  if (lateral_m > 0)
    B2 = Eigen::MatrixXd::Zero(lateral_n, lateral_m);
  R2 = Eigen::MatrixXd::Zero(lateral_n, lateral_n);
  Q2 = Eigen::MatrixXd::Zero(lateral_p, lateral_p);
  P2 = Eigen::MatrixXd::Zero(lateral_p, lateral_n);

  tempIdx = 0;
  nh_.getParam("lateral/A", tempList);
  for (int i = 0; i < lateral_n; i++) {
    for (int j = 0; j < lateral_n; j++) {
      A2(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  if (lateral_m > 0) {
    nh_.getParam("lateral/B", tempList);
    for (int i = 0; i < lateral_n; i++) {
      for (int j = 0; j < lateral_m; j++) {
        B2(i, j) = tempList[tempIdx++];
      }
    }
  }

  tempIdx = 0;
  nh_.getParam("lateral/R", tempList);
  for (int i = 0; i < lateral_n; i++) {
    for (int j = 0; j < lateral_n; j++) {
      R2(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  nh_.getParam("lateral/Q", tempList);
  for (int i = 0; i < lateral_p; i++) {
    for (int j = 0; j < lateral_p; j++) {
      Q2(i, j) = tempList[tempIdx++];
    }
  }

  tempIdx = 0;
  nh_.getParam("lateral/P", tempList);
  for (int i = 0; i < lateral_p; i++) {
    for (int j = 0; j < lateral_n; j++) {
      P2(i, j) = tempList[tempIdx++];
    }
  }

  lateralKalman = new mrs_lib::Lkf(lateral_n, lateral_m, lateral_p, A2, B2, R2, Q2, P2);

  ROS_INFO("[Odometry]: Lateral Kalman prepared");

  // use differential gps
  nh_.param("use_differential_gps", use_differential_gps, false);
  nh_.param("pass_rtk_as_new_odom", pass_rtk_as_new_odom, false);
  nh_.param("max_rtk_correction", max_rtk_correction, 0.5);
  nh_.param("max_altitude_correction", max_altitude_correction_, 0.5);

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
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler                  = new mrs_lib::Profiler(nh_, "Odometry");
  routine_main_timer        = profiler->registerRoutine("main", rate_, 0.002);
  routine_odometry_callback = profiler->registerRoutine("callbackOdometry");
  routine_garmin_callback   = profiler->registerRoutine("callbackGarmin");
  routine_rtk_callback      = profiler->registerRoutine("callbackRtk");

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  // publisher for new odometry
  pub_odom_      = nh_.advertise<nav_msgs::Odometry>("new_odom_out", 1);
  pub_slow_odom_ = nh_.advertise<nav_msgs::Odometry>("slow_odom_out", 1);

  // republisher for rtk local
  pub_rtk_local = nh_.advertise<mrs_msgs::RtkGps>("rtk_local_out", 1);

  // republisher for rtk local odometry (e.g. for rviz)
  pub_rtk_local_odom = nh_.advertise<nav_msgs::Odometry>("rtk_local_odom_out", 1);

  pub_rtk_local = nh_.advertise<mrs_msgs::RtkGps>("rtk_local_out", 1);


  // publisher for tf
  broadcaster_ = new tf::TransformBroadcaster();

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  // subscriber to odometry and rangefinder topics
  sub_pixhawk_ = nh_.subscribe("pixhawk_odom_in", 1, &Odometry::callbackMavrosOdometry, this, ros::TransportHints().tcpNoDelay());

  // subscriber for terarangers range
  sub_terarangerone_ = nh_.subscribe("teraranger_in", 1, &Odometry::callbackTeraranger, this, ros::TransportHints().tcpNoDelay());

  // subscriber for garmin range
  sub_garmin_ = nh_.subscribe("garmin_in", 1, &Odometry::callbackGarmin, this, ros::TransportHints().tcpNoDelay());

  // subscriber for differential gps
  rtk_gps_sub_ = nh_.subscribe("rtk_gps_in", 1, &Odometry::callbackRtkGps, this, ros::TransportHints().tcpNoDelay());

  // subscribe for utm coordinates
  sub_global_position_ = nh_.subscribe("global_position_in", 1, &Odometry::callbackGlobalPosition, this, ros::TransportHints().tcpNoDelay());

  // subscribe for tracker status
  sub_tracker_status_ = nh_.subscribe("tracker_status_in", 1, &Odometry::callbackTrackerStatus, this, ros::TransportHints().tcpNoDelay());

  // subscribe for averaging service
  ser_averaging_ = nh_.advertiseService("average_current_position_in", &Odometry::callbackAveraging, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh_.advertiseService("toggle_garmin_in", &Odometry::callbackToggleGarmin, this);

  // toggling fusing of rtk altitude
  ser_toggle_rtk_altitude = nh_.advertiseService("toggle_rtk_altitude_in", &Odometry::callbackToggleRtkHeight, this);

  // subscribe for resetting home command
  ser_reset_home_ = nh_.advertiseService("reset_home_in", &Odometry::callbackResetHome, this);

  // subscriber for object altitude
  /* object_altitude_sub = nh_.subscribe("object_altitude", 1, &Odometry::callbackObjectHeight, this, ros::TransportHints().tcpNoDelay()); */
  // subscribe for object_altitude toggle service
  /* ser_object_altitude_ = nh_.advertiseService("toggle_object_altitude", &Odometry::callbackToggleObjectHeight, this); */

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer      = nh_.createTimer(ros::Rate(rate_), &Odometry::mainTimer, this);
  slow_odom_timer = nh_.createTimer(ros::Rate(slow_odom_rate), &Odometry::slowOdomTimer, this);
  rtk_rate_timer  = nh_.createTimer(ros::Rate(1), &Odometry::rtkRateTimer, this);

  is_initialized = true;
}

//}

//{ isUavFlying()

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

//{ startAveraging()

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

//{ mainTimer()

void Odometry::mainTimer(const ros::TimerEvent &event) {

  if (!is_initialized)
    return;

  // --------------------------------------------------------------
  // |              publish the new odometry message              |
  // --------------------------------------------------------------

  // if there are some data missing, return
  if (use_differential_gps) {
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position) || !got_rtk) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s",
                        got_odom ? "TRUE" : "FALSE", got_range ? "TRUE" : "FALSE", got_global_position ? "TRUE" : "FALSE", got_rtk ? "TRUE" : "FALSE");
      return;
    }
  } else {
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position)) {
      ROS_INFO_THROTTLE(1, "[Odometry]: Waiting for data from sensors - received? pixhawk: %s, ranger: %s", got_odom ? "TRUE" : "FALSE",
                        got_range ? "TRUE" : "FALSE");
      return;
    }
  }

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

  if (!got_home_position_fix) {

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

    } else {

      routine_main_timer->end();
      return;
    }
  }

  nav_msgs::Odometry new_odom;
  mutex_odom.lock();
  { new_odom = odom_pixhawk; }
  mutex_odom.unlock();

  new_odom.header.frame_id = "local_origin";
  new_odom.child_frame_id  = std::string("fcu_") + uav_name;

  geometry_msgs::PoseStamped newPose;
  newPose.header = new_odom.header;

#if USE_TERARANGER == 1
  // update the altitude state
  mutex_main_altitude_kalman.lock();
  { new_odom.pose.pose.position.z = main_altitude_kalman->getState(0); }
  mutex_main_altitude_kalman.unlock();
#endif

  if (fabs(main_altitude_kalman->getState(0) - failsafe_teraranger_kalman->getState(0)) > 0.5) {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: Main altitude: %2.2f, Failsafe altitude: %2.2f", main_altitude_kalman->getState(0),
                      failsafe_teraranger_kalman->getState(0));
  }

  // if odometry has not been published yet, initialize lateralKF
  if (!odometry_published) {

    mutex_lateral_kalman.lock();
    {
      lateralKalman->setState(0, new_odom.pose.pose.position.x + local_origin_offset_x);
      lateralKalman->setState(1, new_odom.pose.pose.position.y + local_origin_offset_y);
    }
    mutex_lateral_kalman.unlock();
    odometry_published = true;
  }

  // when using differential gps, get the position states from lateralKalman
  if (use_differential_gps) {

    mutex_lateral_kalman.lock();
    {
      new_odom.pose.pose.position.x = lateralKalman->getState(0);
      new_odom.pose.pose.position.y = lateralKalman->getState(1);
    }
    mutex_lateral_kalman.unlock();

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

//{ slowOdomTimer()

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

//{ rtkRateTimer()

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

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackGlobalPosition()

void Odometry::callbackGlobalPosition(const sensor_msgs::NavSatFix &msg) {

  if (!is_initialized)
    return;

  double out_x;
  double out_y;

  mrs_lib::UTM(msg.latitude, msg.longitude, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    ROS_ERROR("[Odometry]: NaN detected in variable \"out_x\"!!!");
    return;
  }

  if (!std::isfinite(out_y)) {
    ROS_ERROR("[Odometry]: NaN detected in variable \"out_y\"!!!");
    return;
  }

  utm_position_x = out_x;
  utm_position_y = out_y;

  got_global_position = true;

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

//{ callbackTeraranger()

void Odometry::callbackTeraranger(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized)
    return;

  range_terarangerone_ = *msg;

  if (!got_odom || (set_home_on_start && !got_global_position)) {

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

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"measurement\" (teraranger)!!!");
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
        ROS_ERROR("[Odometry]: NaN detected in variable \"correction\", setting it to 0!!!");
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
        ROS_ERROR("[Odometry]: NaN detected in variable \"correction\", setting it to 0!!!");
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

//{ callbackGarmin()

void Odometry::callbackGarmin(const sensor_msgs::RangeConstPtr &msg) {

  if (!is_initialized)
    return;

  range_garmin_ = *msg;

  if (!got_odom || (set_home_on_start && !got_global_position)) {

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

    ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"measurement\" (garmin)!!!");

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
        ROS_ERROR("[Odometry]: NaN detected in variable \"correction\", setting it to 0!!!");
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

//{ callbackObjectHeight()

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

//{ callbackAveraging()

bool Odometry::callbackAveraging(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  startAveraging();

  res.success = true;
  res.message = "Started averaging";

  return true;
}

//}

//{ callbackResetHome()

bool Odometry::callbackResetHome(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!is_initialized)
    return false;

  if (isUavFlying()) {

    res.success = false;
    res.message = "Cannot reset home, set motors off!";

    ROS_WARN("[Odometry]: Tried to reset home while the UAV has motors ON!");

  } else {

    mutex_odom.lock();
    {
      local_origin_offset_x = -odom_pixhawk.pose.pose.position.x;
      local_origin_offset_y = -odom_pixhawk.pose.pose.position.y;
    }
    mutex_odom.unlock();

    res.success = true;
    res.message = "home reseted";

    ROS_INFO("[Odometry]: Local origin shifted to current position.");
  }

  return true;
}

//}

//{ callbackRtkGps()

void Odometry::callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg) {

  if (!is_initialized)
    return;

  mrs_msgs::RtkGps rtk_utm;

  if (msg->header.frame_id.compare("gps") == STRING_EQUAL) {

    if (!std::isfinite(msg->gps.latitude)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in variable \"msg->latitude\"!!!");
      return;
    }

    if (!std::isfinite(msg->gps.longitude)) {
      ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in variable \"msg->longitude\"!!!");
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

      got_rtk = true;
    }
  }
  mutex_rtk.unlock();

  if (!got_odom || !got_rtk || (set_home_on_start && !got_global_position)) {

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

  mutex_lateral_kalman.lock();
  {
    // fill the measurement vector
    Eigen::VectorXd mes2 = Eigen::VectorXd::Zero(lateral_p);

    double x_correction = 0;
    double y_correction = 0;

    mutex_rtk.lock();
    {
      x_correction = rtk_local.pose.pose.position.x - lateralKalman->getState(0);
      y_correction = rtk_local.pose.pose.position.y - lateralKalman->getState(1);
    }
    mutex_rtk.unlock();

    // saturate the x_correction
    if (!std::isfinite(x_correction)) {
      x_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in variable \"x_correction\", setting it to 0!!!");
    } else if (x_correction > max_rtk_correction) {
      x_correction = max_rtk_correction;
    } else if (x_correction < -max_rtk_correction) {
      x_correction = -max_rtk_correction;
    }

    // saturate the y_correction
    if (!std::isfinite(y_correction)) {
      y_correction = 0;
      ROS_ERROR("[Odometry]: NaN detected in variable \"y_correction\", setting it to 0!!!");
    } else if (y_correction > max_rtk_correction) {
      y_correction = max_rtk_correction;
    } else if (y_correction < -max_rtk_correction) {
      y_correction = -max_rtk_correction;
    }

    mes2 << lateralKalman->getState(0) + x_correction,  // apply offsetting from desired center
        lateralKalman->getState(1) + y_correction;      // apply offsetting from desired center

    // set the measurement to kalman filter
    lateralKalman->setMeasurement(mes2);

    lateralKalman->doCorrection();

    ROS_WARN_ONCE("[Odometry]: fusing RTK GPS");
  }
  mutex_lateral_kalman.unlock();

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

      ROS_ERROR_THROTTLE(1, "[Odometry]: NaN detected in variable \"rtk_local.position.position.z\" (rtk_altitude)!!!");

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

//{ callbackMavrosOdometry()

void Odometry::callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  if (got_odom) {

    mutex_odom.lock();
    {
      odom_pixhawk_previous_ = odom_pixhawk;
      odom_pixhawk           = *msg;
    }
    mutex_odom.unlock();

  } else {

    mutex_odom.lock();
    {
      odom_pixhawk_previous_ = *msg;
      odom_pixhawk           = *msg;
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
  { interval2 = odom_pixhawk.header.stamp - odom_pixhawk_previous_.header.stamp; }
  mutex_odom.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("[Odometry]: Odometry messages came within %1.8f s", interval2.toSec());

    routine_odometry_callback->end();
    return;
  }

  // set the input computed from two consecutive positions
  mutex_odom.lock();
  { input << (odom_pixhawk.pose.pose.position.z - odom_pixhawk_previous_.pose.pose.position.z) / interval2.toSec(); }
  mutex_odom.unlock();

  //////////////////// Fuse MAIN ALT. KALMAN ////////////////////
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

  //////////////////// Fuse FAILSAFE ALT. KALMAN ////////////////////
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

  //////////////////// Fuse Lateral Kalman ////////////////////
  mutex_lateral_kalman.lock();
  {
    // set the input vector
    input = Eigen::VectorXd::Zero(lateral_m);

    mutex_odom.lock();
    {
      input << (odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous_.pose.pose.position.x) / interval2.toSec(),
          (odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous_.pose.pose.position.y) / interval2.toSec();
    }
    mutex_odom.unlock();

    lateralKalman->setInput(input);

    B2(0, 0) = interval2.toSec();
    B2(1, 1) = interval2.toSec();

    lateralKalman->setB(B2);

    lateralKalman->iterateWithoutCorrection();

    trg_last_update = ros::Time::now();
  }
  mutex_lateral_kalman.unlock();

  routine_odometry_callback->end();
}

//}

//{ callbackTrackerStatus()

void Odometry::callbackTrackerStatus(const mrs_msgs::TrackerStatusConstPtr &msg) {

  if (!is_initialized)
    return;

  tracker_status     = *msg;
  got_tracker_status = true;
}

//}

//{ callbackToggleRtkHeight()

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

//{ callbackToggleObjectHeight()

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

//{ callbackToggleTeraranger()

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

//{ callbackToggleGarmin()

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
}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::Odometry, nodelet::Nodelet)