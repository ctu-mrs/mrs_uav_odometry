#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Range.h>
#include <mrs_estimation/lkf.h>
#include "trgfilter.h"
#include <Eigen/Eigen>
#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <quadrotor_msgs/TrackerStatus.h>
#include <mutex>
#include <thread>
#include <std_srvs/SetBool.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdexcept>
#include <mrs_msgs/RtkGpsLocal.h>

#define USE_TERARANGER 1
#define STRING_EQUAL 0

using namespace Eigen;
using namespace std;

/**
 * @brief The mrsOdometry class
 */
class mrsOdometry {

  public:

    mrsOdometry();                            // definition of constructor
    void publishMessage();                        // definition of callback function
    int rate_;

  private:

    std::string uav_name;
    bool simulation_;

    ros::NodeHandle nh_;

  private:

    ros::Publisher pub_odom_;
    ros::Publisher pub_slow_odom_;
    ros::Publisher pub_pose_;

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

    tf::TransformBroadcaster * broadcaster_;

    nav_msgs::Odometry odom_pixhawk;
    std::mutex mutex_odom;
    nav_msgs::Odometry odom_pixhawk_previous_;

    std::mutex mutex_rtk;
    mrs_msgs::RtkGpsLocal rtk_odom_previous;
    mrs_msgs::RtkGpsLocal rtk_odom;

    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void global_position_callback(const nav_msgs::OdometryConstPtr& msg);
    bool resetHomeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool toggleTerarangerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool toggleGarminCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool toggleRtkAltitudeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void rtkCallback(const mrs_msgs::RtkGpsLocalConstPtr &msg);

    // for keeping new odom
    nav_msgs::Odometry shared_odom;
    std::mutex mutex_shared_odometry;

    // Teraranger altitude subscriber and callback
    ros::Subscriber sub_terarangerone_;
    sensor_msgs::Range range_terarangerone_;
    void terarangerCallback(const sensor_msgs::RangeConstPtr& msg);
    TrgFilter * terarangerFilter;
    int trg_filter_buffer_size;
    double trg_max_valid_altitude;
    double trg_filter_max_difference;
    ros::Time trg_last_update;
    double TrgMaxQ, TrgMinQ, TrgQChangeRate;

    // Garmin altitude subscriber and callback
    ros::Subscriber sub_garmin_;
    sensor_msgs::Range range_garmin_;
    void garminCallback(const sensor_msgs::RangeConstPtr& msg);
    TrgFilter * garminFilter;
    int garmin_filter_buffer_size;
    double garmin_max_valid_altitude;
    double garmin_filter_max_difference;
    ros::Time garmin_last_update;
    double GarminMaxQ, GarminMinQ, GarminQChangeRate;

    bool got_odom, got_range, got_global_position, got_rtk;
    int got_rtk_counter;
    bool got_rtk_fix;

    // for setting home position
    bool set_home_on_start;
    double home_utm_x, home_utm_y;		// position set as a utm home position
    double utm_position_x, utm_position_y; // current utm position

    // subscribing to tracker status
    quadrotor_msgs::TrackerStatus tracker_status;
    void tracker_status_callback(const quadrotor_msgs::TrackerStatusConstPtr& msg);
    bool got_tracker_status;
    bool uav_is_flying();

    // offset to adjust the local origin
    double local_origin_offset_x, local_origin_offset_y;

    // altitude kalman
    int altitude_n, altitude_m, altitude_p;
    MatrixXd A1, B1, R1, Q1, Q3, P1;
    LinearKF * main_altitude_kalman;
    LinearKF * failsafe_teraranger_kalman;
    std::mutex mutex_main_altitude_kalman;
    std::mutex mutex_failsafe_altitude_kalman;

    // lateral kalman
    int lateral_n, lateral_m, lateral_p;
    MatrixXd A2, B2, R2, Q2, P2;
    LinearKF * lateralKalman;
    std::mutex mutex_lateral_kalman;

    // averaging of home position
    double gpos_average_x, gpos_average_y;
    double start_position_average_x, start_position_average_y;
    bool averaging, averaging_started, done_averaging;
    int averaging_num_samples;
    void startAveraging();
    int averaging_got_samples;
    bool averagingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool got_home_position_fix;

    bool odometry_published;

    // orientation offset
    bool use_orientation_offset;
    double orientation_offset[4];

    // use differential gps
    bool use_differential_gps;
    double max_rtk_correction;
    double max_altitude_correction_;

    // disabling teraranger on the flight
    bool teraranger_enabled;
    bool garmin_enabled;

    // slow odom thread
    std::thread slow_odom_thread;
    double slow_odom_rate;
    void slowOdomThread(void);
    void rtkRateThread(void);

    // for fusing rtk altitude
    double trg_z_offset_;
    double garmin_z_offset_;

  private:

    // for fusing rtk altitude
    bool rtk_altitude_enabled;
    double rtk_altitude_integral;
    std::thread rtk_rate_thread;
    double rtkQ;
    double rtk_max_down_difference_;
    double rtk_max_abs_difference_;


  private:

    // ############### stuff for adding another source of altitude data ###############

    /* bool object_altitude_enabled; */
    /* bool got_object_altitude; */
    /* object_detection::ObjectWithType object_altitude; */
    /* std::mutex mutex_object_altitude; */
    /* void objectAltitudeCallback(const object_detection::ObjectWithTypeConstPtr &msg); */
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
    /* bool toggleObjectAltitudeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res); */
    /* ros::ServiceServer ser_object_altitude_; */

};

/**
 * @brief mrsOdometry::mrsOdometry
 */
mrsOdometry::mrsOdometry() {

  nh_ = ros::NodeHandle("~");
  nh_.param("uav_name", uav_name, string());

  if (uav_name.empty()) {

    ROS_ERROR("UAV_NAME is empty");
    ros::shutdown();
  }

  // SUBSCRIBERS

  // subscriber to odometry and rangefinder topics
  sub_pixhawk_= nh_.subscribe("pixhawk_odom", 1, &mrsOdometry::odometryCallback, this, ros::TransportHints().tcpNoDelay());

  // subscriber for terarangers range
  sub_terarangerone_= nh_.subscribe("terarangerone", 1, &mrsOdometry::terarangerCallback, this, ros::TransportHints().tcpNoDelay());

  // subscriber for garmin range
  sub_garmin_= nh_.subscribe("garmin", 1, &mrsOdometry::garminCallback, this, ros::TransportHints().tcpNoDelay());

  // subscriber for differential gps
  rtk_gps_sub_ = nh_.subscribe("rtk_gps", 1, &mrsOdometry::rtkCallback, this, ros::TransportHints().tcpNoDelay());

  // subscribe for utm coordinates
  sub_global_position_ = nh_.subscribe("global_position", 1, &mrsOdometry::global_position_callback, this, ros::TransportHints().tcpNoDelay());

  // subscribe for tracker status
  sub_tracker_status_ = nh_.subscribe("tracker_status", 1, &mrsOdometry::tracker_status_callback, this, ros::TransportHints().tcpNoDelay());

  // subscribe for averaging service
  ser_averaging_ = nh_.advertiseService("average_current_position", &mrsOdometry::averagingCallback, this);

  // subscribe for garmin toggle service
  ser_garmin_ = nh_.advertiseService("toggle_garmin", &mrsOdometry::toggleGarminCallback, this);

  // toggling fusing of rtk altitude
  ser_toggle_rtk_altitude = nh_.advertiseService("toggle_rtk_altitude", &mrsOdometry::toggleRtkAltitudeCallback, this);

  // subscriber for object altitude
  /* object_altitude_sub = nh_.subscribe("object_altitude", 1, &mrsOdometry::objectAltitudeCallback, this, ros::TransportHints().tcpNoDelay()); */
  // subscribe for object_altitude toggle service
  /* ser_object_altitude_ = nh_.advertiseService("toggle_object_altitude", &mrsOdometry::toggleObjectAltitudeCallback, this); */

  // publisher for tf
  broadcaster_ = new tf::TransformBroadcaster();

  // PUBLISHERS

  // publisher for new odometry
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("new_odom", 1);
  pub_slow_odom_ = nh_.advertise<nav_msgs::Odometry>("slow_odom", 1);

  // publisher for new pose
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("new_pose", 1);

  // subscribe for resetting home command
  ser_reset_home_ = nh_.advertiseService("reset_home", &mrsOdometry::resetHomeCallback, this);

  odometry_published = false;
  got_odom = false;
  got_rtk = false;
  got_rtk_fix = false;
  got_global_position = false;
  got_tracker_status = false;
  got_home_position_fix = false;
  got_rtk_counter = 0;

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
  nh_.param("slow_odom_rate", slow_odom_rate, 1.0);

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

  }	else {

    nh_.param("use_home_position", set_home_on_start, false);

    if (set_home_on_start)
      ROS_INFO("SetHomeOnStart enabled");
    else
      ROS_INFO("SetHomeOnStart disabled");
  }

  if (set_home_on_start)
    ROS_INFO("Setting home position on %.5f %.5f", home_utm_x, home_utm_y);

  averaging = false;

  local_origin_offset_x = 0;
  local_origin_offset_y = 0;

  // averaging
  nh_.param("averagingNumSamples", averaging_num_samples, 1);
  averaging_started = false;
  done_averaging = false;
  averaging_got_samples = 0;

  // declare and initialize variables for the altitude KF
  nh_.param("altitude/numberOfVariables", altitude_n, -1);
  nh_.param("altitude/numberOfInputs", altitude_m, -1);
  nh_.param("altitude/numberOfMeasurements", altitude_p, -1);

  A1 = MatrixXd::Zero(altitude_n, altitude_n);
  B1 = MatrixXd::Zero(altitude_n, altitude_m);
  R1 = MatrixXd::Zero(altitude_n, altitude_n);
  Q1 = MatrixXd::Zero(altitude_p, altitude_p);
  Q3 = MatrixXd::Zero(altitude_p, altitude_p);
  P1 = MatrixXd::Zero(altitude_p, altitude_n);

  std::vector<double> tempList;
  int tempIdx = 0;

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

  terarangerFilter = new TrgFilter(trg_filter_buffer_size, 0, false, trg_max_valid_altitude, trg_filter_max_difference);
  garminFilter = new TrgFilter(garmin_filter_buffer_size, 0, false, garmin_max_valid_altitude, garmin_filter_max_difference);

  ROS_INFO("Garmin max valid altitude: %2.2f", garmin_max_valid_altitude);
  /* objectAltitudeFilter = new TrgFilter(object_filter_buffer_size, 0, false, object_max_valid_altitude, object_filter_max_difference); */

  main_altitude_kalman = new LinearKF(altitude_n, altitude_m, altitude_p, A1, B1, R1, Q1, P1);
  failsafe_teraranger_kalman = new LinearKF(altitude_n, altitude_m, altitude_p, A1, B1, R1, Q1, P1);

  // initialize the altitude for standing uav
  main_altitude_kalman->setState(0, 0.3);
  failsafe_teraranger_kalman->setState(0, 0.3);

  ROS_INFO_STREAM("Altitude Kalman Filter was initiated with following parameters: n: " << altitude_n << ", m: " << altitude_m << ", p: " << altitude_p << ", A: " << A1 << ", B: " << B1 << ", R: " << R1 << ", Q: " << Q1 << ", P: " << P1 << ", TrgMaxQ: " << TrgMaxQ << ", TrgMinQ: " << TrgMinQ << ", TrgQChangeRate: " << TrgQChangeRate);

  ROS_INFO("Altitude kalman prepared");

  // declare and initialize variables for the altitude KF
  nh_.param("lateral/numberOfVariables", lateral_n, -1);
  nh_.param("lateral/numberOfInputs", lateral_m, -1);
  nh_.param("lateral/numberOfMeasurements", lateral_p, -1);

  A2 = MatrixXd::Zero(lateral_n, lateral_n);
  if (lateral_m > 0)
    B2 = MatrixXd::Zero(lateral_n, lateral_m);
  R2 = MatrixXd::Zero(lateral_n, lateral_n);
  Q2 = MatrixXd::Zero(lateral_p, lateral_p);
  P2 = MatrixXd::Zero(lateral_p, lateral_n);

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

  lateralKalman = new LinearKF(lateral_n, lateral_m, lateral_p, A2, B2, R2, Q2, P2);

  ROS_INFO("Lateral Kalman prepared");

  // load orientation offset

  nh_.param("orientationOffset/enabled", use_orientation_offset, false);

  nh_.param("orientationOffset/x", orientation_offset[0], 0.0);
  nh_.param("orientationOffset/y", orientation_offset[1], 0.0);
  nh_.param("orientationOffset/z", orientation_offset[2], 0.0);
  nh_.param("orientationOffset/w", orientation_offset[3], 1.0);

  ROS_INFO("Orientation offset %s: x=%2.3f, y=%2.3f, z=%2.3f, w=%2.3f", (use_orientation_offset ? "enabled" : "disabled"), orientation_offset[0], orientation_offset[1], orientation_offset[2], orientation_offset[3]);

  // use differential gps
  nh_.param("useDifferentialGps", use_differential_gps, false);
  nh_.param("max_rtk_correction", max_rtk_correction, 0.5);
  nh_.param("max_altitude_correction", max_altitude_correction_, 0.5);

  ROS_INFO("Differential GPS %s", use_differential_gps ? "enabled" : "disabled");

  trg_last_update = ros::Time::now();

  teraranger_enabled = true;
  garmin_enabled = true;
  rtk_altitude_enabled = false;

  // create threads
  slow_odom_thread = std::thread(&mrsOdometry::slowOdomThread, this);
  rtk_rate_thread = std::thread(&mrsOdometry::rtkRateThread, this);
}

bool mrsOdometry::toggleRtkAltitudeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  // set the intergrated altitude to the current altitude from kalman
  mutex_main_altitude_kalman.lock();
  {
    rtk_altitude_integral = main_altitude_kalman->getState(0);
  }
  mutex_main_altitude_kalman.unlock();

  rtk_altitude_enabled = req.data;

  res.success = true;
  res.message = (rtk_altitude_enabled ? "RTK altitude enabled" : "RTK altitude disabled");

  if (rtk_altitude_enabled) {

    ROS_INFO("Rtk altitude enabled.");
    teraranger_enabled = false;
    garmin_enabled = false;
    /* object_altitude_enabled = false; */

  } else {

    ROS_INFO("Rtk altitude disabled.");
  }

  return true;
}

/*
bool mrsOdometry::toggleObjectAltitudeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  object_altitude_enabled = req.data;

  res.success = true;
  res.message = (object_altitude_enabled ? "Object altitude enabled" : "Object altitude disabled");

  if (object_altitude_enabled) {

    ROS_INFO("Object altitude enabled.");
    teraranger_enabled = false;
    rtk_altitude_enabled = false;

  } else {

    ROS_INFO("Object altitude disabled.");
  }

  return true;
}
*/

bool mrsOdometry::toggleTerarangerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  teraranger_enabled = req.data;

  res.success = true;
  res.message = (teraranger_enabled ? "Teraranger enabled" : "Teraranger disabled");

  if (teraranger_enabled) {

    ROS_INFO("Teraranger enabled.");
    rtk_altitude_enabled = false;

  } else {

    ROS_INFO("Teraranger disabled");
  }

  return true;
}

bool mrsOdometry::toggleGarminCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

  garmin_enabled = req.data;

  res.success = true;
  res.message = (garmin_enabled ? "Garmin enabled" : "Garmin disabled");

  if (garmin_enabled) {

    ROS_INFO("Garmin enabled.");
    rtk_altitude_enabled = false;

  } else {

    ROS_INFO("Garmin disabled");
  }

  return true;
}

// published slow odom in reguar intervals
void mrsOdometry::slowOdomThread(void) {

  ROS_INFO("Slow odom thread started.");
  ros::Rate r(slow_odom_rate);

  nav_msgs::Odometry temp_odom;

  while (ros::ok()) {

    mutex_shared_odometry.lock();
    {
      temp_odom = shared_odom;
    }
    mutex_shared_odometry.unlock();

    try {
      pub_slow_odom_.publish(temp_odom);
    } catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_slow_odom_.getTopic().c_str());
    }
    r.sleep();
  }
}

// thread for checking rtk rate
void mrsOdometry::rtkRateThread(void) {

  ROS_INFO("Rtk rate check thread started.");
  ros::Rate r(1.0);

  while (ros::ok()) {

    if (got_rtk) {

      if (got_rtk_counter < 15) {

        ROS_ERROR_THROTTLE(1.0, "RTK comming at slow rate (%d Hz)!", got_rtk_counter);
      }

      got_rtk_counter = 0;
    }

    r.sleep();
  }
}

void mrsOdometry::rtkCallback(const mrs_msgs::RtkGpsLocalConstPtr &msg) {

  mutex_rtk.lock();
  {
    rtk_odom_previous = rtk_odom;
    rtk_odom = *msg;

    if (++got_rtk_counter > 2) {

      got_rtk = true;
    }
  }
  mutex_rtk.unlock();

  if (!got_rtk) {

    return;
  }

  // check whether we have rtk fix
  got_rtk_fix = msg->rtk_fix;

  // continue to lateral and altitude fusin only when we got a fix
  if (!got_rtk_fix) {
    return;
  }

  if (!std::isfinite(rtk_odom.position.x) || !std::isfinite(rtk_odom.position.y)) {

    ROS_ERROR_THROTTLE(1, "NaN detected in variable \"rtk_odom.position.x\" or \"rtk_odom.position.y\" (rtk)!!!");
    return;
  }

  mutex_lateral_kalman.lock();
  {
    // fill the measurement vector
    VectorXd mes2 = VectorXd::Zero(lateral_p);

    double x_correction = 0;
    double y_correction = 0;

    mutex_rtk.lock();
    {
      x_correction = rtk_odom.position.x - home_utm_x - lateralKalman->getState(0);
      y_correction = rtk_odom.position.y - home_utm_y - lateralKalman->getState(1);
    }
    mutex_rtk.unlock();

    // saturate the x_correction
    if (!std::isfinite(x_correction)) {
      x_correction = 0;
      ROS_ERROR("NaN detected in variable \"x_correction\", setting it to 0!!!");
    } else if (x_correction > max_rtk_correction) {
      x_correction = max_rtk_correction;
    } else if (x_correction < -max_rtk_correction) {
      x_correction = -max_rtk_correction;
    }

    // saturate the y_correction
    if (!std::isfinite(y_correction)) {
      y_correction = 0;
      ROS_ERROR("NaN detected in variable \"y_correction\", setting it to 0!!!");
    } else if (y_correction > max_rtk_correction) {
      y_correction = max_rtk_correction;
    } else if (y_correction < -max_rtk_correction) {
      y_correction = -max_rtk_correction;
    }

    mes2 << lateralKalman->getState(0) + x_correction, // apply offsetting from desired center
         lateralKalman->getState(1) + y_correction; // apply offsetting from desired center

    // set the measurement to kalman filter
    lateralKalman->setMeasurement(mes2);

    lateralKalman->doCorrection();
  }
  mutex_lateral_kalman.unlock();

  if (rtk_altitude_enabled) {

    if (!got_rtk_fix) {

      rtk_altitude_enabled = false;
      teraranger_enabled = true;
      garmin_enabled = true;
      ROS_WARN("We lost RTK fix, switching back to fusing teraranger and garmin.");
      return;
    }

    // ALTITUDE KALMAN FILTER
    // deside on measurement's covariance
    MatrixXd mesCov;
    mesCov = MatrixXd::Zero(altitude_p, altitude_p);

    if (!std::isfinite(rtk_odom.position.z)) {

      ROS_ERROR_THROTTLE(1, "NaN detected in variable \"rtk_odom.position.z\" (rtk_altitude)!!!");
      return;
    }

    //////////////////// update rtk integral ////////////////////
    // compute the difference
    double difference = rtk_odom.position.z - rtk_odom_previous.position.z;

    rtk_altitude_integral += difference;

    //////////////////// Compare integral against failsafe kalman ////////////////////
    if (failsafe_teraranger_kalman->getState(0) < 5) { // only when near to the ground

      // if rtk integral is too above failsafe kalman, switch to fusing teraranger
      if ((rtk_altitude_integral - failsafe_teraranger_kalman->getState(0)) > rtk_max_down_difference_) {

        rtk_altitude_enabled = false;
        teraranger_enabled = true;
        ROS_ERROR("RTK kalman is above failsafe kalman by more than %2.2f m!", rtk_max_down_difference_);
        ROS_ERROR("Switching back to fusing teraranger!");
        return;
      }
    }

    // if rtk integral is too above failsafe kalman, switch to fusing teraranger
    if (fabs(failsafe_teraranger_kalman->getState(0) - rtk_altitude_integral) > rtk_max_abs_difference_) {

      rtk_altitude_enabled = false;
      teraranger_enabled = true;
      ROS_ERROR("RTK kalman differs from Failsafe kalman by more than %2.2f m!", rtk_max_abs_difference_);
      ROS_ERROR("Switching back to fusing teraranger!");
      return;
    }

    // set the measurement vector
    VectorXd mes(1);
    mes << rtk_altitude_integral;

    // set variance of gps measurement
    mesCov << rtkQ;

    mutex_main_altitude_kalman.lock();
    {
      main_altitude_kalman->setMeasurement(mes, mesCov);
      main_altitude_kalman->doCorrection();
    }
    mutex_main_altitude_kalman.unlock();

    ROS_INFO_THROTTLE(1, "Fusing rtk altitude");
  }
}

void mrsOdometry::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {

  if (got_odom) {

    mutex_odom.lock();
    {
      odom_pixhawk_previous_ = odom_pixhawk;
      odom_pixhawk = *msg;
    }
    mutex_odom.unlock();

  } else {

    mutex_odom.lock();
    {
      odom_pixhawk_previous_ = *msg;
      odom_pixhawk = *msg;
    }
    mutex_odom.unlock();

    got_odom = true;
    return;
  }

  if (!got_range) {

    mutex_odom.unlock();
    return;
  }

  // use our ros::Time as a time stamp for simulation, fixes problems
  if (simulation_) {

    mutex_odom.lock();
    {
      odom_pixhawk.header.stamp = ros::Time::now();
    }
    mutex_odom.unlock();
  }

  // set the input vector
  VectorXd input;
  input = VectorXd::Zero(altitude_p);

  // compute the time between two last odometries
  ros::Duration interval2;
  mutex_odom.lock();
  {
    interval2 = odom_pixhawk.header.stamp - odom_pixhawk_previous_.header.stamp;
  }
  mutex_odom.unlock();

  if (fabs(interval2.toSec()) < 0.001) {

    ROS_WARN("Odometry messages came within 0.001 s");
    return;
  }

  // set the input computed from two consecutive positions
  mutex_odom.lock();
  {
    input << (odom_pixhawk.pose.pose.position.z - odom_pixhawk_previous_.pose.pose.position.z)/interval2.toSec();
  }
  mutex_odom.unlock();

  //////////////////// Fuse MAIN ALT. KALMAN ////////////////////
  mutex_main_altitude_kalman.lock();
  {
    main_altitude_kalman->setInput(input);

    MatrixXd newB = MatrixXd::Zero(altitude_n, altitude_p);
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

    MatrixXd newB = MatrixXd::Zero(altitude_n, altitude_p);
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
    input = VectorXd::Zero(lateral_m);

    mutex_odom.lock();
    {
      input << (odom_pixhawk.pose.pose.position.x - odom_pixhawk_previous_.pose.pose.position.x)/interval2.toSec(),
            (odom_pixhawk.pose.pose.position.y - odom_pixhawk_previous_.pose.pose.position.y)/interval2.toSec();
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
}

// callback for tracker status
void mrsOdometry::tracker_status_callback(const quadrotor_msgs::TrackerStatusConstPtr &msg) {

  tracker_status = *msg;
  got_tracker_status = true;
}

// true if the uav is "flyable" in the means of whether there is a tracker activate that can fly it
bool mrsOdometry::uav_is_flying() {

  if (got_tracker_status) {

    if (std::string(tracker_status.tracker).compare("std_trackers/NullTracker") == STRING_EQUAL) {

      return false;
    } else {

      return true;
    }

  } else {

    return false;
  }
}

void mrsOdometry::startAveraging() {

  ROS_INFO("startAveraging() called");

  averaging = true;
  gpos_average_x = utm_position_x;
  gpos_average_y = utm_position_y;
  averaging_got_samples = 1;
}

void mrsOdometry::global_position_callback(const nav_msgs::OdometryConstPtr &msg) {

  utm_position_x = msg->pose.pose.position.x;
  utm_position_y = msg->pose.pose.position.y;
  got_global_position = true;

  if (averaging) {

    gpos_average_x = gpos_average_x + (utm_position_x - gpos_average_x)/(averaging_num_samples + 1);
    gpos_average_y = gpos_average_y + (utm_position_y - gpos_average_y)/(averaging_num_samples + 1);

    // stop averaging
    if (averaging_got_samples++ >= averaging_num_samples) {

      averaging = false;
      ROS_INFO("Averaged current position is: %.5f %.5f", gpos_average_x, gpos_average_y);
      done_averaging = true;

    } else {

      ROS_INFO("Sample %d of %d of GPS: %.5f %.5f", averaging_got_samples, averaging_num_samples, gpos_average_x, gpos_average_y);
    }
  }
}

void mrsOdometry::terarangerCallback(const sensor_msgs::RangeConstPtr &msg) {

  range_terarangerone_ = *msg;

  if (!got_odom) {

    return;
  }

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
  measurement = range_terarangerone_.range*cos(roll)*cos(pitch) + trg_z_offset_;

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "NaN detected in variable \"measurement\" (teraranger)!!!");
    return;
  }

  got_range = true;

  // deside on measurement's covariance
  MatrixXd mesCov;
  mesCov = MatrixXd::Zero(altitude_p, altitude_p);

  //////////////////// Filter out terarange measurement ////////////////////
  mutex_failsafe_altitude_kalman.lock();
  {
    // filter out outliers in the case of hight alttide
    if (failsafe_teraranger_kalman->getState(0) > 8 && measurement < 6) {

      measurement = 0;
    }

    // teraranger filtration
    if (uav_is_flying()) {

      ros::Duration interval;
      interval = ros::Time::now() - range_terarangerone_.header.stamp;
      measurement = terarangerFilter->getValue(measurement, failsafe_teraranger_kalman->getState(0), interval);
    }
  }
  mutex_failsafe_altitude_kalman.unlock();

  { // Update variance of Kalman measurement
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
      correction = measurement - failsafe_teraranger_kalman->getState(0);

      // saturate the correction
      if (!std::isfinite(correction)) {
        correction = 0;
        ROS_ERROR("NaN detected in variable \"correction\", setting it to 0!!!");
      }

      // set the measurement vector
      VectorXd mes(1);
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
        ROS_ERROR("NaN detected in variable \"correction\", setting it to 0!!!");
      } else if (correction > max_altitude_correction_) {
        correction = max_altitude_correction_;
      } else if (correction < -max_altitude_correction_) {
        correction = -max_altitude_correction_;
      }

      // set the measurement vector
      VectorXd mes(1);
      mes << main_altitude_kalman->getState(0) + correction;

      mutex_main_altitude_kalman.lock();
      {
        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_INFO_THROTTLE(1, "Fusing Teraranger");
    }
  }
}

void mrsOdometry::garminCallback(const sensor_msgs::RangeConstPtr &msg) {

  range_garmin_ = *msg;

  if (!got_odom) {

    return;
  }

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
  measurement = range_garmin_.range*cos(roll)*cos(pitch) + garmin_z_offset_;

  if (!std::isfinite(measurement)) {

    ROS_ERROR_THROTTLE(1, "NaN detected in variable \"measurement\" (garmin)!!!");
    return;
  }

  got_range = true;

  // deside on measurement's covariance
  MatrixXd mesCov;
  mesCov = MatrixXd::Zero(altitude_p, altitude_p);

  //////////////////// Filter out garmin measurement ////////////////////
  // garmin filtration
  if (uav_is_flying()) {

    ros::Duration interval;
    interval = ros::Time::now() - range_garmin_.header.stamp;
    measurement = garminFilter->getValue(measurement, main_altitude_kalman->getState(0), interval);
  }

  { // Update variance of Kalman measurement
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
        ROS_ERROR("NaN detected in variable \"correction\", setting it to 0!!!");
      } else if (correction > max_altitude_correction_) {
        correction = max_altitude_correction_;
      } else if (correction < -max_altitude_correction_) {
        correction = -max_altitude_correction_;
      }

      // set the measurement vector
      VectorXd mes(1);
      mes << main_altitude_kalman->getState(0) + correction;

      mutex_main_altitude_kalman.lock();
      {
        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_INFO_THROTTLE(1, "Fusing Garmin");
    }
  }
}

/*
void mrsOdometry::objectAltitudeCallback(const object_detection::ObjectWithTypeConstPtr &msg) {

  mutex_object_altitude.lock();
  {
    object_altitude = *msg;
  }
  mutex_object_altitude.unlock();

  got_object_altitude = true;

  ROS_INFO_THROTTLE(1, "Receiving object altitude");

  // ALTITUDE KALMAN FILTER
  // deside on measurement's covariance
  MatrixXd mesCov;
  mesCov = MatrixXd::Zero(altitude_p, altitude_p);

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

    ROS_ERROR_THROTTLE(1, "NaN detected in variable \"measurement\" (object)!!!");
    return;
  }

  ros::Duration interval;

  // object altitude filtration
  if (uav_is_flying()) {

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
      ROS_ERROR("Object altitude is above failsafe kalman by more than %2.2f m!", object_altitude_max_down_difference_);
      ROS_ERROR("Switching back to fusing teraranger!");
      return;
    }

    // if object altitude is too above failsafe kalman, switch to fusing teraranger
    if (fabs(failsafe_teraranger_kalman->getState(0) - measurement) > object_altitude_max_abs_difference_) {

      object_altitude_enabled = false;
      teraranger_enabled = true;
      ROS_ERROR("Object altitude differs from Failsafe kalman by more than %2.2f m!", rtk_max_abs_difference_);
      ROS_ERROR("Switching back to fusing teraranger!");
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
          ROS_ERROR("NaN detected in variable \"correction\", setting it to 0!!!");
        } else if (correction > max_altitude_correction_) {
          correction = max_altitude_correction_;
        } else if (correction < -max_altitude_correction_) {
          correction = -max_altitude_correction_;
        }

        // set the measurement vector
        VectorXd mes(1);
        mes << main_altitude_kalman->getState(0) + correction;

        main_altitude_kalman->setMeasurement(mes, mesCov);
        main_altitude_kalman->doCorrection();
      }
      mutex_main_altitude_kalman.unlock();

      ROS_INFO_THROTTLE(1, "Fusing object altitude");
    }
  }
}
*/

bool mrsOdometry::averagingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  startAveraging();

  res.success = true;
  res.message = "Started averaging";

  return true;
}

bool mrsOdometry::resetHomeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (uav_is_flying()) {

    res.success = false;
    res.message = "Cannot reset home, set motors off!";

    ROS_WARN("Tried to reset home while the UAV has motors ON!");

  } else {

    mutex_odom.lock();
    {
      local_origin_offset_x = -odom_pixhawk.pose.pose.position.x;
      local_origin_offset_y = -odom_pixhawk.pose.pose.position.y;
    }
    mutex_odom.unlock();

    res.success = true;
    res.message = "home reseted";

    ROS_INFO("Local origin shifted to current position.");

  }

  return true;
}

/**
 * @brief mrsOdometry::publishMessage
 */
void mrsOdometry::publishMessage() {

  // if there are some data missing, return
  if (use_differential_gps) {
    if (!got_odom || !got_range || !got_global_position) {
      ROS_INFO_THROTTLE(1, "Waiting for data from sensors - received? pixhawk: %s, ranger: %s, global position: %s, rtk: %s", got_odom? "TRUE":"FALSE", got_range? "TRUE":"FALSE", got_global_position? "TRUE":"FALSE", got_rtk ? "TRUE":"FALSE");
      return;
    }
  } else {
    if (!got_odom || !got_range || (set_home_on_start && !got_global_position)) {
      ROS_INFO_THROTTLE(1, "Waiting for data from sensors - received? pixhawk: %s, ranger: %s", got_odom? "TRUE":"FALSE", got_range? "TRUE":"FALSE");
      return;
    }
  }

  if (!got_home_position_fix) {

    if (!averaging && !averaging_started) {

      startAveraging();
      averaging_started = true;

      return;

    } else if (done_averaging) {

      start_position_average_x = gpos_average_x;
      start_position_average_y = gpos_average_y;
      got_home_position_fix = true;
      ROS_INFO("Finished averaging of home position.");

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

      return;
    }
  }

  nav_msgs::Odometry new_odom;
  mutex_odom.lock();
  {
    new_odom = odom_pixhawk;
  }
  mutex_odom.unlock();

  new_odom.header.frame_id = "local_origin";
  new_odom.child_frame_id = string("fcu_")+uav_name;

  geometry_msgs::PoseStamped newPose;
  newPose.header = new_odom.header;

#if USE_TERARANGER == 1
  // update the altitude state
  mutex_main_altitude_kalman.lock();
  {
    new_odom.pose.pose.position.z = main_altitude_kalman->getState(0);
  }
  mutex_main_altitude_kalman.unlock();
#endif

  ROS_INFO_THROTTLE(1.0, "Main altitude: %2.2f, Failsafe altitude: %2.2f", main_altitude_kalman->getState(0), failsafe_teraranger_kalman->getState(0));

  // add orientation offset
  if (use_orientation_offset) {

    new_odom.pose.pose.orientation.x += orientation_offset[0];
    new_odom.pose.pose.orientation.y += orientation_offset[1];
    new_odom.pose.pose.orientation.z += orientation_offset[2];
    new_odom.pose.pose.orientation.w += orientation_offset[3];
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

  mutex_shared_odometry.lock();
  {
    shared_odom = new_odom;
  }
  mutex_shared_odometry.unlock();

  // publish the odometry
  try {
    pub_odom_.publish(new_odom);
  } catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_odom_.getTopic().c_str());
  }

  // publish the pose
  newPose.pose = new_odom.pose.pose;
  try {
    pub_pose_.publish(newPose);
  } catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_pose_.getTopic().c_str());
  }

  // publish TF
  geometry_msgs::Quaternion orientation = new_odom.pose.pose.orientation;
  geometry_msgs::Point position = new_odom.pose.pose.position;
  try {
    broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w),tf::Vector3(position.x,position.y,position.z)), new_odom.header.stamp, "local_origin", string("fcu_")+uav_name));
  } catch (...) {
    ROS_ERROR("Exception caught during publishing TF.");
  }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "mrs_odom");
  ROS_INFO ("Node initialized.");
  mrsOdometry odom;
  ros::Rate loop_rate(odom.rate_);

  while (ros::ok()) {

    odom.publishMessage();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};
