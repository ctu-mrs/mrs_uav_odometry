/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>

#include <mrs_msgs/Bestpos.h>

#include <std_srvs/Trigger.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <support.h>

//}

#define btoa(x) ((x) ? "true" : "false")

namespace mrs_uav_odometry
{

/* class RtkRepublisher //{ */

class RtkRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

private:
  // subscribers and publishers
  ros::Subscriber    subscriber_global_odom_;
  ros::Subscriber    subscriber_tersus_;
  ros::Publisher     rtk_publisher_;
  ros::Publisher     odom_publisher;
  ros::Publisher     pose_publisher_;
  ros::ServiceServer service_server_jump_emulation_;
  ros::ServiceServer service_server_random_jumps_;

  // publisher rate
  int _rate_;

  double _offset_x_, _offset_y_;

  double jump_offset_, jump_hdg_offset_;

  // simulation of RTK signal degradation
  bool   add_random_jumps_;
  bool   random_jump_active_;
  double random_jump_;
  int    until_next_jump_;
  int    until_jump_end_;
  double jump_amplitude_;

  mrs_msgs::RtkFixType fix_type_;

  // mutex for locking the position info
  std::mutex mutex_odom_;
  std::mutex mutex_tersus_;

  ros::Timer timer_main_;

  // global odometry from gazebo
  nav_msgs::Odometry odom_;
  bool               got_odom_ = false;

  // republished pose message
  geometry_msgs::PoseWithCovarianceStamped pose_msg_out_;

  // rtk message from tersus_
  mrs_msgs::Bestpos tersus_;
  bool              got_tersus_ = false;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void callbackTersus(const mrs_msgs::BestposConstPtr &msg);
  bool emulateJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool toggleRandomJumps([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  void timerMain(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler *profiler_;
  bool               profiler_enabled_ = false;
};

//}

/* onInit() //{ */

void RtkRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(nh_, "RtkRepublisher");

  param_loader.loadParam("enable_profiler", profiler_enabled_);

  param_loader.loadParam("rate", _rate_);
  param_loader.loadParam("offset_x", _offset_x_);
  param_loader.loadParam("offset_y", _offset_y_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RtkRepublisher]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_global_odom_ = nh_.subscribe("odom_in", 1, &RtkRepublisher::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_tersus_      = nh_.subscribe("tersus_in", 1, &RtkRepublisher::callbackTersus, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  rtk_publisher_  = nh_.advertise<mrs_msgs::RtkGps>("rtk_out", 1);
  pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 1);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_rate_), &RtkRepublisher::timerMain, this);

  // | ------------------------ services ------------------------ |

  service_server_jump_emulation_ = nh_.advertiseService("emulate_jump", &RtkRepublisher::emulateJump, this);
  service_server_random_jumps_   = nh_.advertiseService("toggle_random_jumps", &RtkRepublisher::toggleRandomJumps, this);

  // | ------------------------ profiler ------------------------ |

  profiler_ = new mrs_lib::Profiler(nh_, "RtkRepublisher", profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  jump_offset_     = 0.0;
  jump_hdg_offset_ = 0.0;

  add_random_jumps_   = false;
  random_jump_active_ = false;
  until_next_jump_    = 0.0;
  until_jump_end_     = 0.0;
  random_jump_        = 0.0;

  fix_type_.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

  is_initialized_ = true;

  ROS_INFO("[RtkRepublisher]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdometry() //{ */

void RtkRepublisher::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_->createRoutine("callbackOdometry");

  got_odom_ = true;

  {
    std::scoped_lock lock(mutex_odom_);

    odom_ = *msg;
  }
}

//}

/* callbackTersus() //{ */

void RtkRepublisher::callbackTersus(const mrs_msgs::BestposConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_->createRoutine("callbackTersus");

  {
    std::scoped_lock lock(mutex_tersus_);

    tersus_ = *msg;
  }
  mrs_msgs::RtkGps rtk_msg_out;

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = "gps";

  // copy the position
  {
    std::scoped_lock lock(mutex_tersus_);

    rtk_msg_out.gps.latitude      = tersus_.latitude;
    rtk_msg_out.gps.longitude     = tersus_.longitude;
    rtk_msg_out.gps.altitude      = tersus_.height;
    rtk_msg_out.gps.covariance[0] = std::pow(tersus_.latitude_std, 2);
    rtk_msg_out.gps.covariance[4] = std::pow(tersus_.longitude_std, 2);
    rtk_msg_out.gps.covariance[8] = std::pow(tersus_.height_std, 2);

    if (tersus_.position_type == "L1_INT" || tersus_.position_type == "NARROW_INT" || tersus_.position_type == "WIDE_INT") {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

    } else if (tersus_.position_type == "L1_FLOAT" || tersus_.position_type == "NARROW_FLOAT" || tersus_.position_type == "WIDE_FLOAT") {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FLOAT;

    } else if (tersus_.position_type == "PSRDIFF") {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::DGPS;

    } else if (tersus_.position_type == "SINGLE") {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::SPS;

    } else if (tersus_.position_type == "NONE") {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::NO_FIX;

    } else {

      rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::UNKNOWN;
    }
  }

  try {
    rtk_publisher_.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_msg_out)));
    ROS_INFO_ONCE("[RtkRepublisher]: publishing RTK from Tersus msgs.");
  }
  catch (...) {
    ROS_ERROR("[RtkRepublisher]: exception caught during publishing topic %s.", rtk_publisher_.getTopic().c_str());
  }
}

//}

/* emulateJump() //{ */

bool RtkRepublisher::emulateJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (jump_offset_ != 2.0) {
    jump_offset_       = 2.0;
    jump_hdg_offset_   = 1.0;
    fix_type_.fix_type = mrs_msgs::RtkFixType::SPS;
  } else {
    jump_offset_       = 0.0;
    jump_hdg_offset_   = 0.0;
    fix_type_.fix_type = mrs_msgs::RtkFixType::RTK_FIX;
  }

  ROS_INFO("[RtkRepublisher]: emulated jump: %f", jump_offset_);

  res.message = "yep";
  res.success = true;

  return true;
}

//}

/* toggleRandomJumps() //{ */

bool RtkRepublisher::toggleRandomJumps([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!add_random_jumps_) {
    add_random_jumps_ = true;
    until_next_jump_  = 3 * _rate_;
    until_jump_end_   = 5 * _rate_;
    jump_amplitude_   = 1.0;
  } else {
    add_random_jumps_ = false;
  }

  ROS_INFO("[Rtk_republisher]: random jumps: %s", btoa(add_random_jumps_));

  res.message = "yep";
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void RtkRepublisher::timerMain(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_->createRoutine("timerMain", _rate_, 0.01, event);

  mrs_msgs::RtkGps rtk_msg_out;

  if (!got_odom_) {

    return;
  }

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = "utm";

  // copy the position, orientation and velocity
  {
    std::scoped_lock lock(mutex_odom_);

    rtk_msg_out.pose  = odom_.pose;
    rtk_msg_out.twist = odom_.twist;

    pose_msg_out_.pose = odom_.pose;
    pose_msg_out_.pose.pose.position.x += jump_offset_;
    pose_msg_out_.pose.pose.position.y += jump_offset_;

    double hdg = mrs_lib::AttitudeConverter(pose_msg_out_.pose.pose.orientation).getHeading();
    pose_msg_out_.pose.pose.orientation = mrs_lib::AttitudeConverter(pose_msg_out_.pose.pose.orientation).setHeadingByYaw(hdg + jump_hdg_offset_);

    pose_msg_out_.header.stamp    = ros::Time::now();
    pose_msg_out_.header.frame_id = "local_origin";
  }

  rtk_msg_out.pose.pose.position.x += _offset_x_;
  rtk_msg_out.pose.pose.position.y += _offset_y_;

  rtk_msg_out.pose.pose.position.x += jump_offset_;
  rtk_msg_out.pose.pose.position.y += jump_offset_;

  rtk_msg_out.pose.pose.position.x += random_jump_;
  rtk_msg_out.pose.pose.position.y += random_jump_;

  rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
  rtk_msg_out.fix_type      = fix_type_;

  if (add_random_jumps_) {
    if (!random_jump_active_ && --until_next_jump_ <= 0) {
      random_jump_        = jump_amplitude_;
      random_jump_active_ = true;
      ROS_INFO("[RtkRepublisher]: jump %.2f added to RTK", random_jump_);
    }

    if (random_jump_active_ && --until_jump_end_ <= 0) {
      random_jump_        = 0.0;
      random_jump_active_ = false;
      until_next_jump_    = std::floor((double)std::rand() / RAND_MAX * 20.0 * _rate_);
      until_jump_end_     = std::floor((double)std::rand() / RAND_MAX * 10.0 * _rate_);
      jump_amplitude_     = std::floor((double)std::rand() / RAND_MAX * 5.0);
      ROS_INFO("[RtkRepublisher]: RTK jump ended. Next jump after %d samples, %d samples long, %.2f amplitude", until_next_jump_, until_jump_end_,
               jump_amplitude_);
    }
  }

  try {
    rtk_publisher_.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_msg_out)));
    ROS_INFO_ONCE("[RtkRepublisher]: publishing RTK from Gazebo simulator.");
  }
  catch (...) {
    ROS_ERROR("[RtkRepublisher]: exception caught during publishing topic %s.", rtk_publisher_.getTopic().c_str());
  }

  try {
    pose_publisher_.publish(geometry_msgs::PoseWithCovarianceStampedConstPtr(new geometry_msgs::PoseWithCovarianceStamped(pose_msg_out_)));
    ROS_INFO_ONCE("[RtkRepublisher]: publishing RTK from Gazebo simulator.");
  }
  catch (...) {
    ROS_ERROR("[RtkRepublisher]: exception caught during publishing topic %s.", pose_publisher_.getTopic().c_str());
  }
}

//}

}  // namespace mrs_uav_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_odometry::RtkRepublisher, nodelet::Nodelet)
