#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

namespace mrs_odometry
{

//{ class RtkRepublisher

class RtkRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

private:
  // subscribers and publishers
  ros::Subscriber global_odom_subscriber;
  ros::Publisher  rtk_publisher;
  ros::Publisher  odom_publisher;

  // publisher rate
  int rate_;

  double offset_x_, offset_y_;

  // mutex for locking the position info
  std::mutex mutex_odom;

  ros::Timer main_timer;

  // global pose from gazebo
  nav_msgs::Odometry odom;
  bool               got_odom = false;

private:
  void callbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  void mainTimer(const ros::TimerEvent& event);
};

//}

// --------------------------------------------------------------
// |                      internal routines                     |
// --------------------------------------------------------------

//{ onInit()

void RtkRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  nh_.param("rate", rate_, -1);
  nh_.param("offset_x", offset_x_, -10e15);
  nh_.param("offset_y", offset_y_, -10e15);

  if (rate_ < 0) {
    ROS_INFO("[RtkRepublisher]: the 'rate' parameter was not specified!");
    return;
  }

  if (offset_x_ < 0) {
    ROS_INFO("[RtkRepublisher]: the 'offset_x' parameter was not specified!");
    return;
  }

  if (offset_y_ < 0) {
    ROS_INFO("[RtkRepublisher]: the 'offset_y' parameter was not specified!");
    return;
  }

  // SUBSCRIBERS
  global_odom_subscriber = nh_.subscribe("odom_in", 1, &RtkRepublisher::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  rtk_publisher = nh_.advertise<mrs_msgs::RtkGps>("rtk_out", 1);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(rate_), &RtkRepublisher::mainTimer, this);

  ROS_INFO("[RtkRepublisher]: [%s]: initialized", ros::this_node::getName().c_str());

  is_initialized = true;
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

//{ callbackOdometry()

void RtkRepublisher::callbackOdometry(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized)
    return;

  got_odom = true;

  mutex_odom.lock();
  { odom = *msg; }
  mutex_odom.unlock();
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

//{ mainTimer()

void RtkRepublisher::mainTimer(const ros::TimerEvent& event) {

  if (!is_initialized)
    return;

  mrs_msgs::RtkGps rtk_msg_out;

  if (!got_odom) {

    return;
  }

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = "utm";

  // copy the position, orientation and velocity
  mutex_odom.lock();
  {
    rtk_msg_out.pose  = odom.pose;
    rtk_msg_out.twist = odom.twist;
  }
  mutex_odom.unlock();

  rtk_msg_out.pose.pose.position.x += offset_x_;
  rtk_msg_out.pose.pose.position.y += offset_y_;

  rtk_msg_out.status.status     = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
  rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

  try {
    rtk_publisher.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_msg_out)));
  } catch (...) {
    ROS_ERROR("[RtkRepublisher]: Exception caught during publishing topic %s.", rtk_publisher.getTopic().c_str());
  }
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::RtkRepublisher, nodelet::Nodelet)
