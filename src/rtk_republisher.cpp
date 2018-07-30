#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/RtkGps.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>

namespace mrs_odometry
{

class RtkRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

private:
  // subscribers and publishers
  ros::Subscriber global_odom_subscriber;
  ros::Publisher  rtk_publisher;
  ros::Publisher  odom_publisher;

  // publisher rate
  int rate_;

  // offset of the simulation world
  double offset_x_, offset_y_;

  // mutex for locking the position info
  std::mutex mutex_odom;

  ros::Timer main_timer;

  // global pose from gazebo
  nav_msgs::Odometry odom;
  bool               got_odom = false;

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void mainTimer(const ros::TimerEvent& event);
};

// constructor
void RtkRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  nh_.param("rate", rate_, 5);
  nh_.param("offset_x", offset_x_, 0.0);
  nh_.param("offset_y", offset_y_, 0.0);

  // SUBSCRIBERS
  global_odom_subscriber = nh_.subscribe("odom_in", 1, &RtkRepublisher::odomCallback, this, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  rtk_publisher  = nh_.advertise<mrs_msgs::RtkGps>("rtk_out", 1);
  odom_publisher = nh_.advertise<nav_msgs::Odometry>("odometry_out", 1);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(rate_), &RtkRepublisher::mainTimer, this);

  NODELET_INFO("[RtkRepublisher]: [%s]: initialized", ros::this_node::getName().c_str());
}

// is called every time new Odometry comes in
void RtkRepublisher::odomCallback(const nav_msgs::OdometryConstPtr& msg) {

  got_odom                    = true;
  nav_msgs::Odometry odom_out = *msg;

  mutex_odom.lock();
  { odom = *msg; }
  mutex_odom.unlock();

  /* odom_out.pose.pose.position.x += offset_x_; */
  /* odom_out.pose.pose.position.y += offset_y_; */

  odom_out.header.frame_id = "local_origin";

  odom_publisher.publish(odom_out);
}

void RtkRepublisher::mainTimer(const ros::TimerEvent& event) {

  mrs_msgs::RtkGps rtk_msg_out;

  if (!got_odom) {

    return;
  }

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = "utm";
  mutex_odom.lock();
  {
    rtk_msg_out.pose = odom.pose.pose;
    rtk_msg_out.velocity = odom.twist.twist;
  }

  rtk_msg_out.pose.position.x += offset_x_;
  rtk_msg_out.pose.position.y += offset_y_;

  rtk_msg_out.rtk_fix = true;

  mutex_odom.unlock();

  rtk_publisher.publish(rtk_msg_out);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::RtkRepublisher, nodelet::Nodelet)
