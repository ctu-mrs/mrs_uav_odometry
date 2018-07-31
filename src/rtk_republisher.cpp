#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>
#include <nav_msgs/Odometry.h>
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

  nh_.param("rate", rate_, -1);

  if (rate_ < 0) {
    ROS_INFO("[RtkRepublisher]: the 'rate' parameter was not specified!"); 
    return;
  }

  // SUBSCRIBERS
  global_odom_subscriber = nh_.subscribe("odom_in", 1, &RtkRepublisher::odomCallback, this, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  rtk_publisher = nh_.advertise<mrs_msgs::RtkGps>("rtk_out", 1);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(rate_), &RtkRepublisher::mainTimer, this);

  ROS_INFO("[RtkRepublisher]: [%s]: initialized", ros::this_node::getName().c_str());
}

// is called every time new Odometry comes in
void RtkRepublisher::odomCallback(const nav_msgs::OdometryConstPtr& msg) {

  got_odom = true;

  mutex_odom.lock();
  { odom = *msg; }
  mutex_odom.unlock();
}

void RtkRepublisher::mainTimer(const ros::TimerEvent& event) {

  mrs_msgs::RtkGps rtk_msg_out;

  if (!got_odom) {

    return;
  }

  rtk_msg_out.header.stamp    = ros::Time::now();
  rtk_msg_out.header.frame_id = "utm";

  // copy the position, orientation and velocity
  mutex_odom.lock();
  {
    rtk_msg_out.pose     = odom.pose.pose;
    rtk_msg_out.velocity = odom.twist.twist;
  }
  mutex_odom.unlock();

  rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
  rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

  rtk_publisher.publish(rtk_msg_out);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::RtkRepublisher, nodelet::Nodelet)
