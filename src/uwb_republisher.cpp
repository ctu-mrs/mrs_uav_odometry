/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <positioning_systems_ros/RtlsTrackerFrame.h>

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

/* class UWBRepublisher //{ */

class UWBRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

private:
  // subscribers and publishers
  ros::Subscriber    subscriber_uwb_;
  ros::Publisher     pose_publisher_;

private:
  void callbackUWBPosition(const positioning_systems_ros::RtlsTrackerFrame &msg);

};

//}

/* onInit() //{ */

void UWBRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  // | ----------------------- subscribers ---------------------- |

  subscriber_uwb_ = nh_.subscribe("uwb_in", 1, &UWBRepublisher::callbackUWBPosition, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_out", 1);

  is_initialized_ = true;

  ROS_INFO("[UWBRepublisher]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackUWBPose() //{ */

void UWBRepublisher::callbackUWBPosition(const positioning_systems_ros::RtlsTrackerFrame &msg) {

  if (!is_initialized_)
    return;

  if (msg.position.x == 0) {
    return;
  }

  geometry_msgs::PoseStamped pose_out;
  pose_out.header.frame_id = "map";
  pose_out.header.stamp = ros::Time::now();
  pose_out.pose.position = msg.position;
  pose_out.pose.orientation.w = 1.0;

  try {
    pose_publisher_.publish(pose_out);
  } catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pose_publisher_.getTopic().c_str());
  }

}

//}

}  // namespace mrs_uav_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_odometry::UWBRepublisher, nodelet::Nodelet)
