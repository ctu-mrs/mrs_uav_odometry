#ifndef MEASUREMENT_MANAGER_H
#define MEASUREMENT_MANAGER_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <lateral_measurement.h>
#include <measurement_buffer.h>

//}

namespace mrs_odometry
{

class MeasurementManager {

public:
  MeasurementManager(const ros::NodeHandle &parent_nh) {};
  ~MeasurementManager(void) {};

private:
  ros::Subscriber sub_odom_mavros_;
  ros::Subscriber sub_odom_vio_;
  ros::Subscriber sub_odom_lidar_;
  ros::Subscriber sub_odom_aloam_;

  ros::Subscriber sub_pose_hector_;
  ros::Subscriber sub_pose_vslam_;

  ros::Subscriber sub_twist_optflow_;

  ros::Subscriber sub_attitude_command_;
  ros::Subscriber sub_control_accel_;

public:
  void initialize(const ros::NodeHandle &parent_nh, const std::string& name, const std::string& uav_name);


  void getLateralMeasurements(MeasurementBuffer<LateralMeasurement>& lm);


  void callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg);

  void callbackVioOdom(const nav_msgs::OdometryConstPtr &msg);

  void callbackLidarOdom(const nav_msgs::OdometryConstPtr &msg);

  void callbackAloamOdom(const nav_msgs::OdometryConstPtr &msg);


  void callbackHectorPose(const geometry_msgs::PoseStampedConstPtr &msg);

  void callbackVslamPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);


  void callbackOptflowTwist(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);


  void callbackRtkGps(const mrs_msgs::RtkGpsConstPtr &msg);
  



  void callbackAttitudeCommand(const mrs_msgs::AttitudeCommandConstPtr &msg);

  void callbackControlAccel(const sensor_msgs::ImuConstPtr &msg);

};
}  // namespace mrs_odometry

#endif
