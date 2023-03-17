#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */
#include <ros/publisher.h>

#include <geometry_msgs/Quaternion.h>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

//}

namespace mrs_uav_odometry
{

/* tf2FromPose() //{ */

tf2::Transform tf2FromPose(const geometry_msgs::Pose& pose_in) {

  tf2::Vector3 position(pose_in.position.x, pose_in.position.y, pose_in.position.z);

  tf2::Quaternion q;
  tf2::fromMsg(pose_in.orientation, q);

  tf2::Transform tf_out;
  tf_out.setOrigin(position);
  tf_out.setRotation(q);
  tf_out.inverse();

  return tf_out;
}

//}

/* poseFromTf2() //{ */

geometry_msgs::Pose poseFromTf2(const tf2::Transform& tf_in) {

  geometry_msgs::Pose pose_out;
  pose_out.position.x = tf_in.getOrigin().getX();
  pose_out.position.y = tf_in.getOrigin().getY();
  pose_out.position.z = tf_in.getOrigin().getZ();

  pose_out.orientation = tf2::toMsg(tf_in.getRotation());

  return pose_out;
}

//}

/* pointToVector3() //{ */

geometry_msgs::Vector3 pointToVector3(const geometry_msgs::Point& point_in) {

  geometry_msgs::Vector3 vec_out;
  vec_out.x = point_in.x;
  vec_out.y = point_in.y;
  vec_out.z = point_in.z;

  return vec_out;
}

//}

/* distance() //{ */
double distance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
  return std::sqrt(pow(odom1.pose.pose.position.x - odom2.pose.pose.position.x, 2) + pow(odom1.pose.pose.position.y - odom2.pose.pose.position.y, 2) +
                   pow(odom1.pose.pose.position.z - odom2.pose.pose.position.z, 2));
}
//}

/* toLowercase //{ */

std::string toLowercase(const std::string str_in) {
  std::string str_out = str_in;
  std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::tolower);
  return str_out;
}

//}

/* toUppercase //{ */

std::string toUppercase(const std::string str_in) {
  std::string str_out = str_in;
  std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::toupper);
  return str_out;
}

//}

/* tryPublish() //{ */

template <typename MsgType>
void tryPublish(const ros::Publisher& pub, const MsgType& msg) {

  try {
    pub.publish(msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub.getTopic().c_str());
  }
}

//}

/* noNans() //{ */
bool noNans(const geometry_msgs::TransformStamped& tf) {

  return (std::isfinite(tf.transform.rotation.z) && std::isfinite(tf.transform.translation.x) && std::isfinite(tf.transform.translation.z));
}
//}

/* noNans() //{ */
bool noNans(const geometry_msgs::Quaternion& q) {

  return (std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w));
}
//}

/* isZeroQuaternion() //{ */
bool isZeroQuaternion(const geometry_msgs::Quaternion& q) {

  return (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0);
}
//}

/* publishTFFromOdom() //{*/
void publishTFFromOdom(const nav_msgs::Odometry& odom, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster, const ros::Time& time) {

  tf2::Transform tf, tf_inv;
  tf     = tf2FromPose(odom.pose.pose);
  tf_inv = tf.inverse();

  geometry_msgs::Pose pose_inv;
  pose_inv = mrs_uav_odometry::poseFromTf2(tf_inv);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.transform.translation = pointToVector3(pose_inv.position);
  tf_msg.transform.rotation    = pose_inv.orientation;

  if (noNans(tf_msg)) {

    tf_msg.header.stamp    = time;
    tf_msg.header.frame_id = odom.child_frame_id;
    tf_msg.child_frame_id  = odom.header.frame_id;

    try {
      broadcaster->sendTransform(tf_msg);
    }
    catch (...) {
      ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf_msg.child_frame_id.c_str(), tf_msg.header.frame_id.c_str());
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[Odometry]: NaN detected in transform from %s to %s. Not publishing tf.", tf_msg.header.frame_id.c_str(),
                      tf_msg.child_frame_id.c_str());
  }
}

void publishTFFromOdom(const nav_msgs::Odometry& odom, const std::shared_ptr<mrs_lib::TransformBroadcaster>& broadcaster) {
  publishTFFromOdom(odom, broadcaster, ros::Time::now());
}

/*//}*/

}  // namespace mrs_uav_odometry

#endif
