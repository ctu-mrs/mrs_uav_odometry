#include "support.h"

/* tf2FromPose() //{ */

tf2::Transform mrs_odometry::tf2FromPose(const geometry_msgs::Pose& pose_in) {

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

/* posefromTf2() //{ */

geometry_msgs::Pose mrs_odometry::poseFromTf2(const tf2::Transform& tf_in) {

  geometry_msgs::Pose pose_out;
  pose_out.position.x = tf_in.getOrigin().getX();
  pose_out.position.y = tf_in.getOrigin().getY();
  pose_out.position.z = tf_in.getOrigin().getZ();

  pose_out.orientation = tf2::toMsg(tf_in.getRotation());

  return pose_out;
}

//}

/* pointToVector3() //{ */

geometry_msgs::Vector3 mrs_odometry::pointToVector3(const geometry_msgs::Point& point_in) {

  geometry_msgs::Vector3 vec_out;
  vec_out.x = point_in.x;
  vec_out.y = point_in.y;
  vec_out.z = point_in.z;

  return vec_out;
}

//}

/* distance() //{ */
double mrs_odometry::distance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2) {
  return std::sqrt(pow(odom1.pose.pose.position.x - odom2.pose.pose.position.x, 2) + pow(odom1.pose.pose.position.y - odom2.pose.pose.position.y, 2) +
                   pow(odom1.pose.pose.position.z - odom2.pose.pose.position.z, 2));
}
//}

/* toLowercase //{ */

std::string mrs_odometry::toLowercase(const std::string str_in) {
  std::string str_out = str_in;
  std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::tolower);
  return str_out;
}

//}

/* toUppercase //{ */

std::string mrs_odometry::toUppercase(const std::string str_in) {
  std::string str_out = str_in;
  std::transform(str_out.begin(), str_out.end(), str_out.begin(), ::toupper);
  return str_out;
}

//}

/* tryPublish() //{ */

template <typename MsgType>
void mrs_odometry::tryPublish(const ros::Publisher& pub, const MsgType& msg) {

  try {
    pub.publish(msg);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing topic %s.", pub.getTopic().c_str());
  }
}

//}

/* noNans() //{ */
bool mrs_odometry::noNans(const geometry_msgs::TransformStamped& tf) {

  return (std::isfinite(tf.transform.rotation.z) && std::isfinite(tf.transform.translation.x) && std::isfinite(tf.transform.translation.z));
}
//}
