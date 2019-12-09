#pragma once
#ifndef SUPPORT_H
#define SUPPORT_H

/* includes //{ */
#include <ros/publisher.h>

#include <geometry_msgs/Quaternion.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <nav_msgs/Odometry.h>

//}

namespace mrs_odometry {

void addRPY(geometry_msgs::Quaternion& q_msg, const double& roll_add, const double& pitch_add, const double& yaw_add); 

void getRPY(const geometry_msgs::Quaternion& q_msg_in, double& roll_out, double& pitch_out, double& yaw_out);

void setRPY(const double& roll_in, const double& pitch_in, const double& yaw_in, geometry_msgs::Quaternion& q_msg_out);

double getYaw(tf2::Quaternion &q_tf);

double getYaw(geometry_msgs::Quaternion &q_msg);

void setYaw(geometry_msgs::Quaternion& q_msg, const double& yaw_in);

void setYaw(tf2::Quaternion &q_msg, const double& yaw_in);

void addYaw(geometry_msgs::Quaternion& q_msg, const double& yaw_add);

double distance(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2); 

bool isEqual(const char* s1, const char* s2);
bool isEqual(const std::string& s1, const std::string& s2);
bool isEqual(const std::string& s1, const char* s2);
bool isEqual(const char* s1, const std::string& s2);

double unwrapAngle(const double yaw, const double yaw_previous);

double wrapAngle(const double angle_in);

double disambiguateAngle(const double yaw, const double yaw_previous);

template <typename MsgType>
void tryPublish(const ros::Publisher& pub, const MsgType& msg);

bool noNans(const geometry_msgs::TransformStamped& tf);

}

#endif
