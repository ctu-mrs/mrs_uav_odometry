// some ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/RtkGpsLocal.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <thread>

// subscribers and publishers
ros::Subscriber global_odom_subscriber;
ros::Publisher rtk_publisher;

// publisher rate
int rate_;

// offset of the simulation world
double offset_x_, offset_y_;

// mutex for locking the position info
std::mutex mutex_odom;

// main thread
std::thread main_thread;

// global pose from gazebo
nav_msgs::Odometry odom;
bool got_odom = false;

// is called every time new Odometry comes in
void odomCallback(const nav_msgs::OdometryConstPtr& msg) {

  got_odom = true;

  mutex_odom.lock();
  {
    odom = *msg;
  }
  mutex_odom.unlock();
}

void mainThread(void) {

  ros::Rate r(rate_);

  mrs_msgs::RtkGpsLocal pose;

  while (ros::ok()) {

    if (!got_odom) {

      r.sleep();
      continue;
    }

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "utm";
    mutex_odom.lock();
    {
      pose.position = odom.pose.pose.position;
    }

    pose.position.x += offset_x_;
    pose.position.y += offset_y_;

    pose.rtk_fix = true;
    
    mutex_odom.unlock();

    rtk_publisher.publish(pose);

    r.sleep();
  }
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "rtk_republisher");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  nh_.param("rate", rate_, 5);
  nh_.param("offset_x", offset_x_, 0.0);
  nh_.param("offset_y", offset_y_, 0.0);

  // SUBSCRIBERS
  global_odom_subscriber = nh_.subscribe("odom_in", 1, &odomCallback, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  rtk_publisher = nh_.advertise<mrs_msgs::RtkGpsLocal>("rtk_out", 1); 

  // launch the thread
  main_thread = std::thread(&mainThread);

  // needed to make stuff work
  ros::spin();

  return 0;
}
