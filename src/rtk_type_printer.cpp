// some ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <diagnostic_msgs/DiagnosticArray.h>

// subscribers and publishers
ros::Subscriber subsriber;

// some std includes
#include <stdlib.h>
#include <stdio.h>

#include <mutex>
#include <thread>

diagnostic_msgs::DiagnosticArray diagnostics;
ros::Time last_time;
std::mutex mutex_diagnostics;
std::thread main_thread;
bool got_diagnostics = false;

// is called every time new Odometry comes in
void rtkCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& msg) {

  // take it and stick it into the file
  ROS_INFO_ONCE("Got first diagnostics.. ");

  got_diagnostics = true;

  mutex_diagnostics.lock();
  {
    diagnostics = *msg;
    last_time = ros::Time::now();
  }
  mutex_diagnostics.unlock();
}

void mainThread(void) {

  ros::Rate r(1.0);
  ROS_INFO("Main thread started...");

  while (ros::ok()) {

    r.sleep();

    if (!got_diagnostics)
      continue;

    // open the file
    FILE * f = fopen("/home/mrs/rtk_type.txt", "w");

    if (f == NULL) {
      ROS_ERROR("Cannot open the file");
    } else {

      if ((ros::Time::now() - last_time).toSec() > 5.0) {


      } else {

        mutex_diagnostics.lock();
        {
          fprintf(f, " ");
          fprintf(f, "%s", diagnostics.status[0].values[6].value.c_str());
          
        }
        mutex_diagnostics.unlock();
      }

      fflush(f);
      fclose(f);
    }
  }
}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "rtk_type_printer");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  // SUBSCRIBERS
  subsriber = nh_.subscribe("diagnostics", 1, &rtkCallback, ros::TransportHints().tcpNoDelay());

  ros::Time::waitForValid();

  last_time = ros::Time::now();

  main_thread = std::thread(&mainThread);

  // needed to make stuff work
  ros::spin();

  return 0;
}
