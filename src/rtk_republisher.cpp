#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/RtkFixType.h>

#include <mrs_msgs/Bestpos.h>

#include <std_srvs/Trigger.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/Profiler.h>

#include <support.h>

#include <mutex>

#define STRING_EQUAL 0
#define btoa(x) ((x) ? "true" : "false")

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
    ros::Subscriber    global_odom_subscriber;
    ros::Subscriber    tersus_subscriber;
    ros::Publisher     rtk_publisher;
    ros::Publisher     odom_publisher;
    ros::Publisher     pose_publisher;
    ros::ServiceServer service_server_jump_emulation;
    ros::ServiceServer service_server_random_jumps;

    // publisher rate
    int rate_;

    double offset_x_, offset_y_;

    double jump_offset, jump_yaw_offset;

    // simulation of RTK signal degradation
    bool   add_random_jumps;
    bool   random_jump_active;
    double random_jump;
    int    until_next_jump;
    int    until_jump_end;
    double jump_amplitude;

    mrs_msgs::RtkFixType fix_type;

    // mutex for locking the position info
    std::mutex mutex_odom;
    std::mutex mutex_tersus;

    ros::Timer main_timer;

    // global odometry from gazebo
    nav_msgs::Odometry odom;
    bool               got_odom = false;

    // republished pose message 
    geometry_msgs::PoseWithCovarianceStamped pose_msg_out;

    // rtk message from tersus
    mrs_msgs::Bestpos tersus;
    bool              got_tersus = false;

  private:
    void callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
    void callbackTersus(const mrs_msgs::BestposConstPtr &msg);
    bool emulateJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool toggleRandomJumps([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void mainTimer(const ros::TimerEvent &event);

  private:
    mrs_lib::Profiler *profiler;
    bool               profiler_enabled_ = false;
  };

  //}

  // --------------------------------------------------------------
  // |                      internal routines                     |
  // --------------------------------------------------------------

  //{ onInit()

  void RtkRepublisher::onInit() {

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "RtkRepublisher");

    param_loader.load_param("enable_profiler", profiler_enabled_);

    param_loader.load_param("rate", rate_);
    param_loader.load_param("offset_x", offset_x_);
    param_loader.load_param("offset_y", offset_y_);

    // SUBSCRIBERS
    global_odom_subscriber = nh_.subscribe("odom_in", 1, &RtkRepublisher::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
    tersus_subscriber      = nh_.subscribe("tersus_in", 1, &RtkRepublisher::callbackTersus, this, ros::TransportHints().tcpNoDelay());

    // PUBLISHERS
    rtk_publisher = nh_.advertise<mrs_msgs::RtkGps>("rtk_out", 1);
    pose_publisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 1);

    // --------------------------------------------------------------
    // |                           timers                           |
    // --------------------------------------------------------------

    main_timer = nh_.createTimer(ros::Rate(rate_), &RtkRepublisher::mainTimer, this);

    // --------------------------------------------------------------
    // |                          services                          |
    // --------------------------------------------------------------

    service_server_jump_emulation = nh_.advertiseService("emulate_jump", &RtkRepublisher::emulateJump, this);
    service_server_random_jumps   = nh_.advertiseService("toggle_random_jumps", &RtkRepublisher::toggleRandomJumps, this);

    // --------------------------------------------------------------
    // |                          profiler                          |
    // --------------------------------------------------------------

    profiler = new mrs_lib::Profiler(nh_, "RtkRepublisher", profiler_enabled_);

    // | ----------------------- finish init ---------------------- |

    if (!param_loader.loaded_successfully()) {
      ROS_ERROR("[RtkRepublisher]: Could not load all parameters!");
      ros::shutdown();
    }

    jump_offset = 0.0;
    jump_yaw_offset = 0.0;

    add_random_jumps   = false;
    random_jump_active = false;
    until_next_jump    = 0.0;
    until_jump_end     = 0.0;
    random_jump        = 0.0;

    fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

    is_initialized = true;

    ROS_INFO("[RtkRepublisher]: initialized");
  }

  //}

  // --------------------------------------------------------------
  // |                          callbacks                         |
  // --------------------------------------------------------------

  //{ callbackOdometry()

  void RtkRepublisher::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackOdometry");

    got_odom = true;

    {
      std::scoped_lock lock(mutex_odom);

      odom = *msg;
    }
  }

  //}

  //{ callbackTersus()

  void RtkRepublisher::callbackTersus(const mrs_msgs::BestposConstPtr &msg) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("callbackTersus");

    {
      std::scoped_lock lock(mutex_tersus);

      tersus = *msg;
    }
    mrs_msgs::RtkGps rtk_msg_out;

    rtk_msg_out.header.stamp    = ros::Time::now();
    rtk_msg_out.header.frame_id = "gps";

    // copy the position
    {
      std::scoped_lock lock(mutex_tersus);

      rtk_msg_out.gps.latitude      = tersus.latitude;
      rtk_msg_out.gps.longitude     = tersus.longitude;
      rtk_msg_out.gps.altitude      = tersus.height;
      rtk_msg_out.gps.covariance[0] = std::pow(tersus.latitude_std, 2);
      rtk_msg_out.gps.covariance[4] = std::pow(tersus.longitude_std, 2);
      rtk_msg_out.gps.covariance[8] = std::pow(tersus.height_std, 2);

      if (std::strcmp(tersus.position_type.c_str(), "L1_INT") == STRING_EQUAL || std::strcmp(tersus.position_type.c_str(), "NARROW_INT") == STRING_EQUAL ||
          std::strcmp(tersus.position_type.c_str(), "WIDE_INT") == STRING_EQUAL) {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;

      } else if (std::strcmp(tersus.position_type.c_str(), "L1_FLOAT") == STRING_EQUAL ||
                 std::strcmp(tersus.position_type.c_str(), "NARROW_FLOAT") == STRING_EQUAL ||
                 std::strcmp(tersus.position_type.c_str(), "WIDE_FLOAT") == STRING_EQUAL) {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FLOAT;

      } else if (std::strcmp(tersus.position_type.c_str(), "PSRDIFF") == STRING_EQUAL) {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::DGPS;

      } else if (std::strcmp(tersus.position_type.c_str(), "SINGLE") == STRING_EQUAL) {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::SPS;

      } else if (std::strcmp(tersus.position_type.c_str(), "NONE") == STRING_EQUAL) {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::NO_FIX;
      } else {
        rtk_msg_out.fix_type.fix_type = mrs_msgs::RtkFixType::UNKNOWN;
      }
    }

    try {
      rtk_publisher.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_msg_out)));
      ROS_INFO_ONCE("[RtkRepublisher]: Publishing RTK from Tersus msgs.");
    }
    catch (...) {
      ROS_ERROR("[RtkRepublisher]: Exception caught during publishing topic %s.", rtk_publisher.getTopic().c_str());
    }
  }

  //}

  /* emulateJump() //{ */
  bool RtkRepublisher::emulateJump([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (jump_offset != 2.0) {
      jump_offset       = 2.0;
      jump_yaw_offset   = 1.0;
      fix_type.fix_type = mrs_msgs::RtkFixType::SPS;
    } else {
      jump_offset       = 0.0;
      jump_yaw_offset   = 0.0;
      fix_type.fix_type = mrs_msgs::RtkFixType::RTK_FIX;
    }

    ROS_INFO("[RtkRepublisher]: Emulated jump: %f", jump_offset);

    res.message = "yep";
    res.success = true;

    return true;
  }
  //}

  /* toggleRandomJumps() //{ */
  bool RtkRepublisher::toggleRandomJumps([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    if (!add_random_jumps) {
      add_random_jumps = true;
      until_next_jump  = 3 * rate_;
      until_jump_end   = 5 * rate_;
      jump_amplitude   = 1.0;
    } else {
      add_random_jumps = false;
    }

    ROS_INFO("[Rtk_republisher]: Random jumps: %s", btoa(add_random_jumps));

    res.message = "yep";
    res.success = true;

    return true;
  }
  //}

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  //{ mainTimer()

  void RtkRepublisher::mainTimer(const ros::TimerEvent &event) {

    if (!is_initialized)
      return;

    mrs_lib::Routine profiler_routine = profiler->createRoutine("mainTimer", rate_, 0.01, event);

    mrs_msgs::RtkGps rtk_msg_out;

    if (!got_odom) {

      return;
    }

    rtk_msg_out.header.stamp    = ros::Time::now();
    rtk_msg_out.header.frame_id = "utm";

    // copy the position, orientation and velocity
    {
      std::scoped_lock lock(mutex_odom);

      rtk_msg_out.pose  = odom.pose;
      rtk_msg_out.twist = odom.twist;

      pose_msg_out.pose  = odom.pose;
      pose_msg_out.pose.pose.position.x += jump_offset;
      pose_msg_out.pose.pose.position.y += jump_offset;
      mrs_odometry::addYaw(pose_msg_out.pose.pose.orientation, jump_yaw_offset);
      pose_msg_out.header.stamp    = ros::Time::now();
      pose_msg_out.header.frame_id = "local_origin";
    }

    rtk_msg_out.pose.pose.position.x += offset_x_;
    rtk_msg_out.pose.pose.position.y += offset_y_;

    rtk_msg_out.pose.pose.position.x += jump_offset;
    rtk_msg_out.pose.pose.position.y += jump_offset;

    rtk_msg_out.pose.pose.position.x += random_jump;
    rtk_msg_out.pose.pose.position.y += random_jump;

    rtk_msg_out.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    rtk_msg_out.fix_type      = fix_type;

    if (add_random_jumps) {
      if (!random_jump_active && --until_next_jump <= 0) {
        random_jump        = jump_amplitude;
        random_jump_active = true;
        ROS_INFO("[RtkRepublisher]: Jump %f added to RTK", random_jump);
      }

      if (random_jump_active && --until_jump_end <= 0) {
        random_jump        = 0.0;
        random_jump_active = false;
        until_next_jump    = std::floor((double)std::rand() / RAND_MAX * 20.0 * rate_);
        until_jump_end     = std::floor((double)std::rand() / RAND_MAX * 10.0 * rate_);
        jump_amplitude     = std::floor((double)std::rand() / RAND_MAX * 5.0);
        ROS_INFO("[RtkRepublisher]: RTK jump ended. Next jump after %d samples, %d samples long, %f amplitude", until_next_jump, until_jump_end,
                 jump_amplitude);
      }
    }

    try {
      rtk_publisher.publish(mrs_msgs::RtkGpsConstPtr(new mrs_msgs::RtkGps(rtk_msg_out)));
      ROS_INFO_ONCE("[RtkRepublisher]: Publishing RTK from Gazebo simulator.");
    }
    catch (...) {
      ROS_ERROR("[RtkRepublisher]: Exception caught during publishing topic %s.", rtk_publisher.getTopic().c_str());
    }

    try {
      pose_publisher.publish(geometry_msgs::PoseWithCovarianceStampedConstPtr(new geometry_msgs::PoseWithCovarianceStamped(pose_msg_out)));
      ROS_INFO_ONCE("[RtkRepublisher]: Publishing RTK from Gazebo simulator.");
    }
    catch (...) {
      ROS_ERROR("[RtkRepublisher]: Exception caught during publishing topic %s.", pose_publisher.getTopic().c_str());
    }
  }

  //}

}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::RtkRepublisher, nodelet::Nodelet)
