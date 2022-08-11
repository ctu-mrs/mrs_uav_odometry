#include <nodelet/nodelet.h>

#include <chrono>
#include <thread>

#include <estimators/lateral/gps.h>
#include <estimators/altitude/garmin.h>


namespace mrs_odometry
{

/*//{ class OdometryTesting */
class OdometryTesting : public nodelet::Nodelet {

private:
  bool is_initialized_ = false;

  std::unique_ptr<Gps>    gps_estimator_;
  std::unique_ptr<Garmin> garmin_estimator_;

public:
  virtual void onInit();
};
/*//}*/

/*//{ onInit() */
void OdometryTesting::onInit() {
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OdometryTesting]: initializing");

  is_initialized_ = true;

  gps_estimator_ = std::make_unique<mrs_odometry::Gps>();

  gps_estimator_->initialize(nh_);

  for (int i = 0; i < 10; i++) {
    if (gps_estimator_->isReady()) {
      ROS_INFO("[OdometryTesting]: starting GPS estimator");
      gps_estimator_->start();
      break;
    }
    ROS_INFO("[OdometryTesting]: waiting for GPS estimator to be ready");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  garmin_estimator_ = std::make_unique<mrs_odometry::Garmin>();

  garmin_estimator_->initialize(nh_);

  for (int i = 0; i < 10; i++) {
    if (garmin_estimator_->isReady()) {
      ROS_INFO("[OdometryTesting]: starting GARMIN estimator");
      garmin_estimator_->start();
      break;
    }
    ROS_INFO("[OdometryTesting]: waiting for GARMIN estimator to be ready");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  while (true) {

    ROS_INFO("[OdometryTesting]: running");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}
/*//}*/

}  // namespace mrs_odometry

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_odometry::OdometryTesting, nodelet::Nodelet)
