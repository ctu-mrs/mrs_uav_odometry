cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  mrs_lib
  cmake_modules
  mrs_msgs
  nodelet
  dynamic_reconfigure
  )

find_package(mavros_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# please, NEVER commit those alternative flags with specific overrides of optimization
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O0 -g")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  cfg/odometry_dynparam.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Odometry
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf2_geometry_msgs geometry_msgs mrs_lib mrs_msgs
  DEPENDS mavros_msgs Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(Odometry src/support.cpp src/StateEstimator.cpp src/AltitudeEstimator.cpp src/AltitudeEstimatorAloamGarm.cpp src/HeadingEstimator.cpp src/odometry.cpp)

target_link_libraries(Odometry
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_dependencies(Odometry
  ${PROJECT_NAME}_gencfg
  )

add_library(RtkRepublisher src/rtk_republisher.cpp src/support.cpp)

target_link_libraries(RtkRepublisher
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )

add_library(TFConnectorDummy src/tf_connector_dummy.cpp)

target_link_libraries(TFConnectorDummy
  ${catkin_LIBRARIES}
  ${mavros_msgs_LIBRARIES}
  )
