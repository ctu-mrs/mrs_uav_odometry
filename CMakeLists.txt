cmake_minimum_required(VERSION 2.8.3)
project(mrs_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf
  mrs_estimation
  cmake_modules
  quadrotor_msgs
  mrs_nmea_navsat_driver
  object_detection
  cross_detector
)

set(CMAKE_CXX_FLAGS "-std=c++11")

#find_package(Eigen3)
#find_package(PkgConfig)
#pkg_search_module(Eigen3 REQUIRED eigen3)
#if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
#endif()

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   cross_pos.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES cross_detector
	CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_estimation mrs_nmea_navsat_driver object_detection
#  DEPENDS system_libi
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(detector
#   src/detector.cpp
#)

add_library(trgfilter src/trgfilter.cpp)

## Declare a C++ executable
add_executable(odometry src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odometry
	${catkin_LIBRARIES}
	${EIGEN3_LIBRARIES}
	trgfilter
)

## Declare a C++ executable
add_executable(rtk_republisher src/rtk_republisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rtk_republisher
	${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(rtk_type_printer src/rtk_type_printer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rtk_type_printer
	${catkin_LIBRARIES}
)
