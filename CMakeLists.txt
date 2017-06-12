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
  mrs_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES cross_detector
	CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_estimation mrs_msgs
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
  ${Eigen_INCLUDE_DIRS}
)

add_library(trgfilter src/trgfilter.cpp)

## Declare a C++ executable
add_executable(odometry src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odometry
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	trgfilter
)

## Declare a C++ executable
add_executable(rtk_republisher src/rtk_republisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rtk_republisher
	${catkin_LIBRARIES}
)
