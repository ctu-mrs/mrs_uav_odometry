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
  mrs_lib
  cmake_modules
  mrs_msgs
  nodelet
  dynamic_reconfigure
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
  config/lkf.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Odometry
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_lib mrs_msgs
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

add_library(range_filter src/range_filter.cpp)

## Declare a C++ executable
add_library(Odometry src/odometry.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(Odometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  range_filter
  )

## Add configure headers for dynamic reconfigure
add_dependencies(Odometry ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
add_library(RtkRepublisher src/rtk_republisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(RtkRepublisher
  ${catkin_LIBRARIES}
  )
