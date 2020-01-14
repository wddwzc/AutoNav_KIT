cmake_minimum_required(VERSION 2.8.3)
project(gps2odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gnss_driver
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs gnss_driver nav_msgs geometry_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps2odometry src/main.cpp)
target_link_libraries(gps2odometry ${catkin_LIBRARIES})
add_dependencies(gps2odometry ${PROJECT_NAME}_gencpp)
