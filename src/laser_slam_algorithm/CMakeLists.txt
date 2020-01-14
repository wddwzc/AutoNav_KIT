cmake_minimum_required(VERSION 2.8.3)
project(laser_slam_algorithm)

set(CMAKE_BUILD_TYPE Release)
#set(BUILD_STATIC_LIBS ON)
#set(BUILD_SHARED_LIBS OFF)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


 add_service_files(
  DIRECTORY srv
  FILES
   srv_record_map.srv
   srv_relocalization.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

 
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES laser_slam_algorithm
  CATKIN_DEPENDS pcl_ros roscpp std_msgs message_runtime sensor_msgs
  #DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)



add_executable(register src/register.cpp)
add_executable(odom src/odom.cpp)
add_executable(mapping src/mapping.cpp)
add_executable(transform src/transform.cpp)
add_executable(relocalization src/relocalization.cpp)

target_link_libraries(register ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(odom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(mapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(relocalization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(relocalization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

