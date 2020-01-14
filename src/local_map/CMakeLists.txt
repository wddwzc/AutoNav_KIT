cmake_minimum_required(VERSION 2.8.3)
project(local_map)
set(CMAKE_BUILD_TYPE RELEASE)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

#cmake settings
#=================================================================
#set output library directory;
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)#输出路径
#message(STATUS "this is LIBRARY_OUTPUT_PATH " ${LIBRARY_OUTPUT_PATH})

#set output binary directory;
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)#输出路径
#message(STATUS "this is EXECUTABLE_OUTPUT_PATH " ${EXECUTABLE_OUTPUT_PATH})

#set runtime output directory;
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is CMAKE_RUNTIME_OUTPUT_DIRECTORY " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#set include files;
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
#message(STATUS "this is CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})

#set source files;
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)
#=======


#add_service_files(FILES)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)



catkin_package(
  CATKIN_DEPENDS message_runtime  nav_msgs roscpp rospy std_msgs sensor_msgs
)

add_executable(app_local_map ${PROJECT_SOURCES})
## Specify libraries to link a library or executable target against
target_link_libraries(app_local_map ${catkin_LIBRARIES})
add_dependencies(app_local_map ${catkin_EXPORTED_TARGETS})




