cmake_minimum_required(VERSION 2.8.3)
project(path_planner)
set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#ROS
#=================================================================
find_package(catkin REQUIRED
	COMPONENTS 
	roscpp 
	rospy
	std_msgs
	sensor_msgs
	nav_msgs
	message_generation
	tf
	Base_control
)

find_package(PCL REQUIRED)

#=================================================================

#cmake settings
#=================================================================
#set output library directory;
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#message(STATUS "this is LIBRARY_OUTPUT_PATH " ${LIBRARY_OUTPUT_PATH})

#set output binary directory;
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is EXECUTABLE_OUTPUT_PATH " ${EXECUTABLE_OUTPUT_PATH})

#set runtime output directory;
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is CMAKE_RUNTIME_OUTPUT_DIRECTORY " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#set include files;
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
#message(STATUS "this is CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})

#set source files;
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)
#=================================================================

#===============================================================

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES path_planner
  #$CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs nav_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(app_path_planner ${PROJECT_SOURCES})
target_link_libraries(app_path_planner ${catkin_LIBRARIES})
add_dependencies(app_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
