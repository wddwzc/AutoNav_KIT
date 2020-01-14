cmake_minimum_required(VERSION 2.8.3)
project(Base_control)

set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_service_files(   
	FILES   
	ControlCmd.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

#生成头文件
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES Base_control
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs
  #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/   #头文件必须添加这句申明
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES) #.cpp必须添加这句申明（UART.cpp）


#---------------------需更新处------------------------
add_executable(app_Controlservice src/Base_Service.cpp)
target_link_libraries(app_Controlservice ${catkin_LIBRARIES})
add_dependencies(app_Controlservice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(Client1 src/Client1.cpp)
target_link_libraries(Client1 ${catkin_LIBRARIES})
add_dependencies(Client1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
