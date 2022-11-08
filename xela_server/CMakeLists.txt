cmake_minimum_required(VERSION 2.8.3)
project(xela_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rosbag
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  SensPoint.msg
  xSensorData.msg
  xServerMsg.msg
)

add_service_files(
  FILES
  XelaSensorX.srv
  XelaSensorY.srv
  XelaSensorZ.srv
  XelaSensorXY.srv
  XelaSensorXYZ.srv
  XelaSensorStream.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
add_executable(xela_col scripts/sending.cpp)
add_dependencies(xela_col ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xela_col
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(PROGRAMS
  scripts/xela_server
  scripts/xela_service
  scripts/socket_prog
  launch/service.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
