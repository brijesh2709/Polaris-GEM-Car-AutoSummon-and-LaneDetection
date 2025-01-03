cmake_minimum_required(VERSION 3.0.2)
project(gem_gnss_control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  novatel_gps_msgs
  nav_msgs
  std_msgs
  tf
  message_generation
)


add_message_files(
  FILES
  GpsCoordinates.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


