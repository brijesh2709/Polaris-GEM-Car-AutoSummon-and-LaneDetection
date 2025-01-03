cmake_minimum_required(VERSION 2.8.3)
project(gem_pure_pursuit_sim)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  ackermann_msgs
  visualization_msgs
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

