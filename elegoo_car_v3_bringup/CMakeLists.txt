cmake_minimum_required(VERSION 3.0.2)
project(elegoo_car_v3_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  turtlesim
  genmsg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



