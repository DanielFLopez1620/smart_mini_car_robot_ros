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

catkin_install_python(PROGRAMS 
scripts/demo_elegoo_car.py
scripts/servo_node.py
scripts/car_mov_node.py
scripts/give_me_eight.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
