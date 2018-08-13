cmake_minimum_required(VERSION 2.8.3)
project(bebop_tools)

find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
  rospy
)

catkin_package(
  #  INCLUDE_DIRS include
 CATKIN_DEPENDS bebop_msgs rospy
)

catkin_install_python(
 PROGRAMS nodes/autoflight_joy.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO: Install Rules
