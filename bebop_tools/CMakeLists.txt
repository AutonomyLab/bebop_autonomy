cmake_minimum_required(VERSION 2.8.3)
project(bebop_tools)

find_package(catkin REQUIRED COMPONENTS
  bebop_msgs
)

# catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
 CATKIN_DEPENDS bebop_msgs
)

#############
## Install ##
#############

# Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
