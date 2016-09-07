cmake_minimum_required(VERSION 2.8.3)
project(bebop_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

# TODO: Tests

#############
## Install ##
#############

# Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Copy resources
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
