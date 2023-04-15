cmake_minimum_required(VERSION 3.0.2)
project(parc_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
)

## set(GeographicLib_DIR "$ENV{HOME}/Downloads/GeographicLib-2.2/BUILD")
## find_package(GeographicLib REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  RobotStatus.msg
  Weed.msg
  WeedDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  ## INCLUDE_DIRS include
  ## LIBRARIES gps2cartesian
  ## CATKIN_DEPENDS roscpp rospy message_runtime geometry_msgs
)

include_directories(
  ## include
  ${catkin_INCLUDE_DIRS}
  ## ${GeographicLib_INCLUDE_DIRS}
)

## add_library(gps2cartesian src/gps2cartesian.cpp
## )

## target_link_libraries(
##   gps2cartesian
##   ${catkin_LIBRARIES}
##   ${GeographicLib_LIBRARIES}
## )

## install(
##   TARGETS gps2cartesian
##   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
##   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )

## install(
##   DIRECTORY include/${PROJECT_NAME}/
##   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
## )