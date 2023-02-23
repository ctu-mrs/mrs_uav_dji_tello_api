cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_tello_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  mrs_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  mrs_lib
  mrs_uav_hw_api
  tello_driver
  geometry_msgs
  rospy
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mavros_msgs REQUIRED)

set(LIBRARIES
  MrsUavHwTelloApi
  )

catkin_package(
  CATKIN_DEPENDS roscpp mrs_msgs sensor_msgs mrs_lib mrs_uav_hw_api geometry_msgs tello_driver mavros_msgs geometry_msgs rospy
  DEPENDS mavros_msgs
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  )

# Plugin

add_library(MrsUavHwTelloApi
  src/api.cpp
  )

add_dependencies(MrsUavHwTelloApi
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavHwTelloApi
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
