cmake_minimum_required(VERSION 3.5)
project(mrs_uav_dji_tello_api)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  rclpy
  rclcpp
  rclcpp_components
  geometry_msgs
  mavros_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_hw_api
  nav_msgs
  pluginlib
  sensor_msgs
  std_msgs
  backward_ros
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavHwDjiTelloApi_Api
  )

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_uav_hw_api_INCLUDE_DIRS}
  )

# Plugin

add_library(MrsUavHwDjiTelloApi_Api SHARED
  src/api.cpp
  )

ament_target_dependencies(MrsUavHwDjiTelloApi_Api
  ${DEPENDENCIES}
)

target_compile_definitions(MrsUavHwDjiTelloApi_Api PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${PROJECT_NAME}/tellopy_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
