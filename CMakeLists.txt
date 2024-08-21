cmake_minimum_required(VERSION 3.0.2)
project(CoCEL_Handheld_DataRecorder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  livox_ros_driver
)

find_package(OpenCV REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  record_flag.srv
  save_bag_path.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES OpenCV
  CATKIN_DEPENDS rospy roscpp std_msgs cv_bridge sensor_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(${PROJECT_NAME}_handler src/handler.cpp)
target_link_libraries(${PROJECT_NAME}_handler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(handler_node
  src/main.cpp
)
target_link_libraries(handler_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}_handler
)

