cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
  std_msgs
)


add_service_files(
  FILES
  AnglesConverter.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  controller_manager
  hardware_interface
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
