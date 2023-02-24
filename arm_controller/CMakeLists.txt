cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
  std_msgs
  message_generation
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


add_executable(angles_converter src/angles_converter.cpp)
add_dependencies(angles_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angles_converter ${catkin_LIBRARIES})

add_executable(arm_interface_node src/arm_interface.cpp)
add_dependencies(arm_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_interface_node ${catkin_LIBRARIES})