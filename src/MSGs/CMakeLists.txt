cmake_minimum_required(VERSION 3.0)
project(simulation_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

file(GLOB_RECURSE msg_files ${CMAKE_CURRENT_SOURCE_DIR}/msg ".msg")

add_message_files(
  ${msg_files}
)

file(GLOB srv_files ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
foreach(FILE ${srv_files})
  get_filename_component(n ${FILE} NAME)
  set(srv_file_names ${srv_file_names} ${n})
endforeach()

add_service_files(
  FILES
  ${srv_file_names}
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  message_generation
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)