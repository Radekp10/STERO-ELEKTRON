cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  nav_msgs
  tf2_ros
  tf
  costmap_2d
  global_planner
  nav_core
  dwa_local_planner
  rotate_recovery
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  Velocity.srv
  Lab1_2.srv
  Projekt2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
#   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS  message_runtime
   DEPENDS elektron 
)

add_executable(
  projekt2_server projekt2.cpp
)


target_link_libraries(projekt2_server
   ${catkin_LIBRARIES}
 )

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
