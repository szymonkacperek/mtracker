cmake_minimum_required(VERSION 2.8.3)
project(mtracker)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
  message_generation
  urdf
  std_msgs
  sensor_msgs
  gazebo_msgs
)

add_service_files(FILES Trigger.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include/mtracker
  CATKIN_DEPENDS roscpp geometry_msgs tf message_runtime rospy
  DEPENDS system_lib
)

include_directories(
  # ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/mtracker
)

add_executable(
  real_robot
  src/real_robot.cpp
  src/rs232.c
  include/mtracker/Robot.hpp
  include/mtracker/Serial.hpp
  include/mtracker/rs232.h
)
add_dependencies(real_robot mtracker_generate_messages_cpp)
target_link_libraries(real_robot ${catkin_LIBRARIES})

add_executable(
  controller
  src/controller.cpp
)
add_dependencies(controller mtracker_generate_messages_cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(
  simulation
  src/simulation.cpp
  src/mtracker.cpp
)
add_dependencies(simulation mtracker_generate_messages_cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})

add_executable(
  rsg
  src/rsg.cpp
)
add_dependencies(rsg mtracker_generate_messages_cpp)
target_link_libraries(rsg ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/rsg_trajectory_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

