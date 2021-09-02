cmake_minimum_required(VERSION 2.8.3)
project(mtracker)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf message_generation)

add_service_files(FILES Trigger.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include/mtracker
  CATKIN_DEPENDS roscpp geometry_msgs tf message_runtime
  DEPENDS system_lib
)

include_directories(
  #${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/mtracker
)

add_executable(mtracker
  src/mtracker.cpp  src/rs232.c
  include/mtracker/Robot.hpp
  include/mtracker/Serial.hpp
  include/mtracker/rs232.h
)
add_dependencies(mtracker mtracker_generate_messages_cpp)
target_link_libraries(mtracker ${catkin_LIBRARIES})


add_executable(controller
  src/controller.cpp
)
add_dependencies(controller mtracker_generate_messages_cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(simulator
  src/simulator.cpp
)
add_dependencies(simulator mtracker_generate_messages_cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
