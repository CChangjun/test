cmake_minimum_required(VERSION 3.0.2)
project(chang_serial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  serial_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chang_serial
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs serial tf
  DEPENDS system_lib
)##catkin_depends에 serial 추가 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_core src/serial_core.cpp)
target_link_libraries(serial_core ${catkin_LIBRARIES})
add_dependencies(serial_core chang_serial_generate_messages_cpp)

add_executable(chang_core src/chang_core.cpp)
target_link_libraries(chang_core ${catkin_LIBRARIES})
add_dependencies(chang_core chang_serial_generate_messages_cpp)

add_executable(ing src/ing.cpp)
target_link_libraries(ing ${catkin_LIBRARIES})
add_dependencies(ing chang_serial_generate_messages_cpp)