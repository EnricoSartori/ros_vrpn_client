cmake_minimum_required(VERSION 2.8.3)
project(ros_vrpn_client)

find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs cmake_modules)

catkin_package(
  INCLUDE_DIRS $ENV{VRPN_ROOT}
)

include_directories($ENV{VRPN_ROOT})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_vrpn_client
  src/ros_vrpn_client.cpp
  $ENV{VRPN_ROOT}/vrpn_Connection.C
  $ENV{VRPN_ROOT}/vrpn_Tracker.C
  $ENV{VRPN_ROOT}/vrpn_Shared.C
  $ENV{VRPN_ROOT}/vrpn_FileConnection.C
  $ENV{VRPN_ROOT}/vrpn_BaseClass.C
  $ENV{VRPN_ROOT}/vrpn_Serial.C
)

add_executable(ros_vrpn_hand
  src/ros_vrpn_hand.cpp
  $ENV{VRPN_ROOT}/vrpn_Connection.C
  $ENV{VRPN_ROOT}/vrpn_Tracker.C
  $ENV{VRPN_ROOT}/vrpn_Shared.C
  $ENV{VRPN_ROOT}/vrpn_FileConnection.C
  $ENV{VRPN_ROOT}/vrpn_BaseClass.C
  $ENV{VRPN_ROOT}/vrpn_Serial.C
)

target_link_libraries(ros_vrpn_client ${catkin_LIBRARIES})
target_link_libraries(ros_vrpn_hand ${catkin_LIBRARIES})
