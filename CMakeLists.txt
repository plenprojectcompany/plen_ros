cmake_minimum_required(VERSION 2.8.3)
project(plen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  plen_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/six_axis_node.py
   scripts/ble_node.py
   scripts/eyes_node.py
   scripts/serial_node.py
   scripts/camera_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/plen.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
