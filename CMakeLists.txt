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
   script/six_axis_node.py
   script/ble_node.py
   script/eyes_node.py
   script/serial_node.py
   script/camera_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/plen.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
