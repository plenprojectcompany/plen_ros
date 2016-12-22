cmake_minimum_required(VERSION 2.8.3)
project(plen_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  Eye.msg
  Eyes.msg)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_runtime)
