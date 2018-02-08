cmake_minimum_required(VERSION 2.8.3)
project(robonomics_consumers)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Call.srv
)
