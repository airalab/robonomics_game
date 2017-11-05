cmake_minimum_required(VERSION 2.8.3)
project(robonomics_game_plant)
find_package(catkin REQUIRED
  genmsg
  actionlib_msgs
  actionlib
)

add_action_files(
  DIRECTORY
  action
  FILES
  Order.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robonomics_game_plant
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
