cmake_minimum_required(VERSION 2.8.3)
project(robonomics_game_warehouse)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Order.srv
  Place.srv
  EmptyAll.srv
  FillAll.srv
  QuantityAvailable.srv
  Content.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robonomics_game_warehouse
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(
    PROGRAMS
        scripts/warehouse.py
        scripts/warehouse_node.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
