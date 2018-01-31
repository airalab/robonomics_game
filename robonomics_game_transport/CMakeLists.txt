cmake_minimum_required(VERSION 2.8.3)
project(robonomics_game_transport)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib
  actionlib_msgs
)
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ConveyorDestination.srv
  ConveyorLoad.srv
  ConveyorProductReady.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Stacker.action
  Transport.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robonomics_game_transport
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(
    PROGRAMS
        scripts/color_sensor_node.py
        scripts/conveyor_node.py
        scripts/opcua_client.py
        scripts/stacker.py
        scripts/stacker_node.py
        scripts/storage_node.py
        scripts/supplier_node.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
