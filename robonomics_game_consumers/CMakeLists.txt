cmake_minimum_required(VERSION 2.8.3)
project(robonomics_game_consumers)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Call.srv
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/consumers_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/consumers_ask.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
