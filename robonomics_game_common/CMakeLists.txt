cmake_minimum_required(VERSION 2.8.3)
project(robonomics_game_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Step.srv
)

generate_messages()

catkin_package()
