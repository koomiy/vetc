cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    sf_to_mpc.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    mpc_bw_nlp.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
)
