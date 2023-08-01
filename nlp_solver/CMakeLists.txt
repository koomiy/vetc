cmake_minimum_required(VERSION 3.0.2)
project(nlp_solver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
  custom_msgs
)

find_package(Eigen3 3.3 REQUIRED
)

catkin_package()

include_directories(
  include
  /usr/include/eigen3
  /usr/local/include/cppad
  ${catkin_INCLUDE_DIRS}
)

add_executable(nlp_node
  src/main.cpp
  src/nlp_solver.cpp
)

target_link_libraries(nlp_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  cppad
)

link_directories(
  /usr/include/eigen3
  /usr/local/include/cppad
)
