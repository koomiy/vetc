cmake_minimum_required(VERSION 3.0.2)
project(nlp_solver)

add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  custom_msgs
)

find_package(Eigen3 3.3 REQUIRED
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  /usr/include/eigen3
  /usr/local/include
)

link_directories(
  /usr/local/lib
  /usr/local/lib64
)

add_executable(nlp_node
  src/main.cpp
  src/nlp_solver.cpp
)

target_link_libraries(nlp_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ipopt
  cppad_lib
)
