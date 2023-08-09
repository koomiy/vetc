cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

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
)

link_directories()

add_executable(mpc_node
  src/main.cpp
  src/mpc_controller.cpp
)

target_link_libraries(mpc_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_dependencies(mpc_node
  ${catkin_EXPORTED_TARGETS}
)
