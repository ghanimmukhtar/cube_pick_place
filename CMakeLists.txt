cmake_minimum_required(VERSION 2.8.3)
project(cube_pick_place)

find_package(catkin REQUIRED COMPONENTS
  crustcrawler_mover_utils
  crustcrawler_core_msgs
  crustcrawler_mover_utils
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cube_pick_place
  CATKIN_DEPENDS crustcrawler_core_msgs moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cube_pick_place src/cube_pick_place.cpp)
target_link_libraries(cube_pick_place ${catkin_LIBRARIES})
