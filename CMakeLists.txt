cmake_minimum_required(VERSION 2.8.3)
project(cube_pick_place)

find_package(catkin REQUIRED COMPONENTS
  crustcrawler_core_msgs
  crustcrawler_mover_utils
  baxter_core_msgs
  baxter_mover_utils
  pr2_mover_utils
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
  rgbd_utils
  cafer_core
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

add_message_files(
        FILES
        pick_place_pose.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

find_package(PCL 1.7 REQUIRED)

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} /usr/local/include)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cube_pick_place
  CATKIN_DEPENDS
crustcrawler_core_msgs
moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs rgbd_utils baxter_core_msgs baxter_mover_utils 
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RGBD_UTILS_INCLUDE_DIR}
)

add_executable(cube_pick_place src/cube_pick_place.cpp)
target_link_libraries(cube_pick_place ${catkin_LIBRARIES})

add_executable(pick_and_place_python src/pick_and_place_python.cpp)
target_link_libraries(pick_and_place_python ${catkin_LIBRARIES})

add_executable(pick_and_place_pose src/pick_and_place_pose.cpp)
target_link_libraries(pick_and_place_pose ${catkin_LIBRARIES})

add_executable(baxter_pick_place src/baxter_pick_place.cpp)
target_link_libraries(baxter_pick_place ${catkin_LIBRARIES})

add_executable(crustcrawler_pick_place src/crustcrawler_pick_place.cpp)
target_link_libraries(crustcrawler_pick_place ${catkin_LIBRARIES})

add_executable(moveit_pick_place_tests src/moveit_pick_place_tests.cpp)
target_link_libraries(moveit_pick_place_tests ${catkin_LIBRARIES})

add_executable(get_rpy src/get_rpy.cpp)
target_link_libraries(get_rpy ${catkin_LIBRARIES})

#add_executable(cube_pos_pub src/cube_pos_pub.cpp)
#target_link_libraries(cube_pos_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES}  image_processing yaml-cpp.so iagmm tbb)

add_executable(calvin_pick_n_place src/calvin_pick_n_place.cpp)
target_link_libraries(calvin_pick_n_place ${catkin_LIBRARIES} )

add_executable(test_class src/test_class.cpp)
target_link_libraries(test_class ${catkin_LIBRARIES} )
