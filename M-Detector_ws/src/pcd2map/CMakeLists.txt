cmake_minimum_required(VERSION 3.0.2)
project(pcd2map)
SET(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

add_definitions(-DPcd2map_ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pcd2map
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd2map
  src/pcd2map.cpp
  include/ikd-Tree/ikd_Tree.cpp
  )

target_link_libraries(pcd2map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
)
