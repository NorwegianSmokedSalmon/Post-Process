cmake_minimum_required(VERSION 3.0.2)
project(bag2pcd)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  pcl_ros
  rosbag
)
find_package(PCL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  # CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES bag2pcd
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  include 
  src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(bag2pcd src/bag2pcd/bag2pcd.cpp)
target_link_libraries(bag2pcd jsoncpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})# ${SERIAL_LIB}

add_executable(topic2pcd src/topic2pcd/topic2pcd.cpp)
target_link_libraries(topic2pcd jsoncpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})# ${SERIAL_LIB}

add_executable(view_pcd src/view_pcd/view_pcd.cpp)
target_link_libraries(view_pcd jsoncpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})# ${SERIAL_LIB}

add_executable(trans2error src/trans2error/trans2error.cpp)
target_link_libraries(trans2error jsoncpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})# ${SERIAL_LIB}

# add_library(servo_run_lib src/main.cpp
#                           src/SCSCL.cpp
#                           src/SCSerial.cpp   
# )
