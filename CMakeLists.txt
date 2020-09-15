cmake_minimum_required(VERSION 2.8.3)
project(encode)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenMP)

if (OPENMP_FOUND)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  "/usr/include/eigen3"
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(encode
  src/map_builder.cpp
  src/encode_ros.cpp
  src/node.cpp
)

target_link_libraries(encode
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS encode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

