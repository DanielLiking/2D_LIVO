cmake_minimum_required(VERSION 2.8.3)
project(r3dio)
#add_definitions(-w)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
  cv_bridge
  image_transport
)

find_package(Eigen3)
find_package(OpenCV 3.2 REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

include_directories(
  include
  ./src/tools/
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(r3dio_node  src/DIONode.cpp src/r3dio.cpp  )
target_link_libraries(r3dio_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


