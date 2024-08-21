cmake_minimum_required(VERSION 3.0.2)
project(pubpsd_pkg)

SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
)
find_package(OpenCV 3 REQUIRED)
find_Package(Ceres REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/ 
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  "/usr/local/include/eigen3/"
)

catkin_package()

add_executable(pubpsd_node main/main.cpp src/read_serial_thread.cpp)
add_dependencies(pubpsd_node ${pubpsd_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(pubpsd_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Threads::Threads )
