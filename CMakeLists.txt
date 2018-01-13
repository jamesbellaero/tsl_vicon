cmake_minimum_required (VERSION 3.5)

project (orbot_server)
SET(GCC_FLAGS "-g -std=c++11")
add_definitions(${GCC_FLAGS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)
catkin_package()
include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(orbot_server src/orbot_server.cpp)
target_link_libraries (orbot_server ${catkin_LIBRARIES})
