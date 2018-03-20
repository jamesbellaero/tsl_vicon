cmake_minimum_required (VERSION 3.5)

project (tsl_writer)
SET(GCC_FLAGS "-O1 -g -std=c++11")
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
add_executable(tsl_writer src/tsl_writer.cpp)
target_link_libraries (tsl_writer ${catkin_LIBRARIES})
