cmake_minimum_required(VERSION 2.8.3)
project(prac2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_bumpgo src/nodo_scan.cpp)

target_link_libraries(nodo_scan ${catkin_LIBRARIES})


roslint_cpp(
 src/nodo_scan.cpp
)
