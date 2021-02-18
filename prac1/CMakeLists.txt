cmake_minimum_required(VERSION 2.8.3)
project(prac1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_bump_vel src/nodo_bump_vel.cpp)

target_link_libraries(nodo_bump_vel ${catkin_LIBRARIES})

roslint_cpp(
  src/nodo_bump_vel.cpp
)
