cmake_minimum_required(VERSION 2.8.3)
project(speed_mannager)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  navig_msgs
  roscpp
  rospy
  std_msgs
  tf
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(speed_mannager_node
  src/speed_mannager_node.cpp)

add_dependencies(speed_mannager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(speed_mannager_node
  ${catkin_LIBRARIES}
)
