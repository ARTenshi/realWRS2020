cmake_minimum_required(VERSION 2.8.3)
project(omni_navigation)

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

add_executable(omni_navigation_node
  src/omni_navigation_node.cpp)

add_dependencies(omni_navigation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(omni_navigation_node
  ${catkin_LIBRARIES}
)
