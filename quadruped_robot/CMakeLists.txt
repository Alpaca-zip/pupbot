cmake_minimum_required(VERSION 3.0.2)
project(quadruped_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_quadruped_robot src/move_quadruped_robot.cpp)
add_executable(quadruped_robot_controller src/quadruped_robot_controller.cpp)
target_link_libraries(move_quadruped_robot ${catkin_LIBRARIES})
target_link_libraries(quadruped_robot_controller ${catkin_LIBRARIES})