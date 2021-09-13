################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(pupbot)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  dynamixel_workbench_toolbox
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs trajectory_msgs
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_pupbot src/move_pupbot.cpp)
add_executable(pupbot_controller src/pupbot_controller.cpp)
add_executable(pupbot_controller_ds4 src/pupbot_controller_ds4.cpp)
add_executable(dynamixel_controller src/dynamixel_controller.cpp)
add_executable(pupbot_stabilizer src/pupbot_stabilizer.cpp)
target_link_libraries(move_pupbot ${catkin_LIBRARIES})
target_link_libraries(pupbot_controller ${catkin_LIBRARIES})
target_link_libraries(pupbot_controller_ds4 ${catkin_LIBRARIES})
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})
target_link_libraries(pupbot_stabilizer ${catkin_LIBRARIES})