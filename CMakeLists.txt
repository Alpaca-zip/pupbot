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

add_executable(dynamixel_control src/dynamixel_control.cpp)
add_executable(inverse_kinematics src/inverse_kinematics.cpp)
add_executable(key_control src/key_control.cpp)
add_executable(posture_stabilization src/posture_stabilization.cpp)
add_executable(standing_motion src/standing_motion.cpp)
add_executable(trot_gait src/trot_gait.cpp)
target_link_libraries(dynamixel_control ${catkin_LIBRARIES})
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})
target_link_libraries(key_control ${catkin_LIBRARIES})
target_link_libraries(posture_stabilization ${catkin_LIBRARIES})
target_link_libraries(standing_motion ${catkin_LIBRARIES})
target_link_libraries(trot_gait ${catkin_LIBRARIES})
