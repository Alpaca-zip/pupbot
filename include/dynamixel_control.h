/**
 * Copyright (C) 2022  Alpaca-zip
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PORT_NAME "/dev/ttyACM1"
#define MODEL_NUMBER 12
#define BAUD_RATE 1000000

#define LF_LEG_SHOULDER_ID 3
#define LF_LEG_UPPER_ID 21
#define LF_LEG_LOWER_ID 5
#define LR_LEG_SHOULDER_ID 16
#define LR_LEG_UPPER_ID 8
#define LR_LEG_LOWER_ID 6
#define RR_LEG_SHOULDER_ID 7
#define RR_LEG_UPPER_ID 18
#define RR_LEG_LOWER_ID 14
#define RF_LEG_SHOULDER_ID 2
#define RF_LEG_UPPER_ID 1
#define RF_LEG_LOWER_ID 13

class Dynamixel_Control{
  public:
  Dynamixel_Control();
  void controlLoop();

  private:
  const char *log;
  const char* port_name = PORT_NAME;
  int baud_rate;
  int32_t goal_position[12];
  uint16_t model_number;
  uint8_t dxl_id[12] = {LF_LEG_SHOULDER_ID, LF_LEG_UPPER_ID, LF_LEG_LOWER_ID, LR_LEG_SHOULDER_ID, LR_LEG_UPPER_ID, LR_LEG_LOWER_ID, RR_LEG_SHOULDER_ID, RR_LEG_UPPER_ID, RR_LEG_LOWER_ID, RF_LEG_SHOULDER_ID, RF_LEG_UPPER_ID, RF_LEG_LOWER_ID};
  bool result;
  const uint8_t handler_index = 0;
  DynamixelWorkbench dxl_wb;

  ros::NodeHandle nh;
  ros::Subscriber sub_LF_leg;
  ros::Subscriber sub_LR_leg;
  ros::Subscriber sub_RR_leg;
  ros::Subscriber sub_RF_leg;
  std_msgs::String joint_name[12];
  std_msgs::Float64 joint_pos[12];

  void init();
  void monitor_LF_leg_callback(const trajectory_msgs::JointTrajectory& LF_leg);
  void monitor_LR_leg_callback(const trajectory_msgs::JointTrajectory& LR_leg);
  void monitor_RR_leg_callback(const trajectory_msgs::JointTrajectory& RR_leg);
  void monitor_RF_leg_callback(const trajectory_msgs::JointTrajectory& RF_leg);
  void dxl_init();
  void dxl_torqueOn();
  void dxl_addSyncWriteHandler();
};
