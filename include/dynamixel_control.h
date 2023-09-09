/**
 * Copyright (C) 2021-2023  Alpaca-zip
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

#pragma once

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <vector>

// These are the default values and are overridden by rosparam.
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

class dynamixelControl
{
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  ros::Subscriber _sub_LF_leg;
  ros::Subscriber _sub_LR_leg;
  ros::Subscriber _sub_RR_leg;
  ros::Subscriber _sub_RF_leg;
  std_msgs::Float64 _joint_pos[12];
  std::string _port_name_str;
  std::vector<int> _dxl_id_vector;
  bool _result;
  int _baudrate;
  int _model_number_int;
  uint8_t _dxl_id[12];
  uint16_t _model_number;
  const char* _log;
  const char* _port_name;
  DynamixelWorkbench _dxl_wb;

public:
  dynamixelControl();
  void controlLoop();
  void monitorLFLegCallback(const trajectory_msgs::JointTrajectory& LF_leg);
  void monitorLRLegCallback(const trajectory_msgs::JointTrajectory& LR_leg);
  void monitorRRLegCallback(const trajectory_msgs::JointTrajectory& RR_leg);
  void monitorRFLegCallback(const trajectory_msgs::JointTrajectory& RF_leg);
  void dxlInit();
  void dxlTorqueOn();
  void dxlAddSyncWriteHandler();
};
