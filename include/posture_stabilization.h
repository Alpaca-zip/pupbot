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

#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>

#define X_OFFSET 61
#define Y_OFFSET 94

class postureStabilization
{
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  ros::Publisher _pub_stabilization_variable;
  ros::Publisher _pub_roll_LPF;
  ros::Publisher _pub_pitch_LPF;
  ros::Subscriber _imu_sub;
  ros::Subscriber _key_control_sub_posture_control;
  std_msgs::Float64 _roll_LPF, _pitch_LPF;
  std_msgs::Float64MultiArray _MV;
  std::vector<double> _buff_roll, _buff_pitch;
  std::string _imu_topic;
  bool _posture_control_on;
  int _imu_cnt;
  int _width;
  double _z_offset_LF_leg, _z_offset_LR_leg, _z_offset_RR_leg, _z_offset_RF_leg;
  double _M_LF_leg, _M_LR_leg, _M_RR_leg, _M_RF_leg;
  double _M1_LF_leg, _M1_LR_leg, _M1_RR_leg, _M1_RF_leg;
  double _e_LF_leg, _e_LR_leg, _e_RR_leg, _e_RF_leg;
  double _e1_LF_leg, _e1_LR_leg, _e1_RR_leg, _e1_RF_leg;
  double _e2_LF_leg, _e2_LR_leg, _e2_RR_leg, _e2_RF_leg;
  double _P;
  double _I;
  double _D;
  double _roll_sum, _pitch_sum;

public:
  postureStabilization();
  void controlLoop();
  void imuCallback(const sensor_msgs::Imu& msg);
  void quatToRPY(const geometry_msgs::Quaternion quat, double& roll, double& pitch, double& yaw);
  void postureControlCallback(const std_msgs::Bool& posture_control);
};
