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

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

class StandingMotion
{
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  ros::Publisher _pub_leg_position;
  ros::Publisher _pub_stop_signal;
  ros::Subscriber _sub_standing_motion;
  std_msgs::Bool _stop;
  std_msgs::Float64MultiArray _leg_position;
  double _x_offset, _z_offset_LF_leg, _z_offset_LR_leg, _z_offset_RR_leg, _z_offset_RF_leg;

public:
  StandingMotion();
  void standingMotionCallback(const std_msgs::Bool& stand);
};
