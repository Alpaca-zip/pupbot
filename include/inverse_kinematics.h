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
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#define BONE_LENGTH 83.0
#define LENGTH 21.0

class inverseKinematics{
  private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Publisher _pub_LF_leg;
    ros::Publisher _pub_LR_leg;
    ros::Publisher _pub_RF_leg;
    ros::Publisher _pub_RR_leg;
    ros::Subscriber _sub_leg_position;
    trajectory_msgs::JointTrajectory _LF_leg, _LR_leg, _RF_leg, _RR_leg;
    double _duration;

  public:
    inverseKinematics();
    void inverseKinematicsCallback(const std_msgs::Float64MultiArray& leg_position);
};
