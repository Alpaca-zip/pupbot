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

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"

#define BONE_LENGTH 83.0
#define LENGTH 21.0

class Inverse_Kinematics{
  public:
  Inverse_Kinematics();
  
  private:
  int l;
  double x, y, z, a0, a1, b0;
  double angle1, angle2, angle3;
  double bone_length;
  double target_leg_shoulder_joint, target_L_leg_upper_joint, target_L_leg_lower_joint, target_R_leg_upper_joint, target_R_leg_lower_joint;
  
  ros::NodeHandle nh;
  ros::Publisher pub_LF_leg;
  ros::Publisher pub_LR_leg;
  ros::Publisher pub_RF_leg;
  ros::Publisher pub_RR_leg;
  ros::Subscriber sub_leg_position;
  trajectory_msgs::JointTrajectory LF_leg, LR_leg, RF_leg, RR_leg;

  void init();
  void inverse_kinematics_callback(const std_msgs::Float64MultiArray& leg_position);
};
