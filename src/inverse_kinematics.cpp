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

#include "inverse_kinematics.h"

inverseKinematics::inverseKinematics() : _pnh("~")
{
  _pnh.param<double>("duration", _duration, 0.01);

  _pub_LF_leg =
    _nh.advertise<trajectory_msgs::JointTrajectory>("leftfront_leg_controller/command", 10);
  _pub_LR_leg =
    _nh.advertise<trajectory_msgs::JointTrajectory>("leftback_leg_controller/command", 10);
  _pub_RF_leg =
    _nh.advertise<trajectory_msgs::JointTrajectory>("rightfront_leg_controller/command", 10);
  _pub_RR_leg =
    _nh.advertise<trajectory_msgs::JointTrajectory>("rightback_leg_controller/command", 10);
  _sub_leg_position =
    _nh.subscribe("leg_position", 10, &inverseKinematics::inverseKinematicsCallback, this);

  _LF_leg.joint_names.resize(3);
  _LF_leg.points.resize(1);
  _LF_leg.points[0].positions.resize(3);
  _LF_leg.joint_names[0] = "leftfront_leg_shoulder_joint";
  _LF_leg.joint_names[1] = "leftfront_leg_upper_joint";
  _LF_leg.joint_names[2] = "leftfront_leg_lower_joint";
  _LR_leg.joint_names.resize(3);
  _LR_leg.points.resize(1);
  _LR_leg.points[0].positions.resize(3);
  _LR_leg.joint_names[0] = "leftback_leg_shoulder_joint";
  _LR_leg.joint_names[1] = "leftback_leg_upper_joint";
  _LR_leg.joint_names[2] = "leftback_leg_lower_joint";
  _RR_leg.joint_names.resize(3);
  _RR_leg.points.resize(1);
  _RR_leg.points[0].positions.resize(3);
  _RR_leg.joint_names[0] = "rightback_leg_shoulder_joint";
  _RR_leg.joint_names[1] = "rightback_leg_upper_joint";
  _RR_leg.joint_names[2] = "rightback_leg_lower_joint";
  _RF_leg.joint_names.resize(3);
  _RF_leg.points.resize(1);
  _RF_leg.points[0].positions.resize(3);
  _RF_leg.joint_names[0] = "rightfront_leg_shoulder_joint";
  _RF_leg.joint_names[1] = "rightfront_leg_upper_joint";
  _RF_leg.joint_names[2] = "rightfront_leg_lower_joint";
}

void inverseKinematics::inverseKinematicsCallback(const std_msgs::Float64MultiArray & leg_position)
{
  double x, y, z, a0, a1, b0;
  double angle1, angle2, angle3;
  double target_leg_shoulder_joint, target_L_leg_upper_joint, target_L_leg_lower_joint,
    target_R_leg_upper_joint, target_R_leg_lower_joint;

  for (int l = 0; l < 4; l++) {
    x = leg_position.data[3 * l];
    y = leg_position.data[3 * l + 1];
    z = leg_position.data[3 * l + 2];

    a0 = (y * y + z * z - LENGTH * LENGTH + x * x - 2 * BONE_LENGTH * BONE_LENGTH) /
         (2 * BONE_LENGTH * BONE_LENGTH);

    if (l == 0) {
      angle1 = -atan2(-z, -y) - atan2(sqrt(y * y + z * z - LENGTH * LENGTH), LENGTH);
      angle3 = atan2(sqrt(1 - a0 * a0), a0);
      angle2 = atan2(x, sqrt(y * y + z * z - LENGTH * LENGTH)) -
               atan2(BONE_LENGTH * sin(angle3), BONE_LENGTH * (1 + cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      _LF_leg.points[0].positions[0] = target_leg_shoulder_joint;
      _LF_leg.points[0].positions[1] = target_L_leg_upper_joint;
      _LF_leg.points[0].positions[2] = target_L_leg_lower_joint;
      _LF_leg.header.stamp = ros::Time::now();
      _LF_leg.points[0].time_from_start = ros::Duration(_duration);
      _pub_LF_leg.publish(_LF_leg);
    } else if (l == 1) {
      angle1 = -atan2(-z, -y) - atan2(sqrt(y * y + z * z - LENGTH * LENGTH), LENGTH);
      angle3 = atan2(sqrt(1 - a0 * a0), a0);
      angle2 = atan2(x, sqrt(y * y + z * z - LENGTH * LENGTH)) -
               atan2(BONE_LENGTH * sin(angle3), BONE_LENGTH * (1 + cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      _LR_leg.points[0].positions[0] = target_leg_shoulder_joint;
      _LR_leg.points[0].positions[1] = target_L_leg_upper_joint;
      _LR_leg.points[0].positions[2] = target_L_leg_lower_joint;
      _LR_leg.header.stamp = ros::Time::now();
      _LR_leg.points[0].time_from_start = ros::Duration(_duration);
      _pub_LR_leg.publish(_LR_leg);
    } else if (l == 2) {
      angle1 = -atan2(-z, -y) - atan2(sqrt(y * y + z * z - LENGTH * LENGTH), -LENGTH);
      angle3 = atan2(sqrt(1 - a0 * a0), a0);
      angle2 = atan2(x, sqrt(y * y + z * z - LENGTH * LENGTH)) -
               atan2(BONE_LENGTH * sin(angle3), BONE_LENGTH * (1 + cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      _RR_leg.points[0].positions[0] = target_leg_shoulder_joint;
      _RR_leg.points[0].positions[1] = target_R_leg_upper_joint;
      _RR_leg.points[0].positions[2] = target_R_leg_lower_joint;
      _RR_leg.header.stamp = ros::Time::now();
      _RR_leg.points[0].time_from_start = ros::Duration(_duration);
      _pub_RR_leg.publish(_RR_leg);
    } else if (l == 3) {
      angle1 = -atan2(-z, -y) - atan2(sqrt(y * y + z * z - LENGTH * LENGTH), -LENGTH);
      angle3 = atan2(sqrt(1 - a0 * a0), a0);
      angle2 = atan2(x, sqrt(y * y + z * z - LENGTH * LENGTH)) -
               atan2(BONE_LENGTH * sin(angle3), BONE_LENGTH * (1 + cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      _RF_leg.points[0].positions[0] = target_leg_shoulder_joint;
      _RF_leg.points[0].positions[1] = target_R_leg_upper_joint;
      _RF_leg.points[0].positions[2] = target_R_leg_lower_joint;
      _RF_leg.header.stamp = ros::Time::now();
      _RF_leg.points[0].time_from_start = ros::Duration(_duration);
      _pub_RF_leg.publish(_RF_leg);
    }
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "inverse_kinematics");
  inverseKinematics IK;
  ros::spin();
  return 0;
}
