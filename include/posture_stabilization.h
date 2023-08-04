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
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define X_OFFSET 61
#define Y_OFFSET 94
#define LF_LEG_Z_OFFSET 120
#define LR_LEG_Z_OFFSET 120
#define RR_LEG_Z_OFFSET 120
#define RF_LEG_Z_OFFSET 120
#define WIDTH 27

class Posture_Stabilization{
  public:
  Posture_Stabilization();
  void controlLoop();

  private:
  double roll_data;
  double pitch_data;
  double M_LF_leg, M_LR_leg, M_RR_leg, M_RF_leg;
  double M1_LF_leg, M1_LR_leg, M1_RR_leg, M1_RF_leg;
  double e_LF_leg, e_LR_leg, e_RR_leg, e_RF_leg;
  double e1_LF_leg, e1_LR_leg, e1_RR_leg, e1_RF_leg;
  double e2_LF_leg, e2_LR_leg, e2_RR_leg, e2_RF_leg;
  double P;
  double I;
  double D;
  double buff_roll[WIDTH], buff_pitch[WIDTH];
  double roll_sum, pitch_sum;
  int roll_cnt, pitch_cnt;

  ros::NodeHandle nh;
  ros::Publisher pub_stabilization_variable;
  ros::Publisher pub_roll_LPF;
  ros::Publisher pub_pitch_LPF;
  ros::Subscriber roll_sub;
  ros::Subscriber pitch_sub;
  ros::Subscriber key_control_sub_posture_control;

  std_msgs::Float64 roll_LPF, pitch_LPF;
  std_msgs::Float64MultiArray MV;
  bool posture_control_on;

  void init();
  void roll_callback(const std_msgs::Float64& roll);
  void pitch_callback(const std_msgs::Float64& pitch);
  void posture_control_callback(const std_msgs::Bool& posture_control);
};
