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
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define X_OFFSET 0.0
#define TROT_STEP_EXTENT_X 30.0
#define TROT_STEP_EXTENT_Y 30.0
#define TROT_STEP_EXTENT_Z  8.0
#define LF_LEG_Z_OFFSET 120
#define LR_LEG_Z_OFFSET 120
#define RR_LEG_Z_OFFSET 120
#define RF_LEG_Z_OFFSET 120

class Trot_Gait{
  public:
  Trot_Gait();
  void controlLoop();
  
  private:
  int l;
  int number;
  double x, y, z;
  double x_offset, z_offset_LF_leg, z_offset_LR_leg, z_offset_RR_leg, z_offset_RF_leg;
  double dirupdate_x;
  double turn0;
  double vector_x, vector_y, vector_z;
  double trot_step_extent_x, trot_step_extent_y, trot_step_extent_z;
  double dir_x, dir_y;
  double w0, l0, h0;
  double w0_count_c, a0_count_c;
  double l_inv[4][2] = {{1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};
  double c_iter[4] = {0.0, 0.0, 0.0, 0.0};
  double c[4] = {0.0, 0.0, 0.0, 0.0};
  double c_inv[4] = {0.0, 0.0, 0.0, 0.0};
  bool stop_signal;
  
  ros::NodeHandle nh;
  ros::Publisher pub_leg_position;
  ros::Subscriber sub_trot_foward_motion;
  ros::Subscriber sub_trot_turn_motion;
  ros::Subscriber sub_stop_signal;
  ros::Subscriber sub_stabilization_variable;
  std_msgs::Float64MultiArray leg_position;

  void init();
  void trot_foward_motion_callback(const std_msgs::Float64& direction_x);
  void trot_turn_motion_callback(const std_msgs::Float64& turn);
  void stop_signal_callback(const std_msgs::Bool& stop);
  void stabilization_variable_callback(const std_msgs::Float64MultiArray& MV);
  void trot(double c0_x, double c0_y, bool inv);
  void count_c(double step_extent_x);
  double rDir_x();
  double rDir_y();
};
