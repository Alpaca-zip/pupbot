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
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define X_OFFSET -10.0
#define LF_LEG_Z_OFFSET 120
#define LR_LEG_Z_OFFSET 120
#define RR_LEG_Z_OFFSET 120
#define RF_LEG_Z_OFFSET 120

class Standing_Motion{
  public:
  Standing_Motion();
  
  private:
  double z_offset_LF_leg, z_offset_LR_leg, z_offset_RR_leg, z_offset_RF_leg;
  bool stop_signal;
  
  ros::NodeHandle nh;
  ros::Publisher pub_leg_position;
  ros::Publisher pub_stop_signal;
  ros::Subscriber sub_standing_motion;
  std_msgs::Float64MultiArray leg_position;
  std_msgs::Bool stop;

  void init();
  void standing_motion_callback(const std_msgs::Bool& stand);
};
