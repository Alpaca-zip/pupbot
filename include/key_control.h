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
#include "termios.h"

class Key_Control{
  public:
  Key_Control();
  void controlLoop();

  private:
  ros::NodeHandle nh;
  ros::Publisher trot_foward_motion_pub;
  ros::Publisher trot_turn_motion_pub;
  ros::Publisher standing_motion_pub;
  ros::Publisher posture_control_pub;
  
  std_msgs::Bool posture_control;
  std_msgs::Float64 direction_x;
  std_msgs::Float64 turn;
  std_msgs::Bool stand;

  void init();
  int getch();
};
