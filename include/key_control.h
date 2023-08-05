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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <termios.h>

class keyControl{
  private:
    ros::NodeHandle _nh;
    ros::Publisher _trot_foward_motion_pub;
    ros::Publisher _trot_turn_motion_pub;
    ros::Publisher _standing_motion_pub;
    ros::Publisher _posture_control_pub;
    std_msgs::Bool _posture_control;
    std_msgs::Float64 _direction_x;
    std_msgs::Float64 _turn;
    std_msgs::Bool _stand;

  public:
    keyControl();
    void controlLoop();
    int getch();
};
