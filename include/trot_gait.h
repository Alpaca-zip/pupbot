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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

class trotGait{
  private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Publisher _pub_leg_position;
    ros::Subscriber _sub_trot_foward_motion;
    ros::Subscriber _sub_trot_turn_motion;
    ros::Subscriber _sub_stop_signal;
    ros::Subscriber _sub_stabilization_variable;
    std_msgs::Float64MultiArray _leg_position;
    bool _stop_signal;
    int _iter_number;
    double _x_offset, _z_offset_LF_leg, _z_offset_LR_leg, _z_offset_RR_leg, _z_offset_RF_leg;
    double _dirupdate_x;
    double _turn0;
    double _vector_x, _vector_y, _vector_z;
    double _trot_step_extent_x, _trot_step_extent_y, _trot_step_extent_z;
    double _dir_x, _dir_y;
    double _c[4], _c_inv[4], _c_iter[4], _l_inv[4][2];

  public:
    trotGait();
    void controlLoop();
    void trotFowardMotionCallback(const std_msgs::Float64& direction_x);
    void trotTurnMotionCallback(const std_msgs::Float64& turn);
    void stopSignalCallback(const std_msgs::Bool& stop);
    void stabilizationVariableCallback(const std_msgs::Float64MultiArray& MV);
    void trot(double c0_x, double c0_y, bool inv);
    void countC(const double step_extent_x, const int l);
    double rDirX();
    double rDirY();
};
