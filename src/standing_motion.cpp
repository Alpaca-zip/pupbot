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

#include "standing_motion.h"

standingMotion::standingMotion() : _pnh("~")
{
  _pnh.param<double>("x_offset", _x_offset, -10.0);
  _pnh.param<double>("z_offset_LF_leg", _z_offset_LF_leg, 120.0);
  _pnh.param<double>("z_offset_LR_leg", _z_offset_LR_leg, 120.0);
  _pnh.param<double>("z_offset_RR_leg", _z_offset_RR_leg, 120.0);
  _pnh.param<double>("z_offset_RF_leg", _z_offset_RF_leg, 120.0);

  _pub_leg_position = _nh.advertise<std_msgs::Float64MultiArray>("leg_position", 10);
  _pub_stop_signal = _nh.advertise<std_msgs::Bool>("stop_signal", 10);
  _sub_standing_motion = _nh.subscribe("standing_motion", 10, &standingMotion::standingMotionCallback, this);

  _stop.data = true;
  _leg_position.data.resize(12);
  _leg_position.data[0] = _x_offset;
  _leg_position.data[3] = _x_offset;
  _leg_position.data[6] = _x_offset;
  _leg_position.data[9] = _x_offset;
}

void standingMotion::standingMotionCallback(const std_msgs::Bool& stand)
{
  if (stand.data)
  {
    _leg_position.data[2] = 40.0;
    _leg_position.data[5] = 40.0;
    _leg_position.data[8] = 40.0;
    _leg_position.data[11] = 40.0;
    for (int i = 0; i < 50; i++)
    {
      _leg_position.data[2] += (_z_offset_LF_leg - 40.0) / 50.0;
      _leg_position.data[5] += (_z_offset_LR_leg - 40.0) / 50.0;
      _leg_position.data[8] += (_z_offset_RR_leg - 40.0) / 50.0;
      _leg_position.data[11] += (_z_offset_RF_leg - 40.0) / 50.0;
      _pub_leg_position.publish(_leg_position);
      ros::Duration(0.05).sleep();
    }
    _stop.data = false;
    _pub_stop_signal.publish(_stop);
  }
  else
  {
    _stop.data = true;
    _pub_stop_signal.publish(_stop);
    _leg_position.data[2] = _z_offset_LF_leg;
    _leg_position.data[5] = _z_offset_LR_leg;
    _leg_position.data[8] = _z_offset_RR_leg;
    _leg_position.data[11] = _z_offset_RF_leg;
    for (int i = 0; i < 50; i++)
    {
      _leg_position.data[2] -= (_z_offset_LF_leg - 40.0) / 50.0;
      _leg_position.data[5] -= (_z_offset_LR_leg - 40.0) / 50.0;
      _leg_position.data[8] -= (_z_offset_RR_leg - 40.0) / 50.0;
      _leg_position.data[11] -= (_z_offset_RF_leg - 40.0) / 50.0;
      _pub_leg_position.publish(_leg_position);
      ros::Duration(0.05).sleep();
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "standing_motion");
  standingMotion SM;
  ros::spin();
  return 0;
}
