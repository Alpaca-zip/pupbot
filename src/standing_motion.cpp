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

/* ++++++++++++++++++++++++++++++++++
       Standing_Motion class
++++++++++++++++++++++++++++++++++ */
Standing_Motion::Standing_Motion(){
  init();
}

void Standing_Motion::init(){
  pub_leg_position = nh.advertise<std_msgs::Float64MultiArray>("/leg_position", 10);
  pub_stop_signal = nh.advertise<std_msgs::Bool>("/stop_signal", 10);
  sub_standing_motion = nh.subscribe("/standing_motion", 10, &Standing_Motion::standing_motion_callback, this);
  stop.data = true;
  leg_position.data.resize(12);
  leg_position.data[0] = X_OFFSET;
  leg_position.data[3] = X_OFFSET;
  leg_position.data[6] = X_OFFSET;
  leg_position.data[9] = X_OFFSET;
  z_offset_LF_leg = LF_LEG_Z_OFFSET;
  z_offset_LR_leg = LR_LEG_Z_OFFSET;
  z_offset_RR_leg = RR_LEG_Z_OFFSET;
  z_offset_RF_leg = RF_LEG_Z_OFFSET;
}

void Standing_Motion::standing_motion_callback(const std_msgs::Bool& stand){
if(stand.data){
  leg_position.data[2] = 40;
  leg_position.data[5] = 40;
  leg_position.data[8] = 40;
  leg_position.data[11] = 40;
  for(int i=0;i<50;i++){
    leg_position.data[2] += (z_offset_LF_leg-40)/50;
    leg_position.data[5] += (z_offset_LR_leg-40)/50;
    leg_position.data[8] += (z_offset_RR_leg-40)/50;
    leg_position.data[11] += (z_offset_RF_leg-40)/50;
    pub_leg_position.publish(leg_position);
    ros::Duration(0.05).sleep();
    }
  stop.data = false;
  pub_stop_signal.publish(stop);
  }else{
    stop.data = true;
    pub_stop_signal.publish(stop);
    leg_position.data[2] = z_offset_LF_leg;
    leg_position.data[5] = z_offset_LR_leg;
    leg_position.data[8] = z_offset_RR_leg;
    leg_position.data[11] = z_offset_RF_leg;
    for(int i=0;i<50;i++){
      leg_position.data[2] -= (z_offset_LF_leg-40)/50;
      leg_position.data[5] -= (z_offset_LR_leg-40)/50;
      leg_position.data[8] -= (z_offset_RR_leg-40)/50;
      leg_position.data[11] -= (z_offset_RF_leg-40)/50;
      pub_leg_position.publish(leg_position);
      ros::Duration(0.05).sleep();
    }
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "standing_motion");
  Standing_Motion standing_motion;
  ros::spin();
  return 0;
}
