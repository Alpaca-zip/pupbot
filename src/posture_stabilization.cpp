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

#include "posture_stabilization.h"

/* ++++++++++++++++++++++++++++++++++
     Posture_Stabilization class
++++++++++++++++++++++++++++++++++ */
Posture_Stabilization::Posture_Stabilization(){
  init();
}

void Posture_Stabilization::init(){
  MV.data.resize(4);
  MV.data[0] = LF_LEG_Z_OFFSET;
  MV.data[1] = LR_LEG_Z_OFFSET;
  MV.data[2] = RR_LEG_Z_OFFSET;
  MV.data[3] = RF_LEG_Z_OFFSET;
  roll_LPF.data = 0.0;
  pitch_LPF.data = 0.0;
  pub_stabilization_variable = nh.advertise<std_msgs::Float64MultiArray>("/stabilization_variable", 10);
  pub_roll_LPF = nh.advertise<std_msgs::Float64>("/roll_LPF", 10);
  pub_pitch_LPF = nh.advertise<std_msgs::Float64>("/pitch_LPF", 10);
  roll_sub = nh.subscribe("/roll", 10, &Posture_Stabilization::roll_callback, this);
  pitch_sub = nh.subscribe("/pitch", 10, &Posture_Stabilization::pitch_callback, this);
  key_control_sub_posture_control = nh.subscribe("/posture_control", 10, &Posture_Stabilization::posture_control_callback, this);

  M_LF_leg = M_LR_leg = M_RR_leg = M_RF_leg = 0.0;
  M1_LF_leg = M1_LR_leg = M1_RR_leg = M1_RF_leg = 0.0;
  e_LF_leg = e_LR_leg = e_RR_leg = e_RF_leg = 0.0;
  e1_LF_leg = e1_LR_leg = e1_RR_leg = e1_RF_leg = 0.0;
  e2_LF_leg = e2_LR_leg = e2_RR_leg = e2_RF_leg = 0.0;

  P = 0.5;
  I = 0.1;
  D = 0.15;
  roll_sum = 0.0;
  pitch_sum = 0.0;
  roll_cnt = 0;
  pitch_cnt = 0;
  posture_control_on = false;
}

void Posture_Stabilization::roll_callback(const std_msgs::Float64& roll){
  roll_data = roll.data;
  if (roll_cnt == WIDTH) roll_cnt = 0;
  roll_sum -= buff_roll[roll_cnt];
  buff_roll[roll_cnt] = roll_data;
  roll_sum += buff_roll[roll_cnt];
  roll_cnt++;
  roll_LPF.data = roll_sum/WIDTH;
  pub_roll_LPF.publish(roll_LPF);
}

void Posture_Stabilization::pitch_callback(const std_msgs::Float64& pitch){
  pitch_data = pitch.data;
  if (pitch_cnt == WIDTH) pitch_cnt = 0;
  pitch_sum -= buff_pitch[pitch_cnt];
  buff_pitch[pitch_cnt] = pitch_data;
  pitch_sum += buff_pitch[pitch_cnt];
  pitch_cnt++;
  pitch_LPF.data = pitch_sum/WIDTH;
  pub_pitch_LPF.publish(pitch_LPF);
}

void Posture_Stabilization::posture_control_callback(const std_msgs::Bool& posture_control){
  if(posture_control_on){
    posture_control_on = false;
  }else{
    posture_control_on = true;
  }
}

void Posture_Stabilization::controlLoop(){
  M1_LF_leg = M_LF_leg;
  M1_LR_leg = M_LR_leg;
  M1_RR_leg = M_RR_leg;
  M1_RF_leg = M_RF_leg;
  e2_LF_leg = e1_LF_leg;
  e2_LR_leg = e1_LR_leg;
  e2_RR_leg = e1_RR_leg;
  e2_RF_leg = e1_RF_leg;
  e1_LF_leg = e_LF_leg;
  e1_LR_leg = e_LR_leg;
  e1_RR_leg = e_RR_leg;
  e1_RF_leg = e_RF_leg;

  e_LF_leg = -Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)+X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_LR_leg = -Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)-X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_RR_leg = Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)-X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_RF_leg = Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)+X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);

  if(posture_control_on){
    M_LF_leg = M1_LF_leg+P*(e_LF_leg-e1_LF_leg)+I*e_LF_leg+D*((e_LF_leg-e1_LF_leg)-(e1_LF_leg-e2_LF_leg));
    M_LR_leg = M1_LR_leg+P*(e_LR_leg-e1_LR_leg)+I*e_LR_leg+D*((e_LR_leg-e1_LR_leg)-(e1_LR_leg-e2_LR_leg));
    M_RR_leg = M1_RR_leg+P*(e_RR_leg-e1_RR_leg)+I*e_RR_leg+D*((e_RR_leg-e1_RR_leg)-(e1_RR_leg-e2_RR_leg));
    M_RF_leg = M1_RF_leg+P*(e_RF_leg-e1_RF_leg)+I*e_RF_leg+D*((e_RF_leg-e1_RF_leg)-(e1_RF_leg-e2_RF_leg));
  }else{
    M_LF_leg = M1_LF_leg+0*(e_LF_leg-e1_LF_leg)+0*e_LF_leg+0*((e_LF_leg-e1_LF_leg)-(e1_LF_leg-e2_LF_leg));
    M_LR_leg = M1_LR_leg+0*(e_LR_leg-e1_LR_leg)+0*e_LR_leg+0*((e_LR_leg-e1_LR_leg)-(e1_LR_leg-e2_LR_leg));
    M_RR_leg = M1_RR_leg+0*(e_RR_leg-e1_RR_leg)+0*e_RR_leg+0*((e_RR_leg-e1_RR_leg)-(e1_RR_leg-e2_RR_leg));
    M_RF_leg = M1_RF_leg+0*(e_RF_leg-e1_RF_leg)+0*e_RF_leg+0*((e_RF_leg-e1_RF_leg)-(e1_RF_leg-e2_RF_leg));
  }

  MV.data[0] = LF_LEG_Z_OFFSET+M_LF_leg;
  MV.data[1] = LR_LEG_Z_OFFSET+M_LR_leg;
  MV.data[2] = RR_LEG_Z_OFFSET+M_RR_leg;
  MV.data[3] = RF_LEG_Z_OFFSET+M_RF_leg;

  for(int i=0;i<4;i++){
    if(MV.data[i] > 160.0){
      MV.data[i]  = 160.0;
    }else if(MV.data[i] < 15.0){
      MV.data[i]  = 15.0;
    }
  }

  pub_stabilization_variable.publish(MV);
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "posture_stabilization");
  Posture_Stabilization posture_stabilization;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    posture_stabilization.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
