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

#include "trot_gait.h"

/* ++++++++++++++++++++++++++++++++++
         Trot_Gait class
++++++++++++++++++++++++++++++++++ */
Trot_Gait::Trot_Gait(){
  init();
}

void Trot_Gait::init(){
  pub_leg_position = nh.advertise<std_msgs::Float64MultiArray>("/leg_position", 10);
  sub_trot_foward_motion = nh.subscribe("/trot_foward_motion", 10, &Trot_Gait::trot_foward_motion_callback, this);
  sub_trot_turn_motion = nh.subscribe("/trot_turn_motion", 10, &Trot_Gait::trot_turn_motion_callback, this);
  sub_stop_signal = nh.subscribe("/stop_signal", 10, &Trot_Gait::stop_signal_callback, this);
  sub_stabilization_variable = nh.subscribe("/stabilization_variable", 10, &Trot_Gait::stabilization_variable_callback, this);
  number = 22;
  x_offset = X_OFFSET;
  z_offset_LF_leg = LF_LEG_Z_OFFSET;
  z_offset_LR_leg = LR_LEG_Z_OFFSET;
  z_offset_RR_leg = RR_LEG_Z_OFFSET;
  z_offset_RF_leg = RF_LEG_Z_OFFSET;
  dirupdate_x = 0.0;
  turn0 = 0.0;
  trot_step_extent_x = TROT_STEP_EXTENT_X;
  trot_step_extent_y = TROT_STEP_EXTENT_Y;
  trot_step_extent_z = TROT_STEP_EXTENT_Z;
  stop_signal = true;
  leg_position.data.resize(12);
}

void Trot_Gait::trot_foward_motion_callback(const std_msgs::Float64& direction_x){
  dirupdate_x = direction_x.data;
}

void Trot_Gait::trot_turn_motion_callback(const std_msgs::Float64& turn){
  turn0 = turn.data;
}

void Trot_Gait::stop_signal_callback(const std_msgs::Bool& stop){
  stop_signal = stop.data;
}

void Trot_Gait::stabilization_variable_callback(const std_msgs::Float64MultiArray& MV){
  z_offset_LF_leg = MV.data[0];
  z_offset_LR_leg = MV.data[1];
  z_offset_RR_leg = MV.data[2];
  z_offset_RF_leg = MV.data[3];
}

void Trot_Gait::trot(double c0_x, double c0_y, bool inv){
  w0 = trot_step_extent_x/2.0*dir_x;
  l0 = trot_step_extent_y/2.0*dir_y;
  h0 = trot_step_extent_z;
  if(inv == false){
    c0_x = -c0_x;
    c0_y = -c0_y;
  }
  if(w0 == 0.0 && l0 == 0.0){
    vector_x = 0.0;
    vector_y = 0.0;
    vector_z = 0.0;
  }else if(w0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_y/l0)*(c0_y/l0))*h0*h0));
    vector_x = c0_x;
    vector_y = c0_y;
    vector_z = h1*int(inv);
  }else if(l0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    vector_x = c0_x;
    vector_y = c0_y;
    vector_z = h1*int(inv);
  }else{
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    vector_x = c0_x;
    vector_y = c0_y;
    vector_z = h1*int(inv);
  }
}

void Trot_Gait::count_c(double step_extent_x){
  w0_count_c = step_extent_x*std::max(abs(dir_x),abs(dir_y))/2.0;
  a0_count_c = (2.0*w0_count_c)*(c_iter[l]/number)-w0_count_c;
  c[l] = a0_count_c;
  c_iter[l] += 1.0;
  if(c_iter[l] > number){
    c[l] = -w0_count_c;
    c_iter[l] = 1.0;
    c_inv[l] += 1.0;
    if (c_inv[l] > 31)c_inv[l] = 0.0;
  }
}

double Trot_Gait::rDir_x(){
  if(dir_x != 0.0 || dir_y != 0.0){
    return dir_x/std::max(abs(dir_x), abs(dir_y));
  }else{
    return 1.0;
  } 
}

double Trot_Gait::rDir_y(){
  if(dir_x != 0.0 || dir_y != 0.0){
    return dir_y/std::max(abs(dir_x), abs(dir_y));
  }else{
    return 1.0;
  } 
}

void Trot_Gait::controlLoop(){
  if(!stop_signal){
    for(l=0;l<4;l++){
      dir_x = dirupdate_x+turn0*l_inv[l][1];
      dir_y = turn0*l_inv[l][0];
      count_c(trot_step_extent_x);
      trot(rDir_x()*c[l], l_inv[l][1]*rDir_y()*c[l], bool(l%2)^bool(fmod(c_inv[l], 2.0)));
      x = x_offset+vector_x;
      y = vector_y;
      if(l == 0){
        z = z_offset_LF_leg-vector_z;
      }else if(l == 1){
        z = z_offset_LR_leg-vector_z;
      }else if(l == 2){
        z = z_offset_RR_leg-vector_z;
      }else{
        z = z_offset_RF_leg-vector_z;
      }
      leg_position.data[3*l] = x;
      leg_position.data[3*l+1] = y;
      leg_position.data[3*l+2] = z;
    }
    pub_leg_position.publish(leg_position);
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "trot_gait");
  Trot_Gait trot_gait;
  ros::Rate loop_rate(80);
  while(ros::ok()){
    trot_gait.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
