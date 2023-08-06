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

trotGait::trotGait() : _pnh("~"){
  _pnh.param<int>("iter_number", _iter_number, 22);
  _pnh.param<double>("trot_step_extent_x", _trot_step_extent_x, 30.0);
  _pnh.param<double>("trot_step_extent_y", _trot_step_extent_y, 30.0);
  _pnh.param<double>("trot_step_extent_z", _trot_step_extent_z, 8.0);
  _pnh.param<double>("x_offset", _x_offset, 0.0);
  _pnh.param<double>("z_offset_LF_leg", _z_offset_LF_leg, 120.0);
  _pnh.param<double>("z_offset_LR_leg", _z_offset_LR_leg, 120.0);
  _pnh.param<double>("z_offset_RR_leg", _z_offset_RR_leg, 120.0);
  _pnh.param<double>("z_offset_RF_leg", _z_offset_RF_leg, 120.0);

  _pub_leg_position = _nh.advertise<std_msgs::Float64MultiArray>("leg_position", 10);
  _sub_trot_foward_motion = _nh.subscribe("trot_foward_motion", 10, &trotGait::trotFowardMotionCallback, this);
  _sub_trot_turn_motion = _nh.subscribe("trot_turn_motion", 10, &trotGait::trotTurnMotionCallback, this);
  _sub_stop_signal = _nh.subscribe("stop_signal", 10, &trotGait::stopSignalCallback, this);
  _sub_stabilization_variable = _nh.subscribe("stabilization_variable", 10, &trotGait::stabilizationVariableCallback, this);
  
  _leg_position.data.resize(12);
  _stop_signal = true;
  _dirupdate_x = 0.0;
  _turn0 = 0.0;
  _vector_x = _vector_y = _vector_z = 0.0;
  _dir_x = _dir_y = 0.0;
  for(int i = 0; i < 4; i++) {
    _c[i] = 0.0;
    _c_inv[i] = 0.0;
    _c_iter[i] = 0.0;
  }
  _l_inv[0][0] = 1.0; _l_inv[0][1] = 1.0;
  _l_inv[1][0] = -1.0; _l_inv[1][1] = 1.0;
  _l_inv[2][0] = 1.0; _l_inv[2][1] = -1.0;
  _l_inv[3][0] = -1.0; _l_inv[3][1] = -1.0;
}

void trotGait::trotFowardMotionCallback(const std_msgs::Float64& direction_x){
  _dirupdate_x = direction_x.data;
}

void trotGait::trotTurnMotionCallback(const std_msgs::Float64& turn){
  _turn0 = turn.data;
}

void trotGait::stopSignalCallback(const std_msgs::Bool& stop){
  _stop_signal = stop.data;
}

void trotGait::stabilizationVariableCallback(const std_msgs::Float64MultiArray& MV){
  _z_offset_LF_leg = MV.data[0];
  _z_offset_LR_leg = MV.data[1];
  _z_offset_RR_leg = MV.data[2];
  _z_offset_RF_leg = MV.data[3];
}

void trotGait::trot(double c0_x, double c0_y, bool inv){
  double w0, l0, h0;
  w0 = _trot_step_extent_x/2.0*_dir_x;
  l0 = _trot_step_extent_y/2.0*_dir_y;
  h0 = _trot_step_extent_z;
  if(inv == false){
    c0_x = -c0_x;
    c0_y = -c0_y;
  }
  if(w0 == 0.0 && l0 == 0.0){
    _vector_x = 0.0;
    _vector_y = 0.0;
    _vector_z = 0.0;
  }else if(w0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_y/l0)*(c0_y/l0))*h0*h0));
    _vector_x = c0_x;
    _vector_y = c0_y;
    _vector_z = h1*int(inv);
  }else if(l0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    _vector_x = c0_x;
    _vector_y = c0_y;
    _vector_z = h1*int(inv);
  }else{
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    _vector_x = c0_x;
    _vector_y = c0_y;
    _vector_z = h1*int(inv);
  }
}

void trotGait::countC(const double step_extent_x, const int l){
  double w0_count_c, a0_count_c;
  w0_count_c = step_extent_x*std::max(abs(_dir_x),abs(_dir_y))/2.0;
  a0_count_c = (2.0*w0_count_c)*(_c_iter[l]/_iter_number)-w0_count_c;
  _c[l] = a0_count_c;
  _c_iter[l] += 1.0;
  if(_c_iter[l] > _iter_number){
    _c[l] = -w0_count_c;
    _c_iter[l] = 1.0;
    _c_inv[l] += 1.0;
    if (_c_inv[l] > 31)_c_inv[l] = 0.0;
  }
}

double trotGait::rDirX(){
  if(_dir_x != 0.0 || _dir_y != 0.0){
    return _dir_x/std::max(abs(_dir_x), abs(_dir_y));
  }else{
    return 1.0;
  } 
}

double trotGait::rDirY(){
  if(_dir_x != 0.0 || _dir_y != 0.0){
    return _dir_y/std::max(abs(_dir_x), abs(_dir_y));
  }else{
    return 1.0;
  } 
}

void trotGait::controlLoop(){
  double x, y, z;
  if(!_stop_signal){
    for(int l=0;l<4;l++){
      _dir_x = _dirupdate_x+_turn0*_l_inv[l][1];
      _dir_y = _turn0*_l_inv[l][0];
      countC(_trot_step_extent_x, l);
      trot(rDirX()*_c[l], _l_inv[l][1]*rDirY()*_c[l], bool(l%2)^bool(fmod(_c_inv[l], 2.0)));
      x = _x_offset+_vector_x;
      y = _vector_y;
      if(l == 0){
        z = _z_offset_LF_leg-_vector_z;
      }else if(l == 1){
        z = _z_offset_LR_leg-_vector_z;
      }else if(l == 2){
        z = _z_offset_RR_leg-_vector_z;
      }else{
        z = _z_offset_RF_leg-_vector_z;
      }
      _leg_position.data[3*l] = x;
      _leg_position.data[3*l+1] = y;
      _leg_position.data[3*l+2] = z;
    }
    _pub_leg_position.publish(_leg_position);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "trot_gait");
  trotGait TG;
  ros::Rate loop_rate(50);
  while(ros::ok()){
    TG.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
