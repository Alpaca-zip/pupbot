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

#include "posture_stabilization.h"

postureStabilization::postureStabilization() : _pnh("~"){
  _pnh.param<std::string>("imu_topic", _imu_topic, "imu");
  _pnh.param<int>("width", _width, 27);
  _pnh.param<double>("z_offset_LF_leg", _z_offset_LF_leg, 120);
  _pnh.param<double>("z_offset_LR_leg", _z_offset_LR_leg, 120);
  _pnh.param<double>("z_offset_RR_leg", _z_offset_RR_leg, 120);
  _pnh.param<double>("z_offset_RF_leg", _z_offset_RF_leg, 120);
  _pnh.param<double>("p_gain", _P, 0.5);
  _pnh.param<double>("i_gain", _I, 0.1);
  _pnh.param<double>("D_gain", _D, 0.15);

  _pub_stabilization_variable = _nh.advertise<std_msgs::Float64MultiArray>("stabilization_variable", 10);
  _pub_roll_LPF = _nh.advertise<std_msgs::Float64>("roll_LPF", 10);
  _pub_pitch_LPF = _nh.advertise<std_msgs::Float64>("pitch_LPF", 10);
  _imu_sub = _nh.subscribe(_imu_topic, 10, &postureStabilization::imuCallback, this);
  _key_control_sub_posture_control = _nh.subscribe("posture_control", 10, &postureStabilization::postureControlCallback, this);

  _buff_roll.resize(_width, 0.0);
  _buff_pitch.resize(_width, 0.0);
  _MV.data.resize(4);
  _MV.data[0] = _z_offset_LF_leg;
  _MV.data[1] = _z_offset_LR_leg;
  _MV.data[2] = _z_offset_RR_leg;
  _MV.data[3] = _z_offset_RF_leg;
  _roll_LPF.data = 0.0;
  _pitch_LPF.data = 0.0;
  _M_LF_leg = _M_LR_leg = _M_RR_leg = _M_RF_leg = 0.0;
  _M1_LF_leg = _M1_LR_leg = _M1_RR_leg = _M1_RF_leg = 0.0;
  _e_LF_leg = _e_LR_leg = _e_RR_leg = _e_RF_leg = 0.0;
  _e1_LF_leg = _e1_LR_leg = _e1_RR_leg = _e1_RF_leg = 0.0;
  _e2_LF_leg = _e2_LR_leg = _e2_RR_leg = _e2_RF_leg = 0.0;
  _roll_sum = 0.0;
  _pitch_sum = 0.0;
  _imu_cnt = 0;
  _posture_control_on = false;
}

void postureStabilization::imuCallback(const sensor_msgs::Imu& msg){
  double roll, pitch, _;
  quatToRPY(msg.orientation, roll, pitch, _);

  if (_imu_cnt == _width){
    _imu_cnt = 0;
  }

  _roll_sum -= _buff_roll[_imu_cnt];
  _pitch_sum -= _buff_pitch[_imu_cnt];
  _buff_roll[_imu_cnt] = roll;
  _buff_pitch[_imu_cnt] = pitch;
  _roll_sum += _buff_roll[_imu_cnt];
  _pitch_sum += _buff_pitch[_imu_cnt];
  _imu_cnt++;

  _roll_LPF.data = _roll_sum/_width;
  _pitch_LPF.data = _pitch_sum/_width;
  _pub_roll_LPF.publish(_roll_LPF);
  _pub_pitch_LPF.publish(_pitch_LPF);
}

void postureStabilization::quatToRPY(const geometry_msgs::Quaternion quat, double& roll, double& pitch, double& yaw){
	float q0q0 = quat.w * quat.w;
	float q1q1 = quat.x * quat.x;
	float q2q2 = quat.y * quat.y;
	float q3q3 = quat.z * quat.z;
	float q0q1 = quat.w * quat.x;
	float q0q2 = quat.w * quat.y;
	float q0q3 = quat.w * quat.z;
	float q1q2 = quat.x * quat.y;
	float q1q3 = quat.x * quat.z;
	float q2q3 = quat.y * quat.z;

  roll = atan2f((2.f * (q2q3 + q0q1)), (q0q0 - q1q1 - q2q2 + q3q3));
  pitch = -asinf((2.f * (q1q3 - q0q2)));
  yaw = atan2f((2.f * (q1q2 + q0q3)), (q0q0 + q1q1 - q2q2 - q3q3));
}

void postureStabilization::postureControlCallback(const std_msgs::Bool& posture_control){
  if(_posture_control_on){
    _posture_control_on = false;
  }else{
    _posture_control_on = true;
  }
}

void postureStabilization::controlLoop(){
  _M1_LF_leg = _M_LF_leg;
  _M1_LR_leg = _M_LR_leg;
  _M1_RR_leg = _M_RR_leg;
  _M1_RF_leg = _M_RF_leg;
  _e2_LF_leg = _e1_LF_leg;
  _e2_LR_leg = _e1_LR_leg;
  _e2_RR_leg = _e1_RR_leg;
  _e2_RF_leg = _e1_RF_leg;
  _e1_LF_leg = _e_LF_leg;
  _e1_LR_leg = _e_LR_leg;
  _e1_RR_leg = _e_RR_leg;
  _e1_RF_leg = _e_RF_leg;

  _e_LF_leg = -Y_OFFSET * tan(_roll_LPF.data / 180.0 * M_PI) + X_OFFSET * tan(_pitch_LPF.data / 180.0 * M_PI);
  _e_LR_leg = -Y_OFFSET * tan(_roll_LPF.data / 180.0 * M_PI) - X_OFFSET * tan(_pitch_LPF.data / 180.0 * M_PI);
  _e_RR_leg = Y_OFFSET * tan(_roll_LPF.data / 180.0 * M_PI) - X_OFFSET * tan(_pitch_LPF.data / 180.0 * M_PI);
  _e_RF_leg = Y_OFFSET * tan(_roll_LPF.data / 180.0 * M_PI) + X_OFFSET * tan(_pitch_LPF.data / 180.0 * M_PI);

  if(_posture_control_on){
    _M_LF_leg = _M1_LF_leg + _P * (_e_LF_leg - _e1_LF_leg) + _I * _e_LF_leg + _D * ((_e_LF_leg - _e1_LF_leg) - (_e1_LF_leg - _e2_LF_leg));
    _M_LR_leg = _M1_LR_leg + _P * (_e_LR_leg - _e1_LR_leg) + _I * _e_LR_leg + _D * ((_e_LR_leg - _e1_LR_leg) - (_e1_LR_leg - _e2_LR_leg));
    _M_RR_leg = _M1_RR_leg + _P * (_e_RR_leg - _e1_RR_leg) + _I * _e_RR_leg + _D * ((_e_RR_leg - _e1_RR_leg) - (_e1_RR_leg - _e2_RR_leg));
    _M_RF_leg = _M1_RF_leg + _P * (_e_RF_leg - _e1_RF_leg) + _I * _e_RF_leg + _D * ((_e_RF_leg - _e1_RF_leg) - (_e1_RF_leg - _e2_RF_leg));
  } else{
    _M_LF_leg = _M1_LF_leg + 0 * (_e_LF_leg - _e1_LF_leg) + 0 * _e_LF_leg + 0 * ((_e_LF_leg - _e1_LF_leg) - (_e1_LF_leg - _e2_LF_leg));
    _M_LR_leg = _M1_LR_leg + 0 * (_e_LR_leg - _e1_LR_leg) + 0 * _e_LR_leg + 0 * ((_e_LR_leg - _e1_LR_leg) - (_e1_LR_leg - _e2_LR_leg));
    _M_RR_leg = _M1_RR_leg + 0 * (_e_RR_leg - _e1_RR_leg) + 0 * _e_RR_leg + 0 * ((_e_RR_leg - _e1_RR_leg) - (_e1_RR_leg - _e2_RR_leg));
    _M_RF_leg = _M1_RF_leg + 0 * (_e_RF_leg - _e1_RF_leg) + 0 * _e_RF_leg + 0 * ((_e_RF_leg - _e1_RF_leg) - (_e1_RF_leg - _e2_RF_leg));
  }

  _MV.data[0] = _z_offset_LF_leg + _M_LF_leg;
  _MV.data[1] = _z_offset_LR_leg + _M_LR_leg;
  _MV.data[2] = _z_offset_RR_leg + _M_RR_leg;
  _MV.data[3] = _z_offset_RF_leg + _M_RF_leg;

  for(int i=0; i<4; i++){
    if(_MV.data[i] > 160.0){
      _MV.data[i]  = 160.0;
    }else if(_MV.data[i] < 15.0){
      _MV.data[i]  = 15.0;
    }
  }

  _pub_stabilization_variable.publish(_MV);
}

int main(int argc, char** argv){
  ros::init(argc,argv, "posture_stabilization");
  postureStabilization PS;
  ros::Rate loop_rate(10);
  while (ros::ok()){
    PS.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
