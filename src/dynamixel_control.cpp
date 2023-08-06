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

#include "dynamixel_control.h"

dynamixelControl::dynamixelControl() : _pnh("~"){
  _pnh.param<std::string>("port", _port_name_str, "/dev/ttyACM1");
  _pnh.param<int>("model_number", _model_number_int, 12);
  _pnh.param<int>("baudrate", _baudrate, 1000000);
  if (_pnh.getParam("dxl_ids", _dxl_id_vector) && _dxl_id_vector.size() == 12) {
    for (int i = 0; i < 12; ++i) {
      _dxl_id[i] = static_cast<uint8_t>(_dxl_id_vector[i]);
    }
  } else {
    _dxl_id[0] = LF_LEG_SHOULDER_ID;
    _dxl_id[1] = LF_LEG_UPPER_ID;
    _dxl_id[2] = LF_LEG_LOWER_ID;
    _dxl_id[3] = LR_LEG_SHOULDER_ID;
    _dxl_id[4] = LR_LEG_UPPER_ID;
    _dxl_id[5] = LR_LEG_LOWER_ID;
    _dxl_id[6] = RR_LEG_SHOULDER_ID;
    _dxl_id[7] = RR_LEG_UPPER_ID;
    _dxl_id[8] = RR_LEG_LOWER_ID;
    _dxl_id[9] = RF_LEG_SHOULDER_ID;
    _dxl_id[10] = RF_LEG_UPPER_ID;
    _dxl_id[11] = RF_LEG_LOWER_ID;
  }

  _sub_LF_leg = _nh.subscribe("leftfront_leg_controller/command", 10, &dynamixelControl::monitorLFLegCallback, this);
  _sub_LR_leg = _nh.subscribe("leftback_leg_controller/command", 10, &dynamixelControl::monitorLRLegCallback, this);
  _sub_RR_leg = _nh.subscribe("rightback_leg_controller/command", 10, &dynamixelControl::monitorRRLegCallback, this);
  _sub_RF_leg = _nh.subscribe("rightfront_leg_controller/command", 10, &dynamixelControl::monitorRFLegCallback, this);

  _joint_pos[0].data = 0; // LF_leg_shoulder_joint
  _joint_pos[1].data = 1.5; // LF_leg_upper_joint
  _joint_pos[2].data = 2.5; // LF_leg_lower_joint
  _joint_pos[3].data = 0; // LR_leg_shoulder_joint
  _joint_pos[4].data = 1.5; // LR_leg_upper_joint
  _joint_pos[5].data = 2.5; // LR_leg_lower_joint
  _joint_pos[6].data = 0; // RR_leg_shoulder_joint
  _joint_pos[7].data = -1.5; // RR_leg_upper_joint
  _joint_pos[8].data = -2.5; // RR_leg_lower_joint
  _joint_pos[9].data = 0; // RF_leg_shoulder_joint
  _joint_pos[10].data = -1.5; // RF_leg_upper_joint
  _joint_pos[11].data = -2.5; // RF_leg_lower_joint
  _result = false;
  _model_number = static_cast<uint16_t>(_model_number_int);
  _port_name = _port_name_str.c_str();

  dxlInit();
  dxlTorqueOn();
  dxlAddSyncWriteHandler();
}

void dynamixelControl::monitorLFLegCallback(const trajectory_msgs::JointTrajectory& LF_leg){
  for(int i=0;i<3;i++){
    _joint_pos[i].data = LF_leg.points[0].positions[i];
  }
}

void dynamixelControl::monitorLRLegCallback(const trajectory_msgs::JointTrajectory& LR_leg){
  for(int i=3;i<6;i++){
    _joint_pos[i].data = LR_leg.points[0].positions[i-3];
  }
}

void dynamixelControl::monitorRRLegCallback(const trajectory_msgs::JointTrajectory& RR_leg){
  for(int i=6;i<9;i++){
    _joint_pos[i].data = RR_leg.points[0].positions[i-6];
  }
}

void dynamixelControl::monitorRFLegCallback(const trajectory_msgs::JointTrajectory& RF_leg){
  for(int i=9;i<12;i++){
    _joint_pos[i].data = RF_leg.points[0].positions[i-9];
  }
}

void dynamixelControl::controlLoop(){
  int32_t goal_position[12];
  const uint8_t handler_index = 0;
  for(int i=0;i<12;i++){
    goal_position[i] = 512+3.41*180*(_joint_pos[i].data/M_PI);
  }
  _result = _dxl_wb.syncWrite(handler_index, &goal_position[0], &_log);
  if(_result == false){
    ROS_ERROR("%s", _log);
    ROS_ERROR("Failed to sync write position");
  }
}

void dynamixelControl::dxlInit(){
  _result = _dxl_wb.init(_port_name, _baudrate, &_log);
  if(_result == false){
    ROS_ERROR("%s", _log);
    ROS_ERROR("Failed to init");
  }else ROS_WARN("Succeed to init(%d)", _baudrate);
}

void dynamixelControl::dxlTorqueOn(){
  for(int cnt=0;cnt<12;cnt++){
    _result = _dxl_wb.ping(_dxl_id[cnt], &_model_number, &_log);
    if(_result == false){
      ROS_ERROR("%s", _log);
      ROS_ERROR("Failed to ping");
    }else{
      ROS_WARN("Succeeded to ping");
      ROS_WARN("id : %d, model_number : %d", _dxl_id[cnt], _model_number);
    }
    _result = _dxl_wb.torqueOn(_dxl_id[cnt]);
    if(_result == false) ROS_ERROR("torqueOn ERROR");
  }
}

void dynamixelControl::dxlAddSyncWriteHandler(){
  _result = _dxl_wb.addSyncWriteHandler(_dxl_id[0], "Goal_Position", &_log);
  if(_result == false){
    ROS_ERROR("%s", _log);
    ROS_ERROR("Failed to add sync write handler");
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamixel_control");
  dynamixelControl DC;
  ros::Rate loop_rate(100);
  while(ros::ok()){
    DC.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
