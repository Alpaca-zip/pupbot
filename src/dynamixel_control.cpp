//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Tuesday, April 12, 2022

#include "dynamixel_control.h"

/* ++++++++++++++++++++++++++++++++++
       Dynamixel_Control class
++++++++++++++++++++++++++++++++++ */
Dynamixel_Control::Dynamixel_Control(){
  init();
}

void Dynamixel_Control::init(){
  baud_rate = BAUD_RATE;
  model_number = MODEL_NUMBER;
  result = false;
  sub_LF_leg = nh.subscribe("/LF_leg_controller/command", 10, &Dynamixel_Control::monitor_LF_leg_callback, this);
  sub_LR_leg = nh.subscribe("/LR_leg_controller/command", 10, &Dynamixel_Control::monitor_LR_leg_callback, this);
  sub_RR_leg = nh.subscribe("/RR_leg_controller/command", 10, &Dynamixel_Control::monitor_RR_leg_callback, this);
  sub_RF_leg = nh.subscribe("/RF_leg_controller/command", 10, &Dynamixel_Control::monitor_RF_leg_callback, this);
  joint_name[0].data = "LF_leg_shoulder_joint";
  joint_name[1].data = "LF_leg_upper_joint";
  joint_name[2].data = "LF_leg_lower_joint";
  joint_name[3].data = "LR_leg_shoulder_joint";
  joint_name[4].data = "LR_leg_upper_joint";
  joint_name[5].data = "LR_leg_lower_joint";
  joint_name[6].data = "RR_leg_shoulder_joint";
  joint_name[7].data = "RR_leg_upper_joint";
  joint_name[8].data = "RR_leg_lower_joint";
  joint_name[9].data = "RF_leg_shoulder_joint";
  joint_name[10].data = "RF_leg_upper_joint";
  joint_name[11].data = "RF_leg_lower_joint";
  joint_pos[0].data = 0;
  joint_pos[1].data = 1.5;
  joint_pos[2].data = 2.5;
  joint_pos[3].data = 0;
  joint_pos[4].data = 1.5;
  joint_pos[5].data = 2.5;
  joint_pos[6].data = 0;
  joint_pos[7].data = -1.5;
  joint_pos[8].data = -2.5;
  joint_pos[9].data = 0;
  joint_pos[10].data = -1.5;
  joint_pos[11].data = -2.5;
  dxl_init();
  dxl_torqueOn();
  dxl_addSyncWriteHandler();
}

void Dynamixel_Control::monitor_LF_leg_callback(const trajectory_msgs::JointTrajectory& LF_leg){
  for(int i=0;i<3;i++){
    joint_pos[i].data = LF_leg.points[0].positions[i];
  }
}

void Dynamixel_Control::monitor_LR_leg_callback(const trajectory_msgs::JointTrajectory& LR_leg){
  for(int i=3;i<6;i++){
    joint_pos[i].data = LR_leg.points[0].positions[i-3];
  }
}

void Dynamixel_Control::monitor_RR_leg_callback(const trajectory_msgs::JointTrajectory& RR_leg){
  for(int i=6;i<9;i++){
    joint_pos[i].data = RR_leg.points[0].positions[i-6];
  }
}

void Dynamixel_Control::monitor_RF_leg_callback(const trajectory_msgs::JointTrajectory& RF_leg){
  for(int i=9;i<12;i++){
    joint_pos[i].data = RF_leg.points[0].positions[i-9];
  }
}

void Dynamixel_Control::controlLoop(){
  for(int i=0;i<12;i++){
    goal_position[i] = 512+3.41*180*(joint_pos[i].data/M_PI);
  }
  result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to sync write position\n");
  }
}

void Dynamixel_Control::dxl_init(){
  result = dxl_wb.init(port_name, baud_rate, &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to init\n");
  }else printf("Succeed to init(%d)\n", baud_rate);
}

void Dynamixel_Control::dxl_torqueOn(){
  for(int cnt=0;cnt<12;cnt++){
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    if(result == false){
      printf("%s\n", log);
      printf("Failed to ping\n");
    }else{
      printf("Succeeded to ping\n");
      printf("id : %d, model_number : %d\n", dxl_id[cnt], model_number);
    }
    result = dxl_wb.torqueOn(dxl_id[cnt]);
    if(result == false) printf("torqueOn ERROR \n");
  }
}

void Dynamixel_Control::dxl_addSyncWriteHandler(){
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to add sync write handler\n");
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char **argv){
  ros::init(argc, argv, "dynamixel_control");
  Dynamixel_Control dynamixel_control;

  ros::Rate loop_rate(100);

  while(ros::ok()){
    dynamixel_control.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}