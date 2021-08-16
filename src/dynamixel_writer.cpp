#include "dynamixel_writer.h"

/* ++++++++++++++++++++++++++++++++++
       Dynamixel_Writer class
++++++++++++++++++++++++++++++++++ */
Dynamixel_Writer::Dynamixel_Writer(){
  init();
}

void Dynamixel_Writer::init(){
  baud_rate = BAUD_RATE;
  model_number = MODEL_NUMBER;
  result = false;
  sub_leftfront_leg = nh.subscribe("/leftfront_leg_controller/command", 10, &Dynamixel_Writer::monitor_leftfront_leg_callback, this);
  sub_rightfront_leg = nh.subscribe("/rightfront_leg_controller/command", 10, &Dynamixel_Writer::monitor_rightfront_leg_callback, this);
  sub_leftback_leg = nh.subscribe("/leftback_leg_controller/command", 10, &Dynamixel_Writer::monitor_leftback_leg_callback, this);
  sub_rightback_leg = nh.subscribe("/rightback_leg_controller/command", 10, &Dynamixel_Writer::monitor_rightback_leg_callback, this);
  joint_name[0].data = "leftfront_leg_shoulder_joint";
  joint_name[1].data = "leftfront_leg_upper_joint";
  joint_name[2].data = "leftfront_leg_lower_joint";
  joint_name[3].data = "rightfront_leg_shoulder_joint";
  joint_name[4].data = "rightfront_leg_upper_joint";
  joint_name[5].data = "rightfront_leg_lower_joint";
  joint_name[6].data = "leftback_leg_shoulder_joint";
  joint_name[7].data = "leftback_leg_upper_joint";
  joint_name[8].data = "leftback_leg_lower_joint";
  joint_name[9].data = "rightback_leg_shoulder_joint";
  joint_name[10].data = "rightback_leg_upper_joint";
  joint_name[11].data = "rightback_leg_lower_joint";
  joint_pos[0].data = 0;
  joint_pos[1].data = 1.5;
  joint_pos[2].data = 2.5;
  joint_pos[3].data = 0;
  joint_pos[4].data = -1.5;
  joint_pos[5].data = -2.5;
  joint_pos[6].data = 0;
  joint_pos[7].data = 1.5;
  joint_pos[8].data = 2.5;
  joint_pos[9].data = 0;
  joint_pos[10].data = -1.5;
  joint_pos[11].data = -2.5;
  dxl_init();
  dxl_torqueOn();
  dxl_addSyncWriteHandler();
}

void Dynamixel_Writer::monitor_leftfront_leg_callback(const trajectory_msgs::JointTrajectory& leftfront_leg){
  for(int i=0;i<3;i++){
    joint_pos[i].data = leftfront_leg.points[0].positions[i];
  }
}

void Dynamixel_Writer::monitor_rightfront_leg_callback(const trajectory_msgs::JointTrajectory& rightfront_leg){
  for(int i=3;i<6;i++){
    joint_pos[i].data = rightfront_leg.points[0].positions[i-3];
  }
}

void Dynamixel_Writer::monitor_leftback_leg_callback(const trajectory_msgs::JointTrajectory& leftback_leg){
  for(int i=6;i<9;i++){
    joint_pos[i].data = leftback_leg.points[0].positions[i-6];
  }
}

void Dynamixel_Writer::monitor_rightback_leg_callback(const trajectory_msgs::JointTrajectory& rightback_leg){
  for(int i=9;i<12;i++){
    joint_pos[i].data = rightback_leg.points[0].positions[i-9];
  }
}

void Dynamixel_Writer::controlLoop(){
  for(int i=0;i<12;i++){
    goal_position[i] = 512+3.41*180*(joint_pos[i].data/M_PI);
  }
  result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to sync write position\n");
  }
}

void Dynamixel_Writer::dxl_init(){
  result = dxl_wb.init(port_name, baud_rate, &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to init\n");
  }else printf("Succeed to init(%d)\n", baud_rate);
}

void Dynamixel_Writer::dxl_torqueOn(){
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

void Dynamixel_Writer::dxl_addSyncWriteHandler(){
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
  ros::init(argc, argv, "dynamixel_writer"); 
  Dynamixel_Writer PupBot;

  ros::Rate loop_rate(100);

  while(ros::ok()){
    PupBot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}