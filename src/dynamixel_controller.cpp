#include "dynamixel_controller.h"

/* ++++++++++++++++++++++++++++++++++
       Dynamixel_Controller class
++++++++++++++++++++++++++++++++++ */
Dynamixel_Controller::Dynamixel_Controller(){
  init();
}

void Dynamixel_Controller::init(){
  baud_rate = BAUD_RATE;
  model_number = MODEL_NUMBER;
  result = false;
  dynamixel_state_leftfront_leg = nh.advertise<std_msgs::Int32>("/dynamixel_state/leftfront_leg_position", 10);
  dynamixel_state_leftback_leg = nh.advertise<std_msgs::Int32>("/dynamixel_state/leftback_leg_position", 10);
  dynamixel_state_rightfront_leg = nh.advertise<std_msgs::Int32>("/dynamixel_state/rightfront_leg_position", 10);
  dynamixel_state_rightback_leg = nh.advertise<std_msgs::Int32>("/dynamixel_state/rightback_leg_position", 10);
  sub_leftfront_leg = nh.subscribe("/leftfront_leg_controller/command", 10, &Dynamixel_Controller::monitor_leftfront_leg_callback, this);
  sub_rightfront_leg = nh.subscribe("/rightfront_leg_controller/command", 10, &Dynamixel_Controller::monitor_rightfront_leg_callback, this);
  sub_leftback_leg = nh.subscribe("/leftback_leg_controller/command", 10, &Dynamixel_Controller::monitor_leftback_leg_callback, this);
  sub_rightback_leg = nh.subscribe("/rightback_leg_controller/command", 10, &Dynamixel_Controller::monitor_rightback_leg_callback, this);
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

void Dynamixel_Controller::monitor_leftfront_leg_callback(const trajectory_msgs::JointTrajectory& leftfront_leg){
  for(int i=0;i<3;i++){
    joint_pos[i].data = leftfront_leg.points[0].positions[i];
  }
  result = dxl_wb.itemRead(LEFTFRONT_LEG_LOWER_ID, "Present_Position", &getdata_from_dynamixel, &log);
  if(result == true){
    leftfront_leg_position.data = getdata_from_dynamixel;
    dynamixel_state_leftfront_leg.publish(leftfront_leg_position);
  }
}

void Dynamixel_Controller::monitor_rightfront_leg_callback(const trajectory_msgs::JointTrajectory& rightfront_leg){
  for(int i=3;i<6;i++){
    joint_pos[i].data = rightfront_leg.points[0].positions[i-3];
  }
  result = dxl_wb.itemRead(RIGHTFRONT_LEG_LOWER_ID, "Present_Position", &getdata_from_dynamixel, &log);
  if(result == true){
    rightfront_leg_position.data = getdata_from_dynamixel;
    dynamixel_state_rightfront_leg.publish(rightfront_leg_position);
  }
}

void Dynamixel_Controller::monitor_leftback_leg_callback(const trajectory_msgs::JointTrajectory& leftback_leg){
  for(int i=6;i<9;i++){
    joint_pos[i].data = leftback_leg.points[0].positions[i-6];
  }
  result = dxl_wb.itemRead(LEFTBACK_LEG_LOWER_ID, "Present_Position", &getdata_from_dynamixel, &log);
  if(result == true){
    leftback_leg_position.data = getdata_from_dynamixel;
    dynamixel_state_leftback_leg.publish(leftback_leg_position);
  }
}

void Dynamixel_Controller::monitor_rightback_leg_callback(const trajectory_msgs::JointTrajectory& rightback_leg){
  for(int i=9;i<12;i++){
    joint_pos[i].data = rightback_leg.points[0].positions[i-9];
  }
  result = dxl_wb.itemRead(RIGHTBACK_LEG_LOWER_ID, "Present_Position", &getdata_from_dynamixel, &log);
  if(result == true){
    rightback_leg_position.data = getdata_from_dynamixel;
    dynamixel_state_rightback_leg.publish(rightback_leg_position);
  }
}

void Dynamixel_Controller::controlLoop(){
  for(int i=0;i<12;i++){
    goal_position[i] = 512+3.41*180*(joint_pos[i].data/M_PI);
  }
  result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to sync write position\n");
  }
}

void Dynamixel_Controller::dxl_init(){
  result = dxl_wb.init(port_name, baud_rate, &log);
  if(result == false){
    printf("%s\n", log);
    printf("Failed to init\n");
  }else printf("Succeed to init(%d)\n", baud_rate);
}

void Dynamixel_Controller::dxl_torqueOn(){
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

void Dynamixel_Controller::dxl_addSyncWriteHandler(){
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
  ros::init(argc, argv, "dynamixel_controller"); 
  Dynamixel_Controller PupBot;

  ros::Rate loop_rate(100);

  while(ros::ok()){
    PupBot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}