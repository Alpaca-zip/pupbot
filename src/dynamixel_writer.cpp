#include "dynamixel_writer.h"

/* ++++++++++++++++++++++++++++++++++
       Dynamixel_Writer class
++++++++++++++++++++++++++++++++++ */
Dynamixel_Writer::Dynamixel_Writer(){
  init();
}

void Dynamixel_Writer::init(){
  baud_rate = 1000000;
  model_number = 12;
  result = false;
  sub_joints = nh.subscribe("/joint_states", 1, &Dynamixel_Writer::monitorJointState_callback, this);
  joint_name[0].data = "leftfront_leg_shoulder_joint";
  joint_name[1].data = "leftfront_leg_upper_joint";
  joint_name[2].data = "leftfront_leg_lower_joint";
  dxl_init();
  dxl_torqueOn();
  dxl_addSyncWriteHandler();
}

void Dynamixel_Writer::monitorJointState_callback(const sensor_msgs::JointState::ConstPtr& jointstate){
  for(int i=0;i<3;i++){
    for(int j=0;j<12;j++){
      if(joint_name[i].data == jointstate->name[j]){
        joint_pos[i].data = jointstate->position[j];
      }
    }
  }
}

void Dynamixel_Writer::controlLoop(){
  for(int i=0;i<3;i++){
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
  for(int cnt=0;cnt<3;cnt++){
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

  ros::Rate loop_rate(50);

  while(ros::ok()){
    PupBot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}