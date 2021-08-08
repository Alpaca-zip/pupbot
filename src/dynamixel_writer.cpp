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
  joint_name[0].data = "leftfront_leg_shoulder_joint";
  joint_name[1].data = "leftfront_leg_upper_joint";
  joint_name[2].data = "leftfront_leg_lower_joint";
  dxl_init();
  dxl_torqueOn();
  dxl_addSyncWriteHandler();
}

void Dynamixel_Writer::monitor_leftfront_leg_callback(const trajectory_msgs::JointTrajectory& leftfront_leg){
  for(int i=0;i<3;i++){
    joint_pos[i].data = leftfront_leg.points[0].positions[i];
  }
}

void Dynamixel_Writer::controlLoop(){
  for(int i=0;i<3;i++){
    goal_position[i] = 512+3.41*180*(joint_pos[i].data/M_PI);
  }
  result = dxl_wb.syncWrite(handler_index, &goal_position[0], &log);
  if(result == false){
    std::cout << log << std::endl;
    std::cout << "Failed to sync write position" << std::endl;
  }
}

void Dynamixel_Writer::dxl_init(){
  result = dxl_wb.init(port_name, baud_rate, &log);
  if(result == false){
    std::cout << log << std::endl;
    std::cout << "Failed to init" << std::endl;
  }else std::cout << "Succeed to init" << baud_rate << std::endl;
}

void Dynamixel_Writer::dxl_torqueOn(){
  for(int cnt=0;cnt<3;cnt++){
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    if(result == false){
      std::cout << log << std::endl;
      std::cout << "Failed to ping" << std::endl;
    }else{
      std::cout << "Succeeded to ping" << std::endl;
      std::cout << "id : " << dxl_id[cnt] << ", model_number : " << model_number << std::endl;
    }
    result = dxl_wb.torqueOn(dxl_id[cnt]);
    if(result == false) std::cout << "torqueOn ERROR" << std::endl;
  }
}

void Dynamixel_Writer::dxl_addSyncWriteHandler(){
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  if(result == false){
    std::cout << log << std::endl;
    std::cout << "Failed to add sync write handler" << std::endl;
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