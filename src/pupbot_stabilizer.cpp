#include "pupbot_stabilizer.h"

/* ++++++++++++++++++++++++++++++++++
       Pupbot_stabilizer class
++++++++++++++++++++++++++++++++++ */
Pupbot_stabilizer::Pupbot_stabilizer(){
  init();
}

void Pupbot_stabilizer::init(){
  leftfront_leg_state = 0;
  leftback_leg_state = 0;
  rightfront_leg_state = 0;
  rightback_leg_state = 0;
  sub_dynamixel_state_leftfront_leg = nh.subscribe("/dynamixel_state/leftfront_leg_load", 10, &Pupbot_stabilizer::monitor_leftfront_leg_load_callback, this);
  sub_dynamixel_state_leftback_leg = nh.subscribe("/dynamixel_state/leftback_leg_load", 10, &Pupbot_stabilizer::monitor_leftback_leg_load_callback, this);
  sub_dynamixel_state_rightfront_leg = nh.subscribe("/dynamixel_state/rightfront_leg_load", 10, &Pupbot_stabilizer::monitor_rightfront_leg_load_callback, this);
  sub_dynamixel_state_rightback_leg = nh.subscribe("/dynamixel_state/rightback_leg_load", 10, &Pupbot_stabilizer::monitor_rightback_leg_load_callback, this);
}

void Pupbot_stabilizer::monitor_leftfront_leg_load_callback(const std_msgs::Int32& leftfront_leg_load){
  leftfront_leg_state = leftfront_leg_load.data;
}

void Pupbot_stabilizer::monitor_leftback_leg_load_callback(const std_msgs::Int32& leftback_leg_load){
  leftback_leg_state = leftback_leg_load.data;
}

void Pupbot_stabilizer::monitor_rightfront_leg_load_callback(const std_msgs::Int32& rightfront_leg_load){
  rightfront_leg_state = rightfront_leg_load.data;
}

void Pupbot_stabilizer::monitor_rightback_leg_load_callback(const std_msgs::Int32& rightback_leg_load){
  rightback_leg_state = rightback_leg_load.data;
}

void Pupbot_stabilizer::controlLoop(){

}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "pupbot_stabilizer");
  Pupbot_stabilizer pupbot;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    pupbot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
