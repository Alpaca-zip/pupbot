#include "pupbot_stabilizer.h"

/* ++++++++++++++++++++++++++++++++++
       Pupbot_stabilizer class
++++++++++++++++++++++++++++++++++ */
Pupbot_stabilizer::Pupbot_stabilizer(){
  init();
  leftfront_leg_z_offset.data = Z_OFFSET;
  leftback_leg_z_offset.data = Z_OFFSET;
  rightfront_leg_z_offset.data = Z_OFFSET;
  rightback_leg_z_offset.data = Z_OFFSET;
  pub_leftfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftfront_leg_z_offset", 10);
  pub_leftback_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftback_leg_z_offset", 10);
  pub_rightfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightfront_leg_z_offset", 10);
  pub_rightback_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightback_leg_z_offset", 10);
}

void Pupbot_stabilizer::init(){
  
}

void Pupbot_stabilizer::controlLoop(){
  pub_leftfront_leg_z_offset.publish(leftfront_leg_z_offset);
  pub_leftback_leg_z_offset.publish(leftback_leg_z_offset);
  pub_rightfront_leg_z_offset.publish(rightfront_leg_z_offset);
  pub_rightback_leg_z_offset.publish(rightback_leg_z_offset);
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