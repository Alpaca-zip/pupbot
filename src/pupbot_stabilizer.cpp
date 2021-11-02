#include "pupbot_stabilizer.h"

/* ++++++++++++++++++++++++++++++++++
       Pupbot_stabilizer class
++++++++++++++++++++++++++++++++++ */
Pupbot_stabilizer::Pupbot_stabilizer(){
  init();
}

void Pupbot_stabilizer::init(){
  leftfront_leg_z_offset.data = LEFTFRONTLEG_Z_OFFSET;
  leftback_leg_z_offset.data = LEFTBACKLEG_Z_OFFSET;
  rightfront_leg_z_offset.data = RIGHTFRONTLEG_Z_OFFSET;
  rightback_leg_z_offset.data = RIGHTBACKLEG_Z_OFFSET;
  pub_leftfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftfront_leg_z_offset", 10);
  pub_leftback_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftback_leg_z_offset", 10);
  pub_rightfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightfront_leg_z_offset", 10);
  pub_rightback_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightback_leg_z_offset", 10);
  roll_sub = nh.subscribe("/roll", 10, &Pupbot_stabilizer::roll_callback, this);
  pitch_sub = nh.subscribe("/pitch", 10, &Pupbot_stabilizer::pitch_callback, this);
}

void Pupbot_stabilizer::roll_callback(const std_msgs::Float64& roll){
  roll_data = roll.data;
}

void Pupbot_stabilizer::pitch_callback(const std_msgs::Float64& pitch){
  pitch_data = pitch.data;
}

void Pupbot_stabilizer::controlLoop(){
  leftfront_leg_z_offset.data = LEFTFRONTLEG_Z_OFFSET;
  leftback_leg_z_offset.data = LEFTBACKLEG_Z_OFFSET;
  rightfront_leg_z_offset.data = RIGHTFRONTLEG_Z_OFFSET;
  rightback_leg_z_offset.data = RIGHTBACKLEG_Z_OFFSET;

  leftfront_leg_z_offset.data += Y_OFFSET*sin(roll_data/180.0 * M_PI);
  leftback_leg_z_offset.data += Y_OFFSET*sin(roll_data/180.0 * M_PI);
  rightfront_leg_z_offset.data -= Y_OFFSET*sin(roll_data/180.0 * M_PI);
  rightback_leg_z_offset.data -= Y_OFFSET*sin(roll_data/180.0 * M_PI);
  
  if(leftfront_leg_z_offset.data > 160.0) leftfront_leg_z_offset.data  = 160.0;
  if(leftback_leg_z_offset.data > 160.0) leftback_leg_z_offset.data  = 160.0;
  if(rightfront_leg_z_offset.data > 160.0) rightfront_leg_z_offset.data  = 160.0;
  if(rightback_leg_z_offset.data > 160.0) rightback_leg_z_offset.data  = 160.0;
  
  if(leftfront_leg_z_offset.data < 15.0) leftfront_leg_z_offset.data  = 15.0;
  if(leftback_leg_z_offset.data < 15.0) leftback_leg_z_offset.data  = 15.0;
  if(rightfront_leg_z_offset.data < 15.0) rightfront_leg_z_offset.data  = 15.0;
  if(rightback_leg_z_offset.data < 15.0) rightback_leg_z_offset.data  = 15.0;

  leftfront_leg_z_offset.data -= X_OFFSET*sin(pitch_data/180.0 * M_PI);
  leftback_leg_z_offset.data += X_OFFSET*sin(pitch_data/180.0 * M_PI);
  rightfront_leg_z_offset.data -= X_OFFSET*sin(pitch_data/180.0 * M_PI);
  rightback_leg_z_offset.data += X_OFFSET*sin(pitch_data/180.0 * M_PI);
  
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
