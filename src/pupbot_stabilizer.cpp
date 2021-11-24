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
  M_leftfront_leg = M_leftback_leg = M_rightfront_leg = M_rightback_leg = 0.0;
  M1_leftfront_leg = M1_leftback_leg = M1_rightfront_leg = M1_rightback_leg = 0.0;
  e_leftfront_leg = e_leftback_leg = e_rightfront_leg = e_rightback_leg = 0.0;
  e1_leftfront_leg = e1_leftback_leg = e1_rightfront_leg = e1_rightback_leg = 0.0;
  e2_leftfront_leg = e2_leftback_leg = e2_rightfront_leg = e2_rightback_leg = 0.0;
  Kp = 0.1;
  Ki = 0.1;
  Kd = 0.1;
}

void Pupbot_stabilizer::roll_callback(const std_msgs::Float64& roll){
  roll_data = roll.data;
}

void Pupbot_stabilizer::pitch_callback(const std_msgs::Float64& pitch){
  pitch_data = pitch.data;
}

void Pupbot_stabilizer::controlLoop(){
  M1_leftfront_leg = M_leftfront_leg;
  M1_leftback_leg = M_leftback_leg;
  M1_rightfront_leg = M_rightfront_leg;
  M1_rightback_leg = M_rightback_leg;
  e2_leftfront_leg = e1_leftfront_leg;
  e2_leftback_leg = e1_leftback_leg;
  e2_rightfront_leg = e1_rightfront_leg;
  e2_rightback_leg = e1_rightback_leg;
  e1_leftfront_leg = e_leftfront_leg;
  e1_leftback_leg = e_leftback_leg;
  e1_rightfront_leg = e_rightfront_leg;
  e1_rightback_leg = e_rightback_leg;

  e_leftfront_leg = - Y_OFFSET * sin(roll_data / 180.0 * M_PI) + X_OFFSET * sin(pitch_data / 180.0 * M_PI);
  e_leftback_leg = - Y_OFFSET * sin(roll_data / 180.0 * M_PI) - X_OFFSET * sin(pitch_data / 180.0 * M_PI);
  e_rightfront_leg = Y_OFFSET * sin(roll_data / 180.0 * M_PI) + X_OFFSET * sin(pitch_data / 180.0 * M_PI);
  e_rightback_leg = Y_OFFSET * sin(roll_data / 180.0 * M_PI) - X_OFFSET * sin(pitch_data / 180.0 * M_PI);

  M_leftfront_leg = M1_leftfront_leg + Kp * (e_leftfront_leg - e1_leftfront_leg) + Ki * e_leftfront_leg + Kd * ((e_leftfront_leg - e1_leftfront_leg) - (e1_leftfront_leg - e2_leftfront_leg));
  M_leftback_leg = M1_leftback_leg + Kp * (e_leftback_leg - e1_leftback_leg) + Ki * e_leftback_leg + Kd * ((e_leftback_leg - e1_leftback_leg) - (e1_leftback_leg - e2_leftback_leg));
  M_rightfront_leg = M1_rightfront_leg + Kp * (e_rightfront_leg - e1_rightfront_leg) + Ki * e_rightfront_leg + Kd * ((e_rightfront_leg - e1_rightfront_leg) - (e1_rightfront_leg - e2_rightfront_leg));
  M_rightback_leg = M1_rightback_leg + Kp * (e_rightback_leg - e1_rightback_leg) + Ki * e_rightback_leg + Kd * ((e_rightback_leg - e1_rightback_leg) - (e1_rightback_leg - e2_rightback_leg));

  leftfront_leg_z_offset.data = LEFTFRONTLEG_Z_OFFSET + M_leftfront_leg;
  leftback_leg_z_offset.data = LEFTBACKLEG_Z_OFFSET + M_leftback_leg;
  rightfront_leg_z_offset.data = RIGHTFRONTLEG_Z_OFFSET + M_rightfront_leg;
  rightback_leg_z_offset.data = RIGHTBACKLEG_Z_OFFSET + M_rightback_leg;
  
  if(leftfront_leg_z_offset.data > 160.0) leftfront_leg_z_offset.data  = 160.0;
  if(leftback_leg_z_offset.data > 160.0) leftback_leg_z_offset.data  = 160.0;
  if(rightfront_leg_z_offset.data > 160.0) rightfront_leg_z_offset.data  = 160.0;
  if(rightback_leg_z_offset.data > 160.0) rightback_leg_z_offset.data  = 160.0;
  
  if(leftfront_leg_z_offset.data < 15.0) leftfront_leg_z_offset.data  = 15.0;
  if(leftback_leg_z_offset.data < 15.0) leftback_leg_z_offset.data  = 15.0;
  if(rightfront_leg_z_offset.data < 15.0) rightfront_leg_z_offset.data  = 15.0;
  if(rightback_leg_z_offset.data < 15.0) rightback_leg_z_offset.data  = 15.0;

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
