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
  roll_LPF.data = 0.0;
  pitch_LPF.data = 0.0;
  pub_leftfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftfront_leg_z_offset", 10);
  pub_leftback_leg_z_offset = nh.advertise<std_msgs::Float64>("/leftback_leg_z_offset", 10);
  pub_rightfront_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightfront_leg_z_offset", 10);
  pub_rightback_leg_z_offset = nh.advertise<std_msgs::Float64>("/rightback_leg_z_offset", 10);
  pub_roll_LPF = nh.advertise<std_msgs::Float64>("/roll_LPF", 10);
  pub_pitch_LPF = nh.advertise<std_msgs::Float64>("/pitch_LPF", 10);
  roll_sub = nh.subscribe("/roll", 10, &Pupbot_stabilizer::roll_callback, this);
  pitch_sub = nh.subscribe("/pitch", 10, &Pupbot_stabilizer::pitch_callback, this);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  key_control_sub_Kp = nh.subscribe("/key_control_Kp", 10, &Pupbot_stabilizer::Kp_callback, this);
  key_control_sub_Ki = nh.subscribe("/key_control_Ki", 10, &Pupbot_stabilizer::Ki_callback, this);
  key_control_sub_Kd = nh.subscribe("/key_control_Kd", 10, &Pupbot_stabilizer::Kd_callback, this);
  key_control_sub_PID = nh.subscribe("/PID_on_off", 10, &Pupbot_stabilizer::PID_callback, this);
  PID_on = false;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  M_leftfront_leg = M_leftback_leg = M_rightfront_leg = M_rightback_leg = 0.0;
  M1_leftfront_leg = M1_leftback_leg = M1_rightfront_leg = M1_rightback_leg = 0.0;
  e_leftfront_leg = e_leftback_leg = e_rightfront_leg = e_rightback_leg = 0.0;
  e1_leftfront_leg = e1_leftback_leg = e1_rightfront_leg = e1_rightback_leg = 0.0;
  e2_leftfront_leg = e2_leftback_leg = e2_rightfront_leg = e2_rightback_leg = 0.0;
  P = 0.0;
  I = 0.0;
  D = 0.0;
  roll_sum = 0.0;
  pitch_sum = 0.0;
  roll_cnt = 0;
  pitch_cnt = 0;
}

void Pupbot_stabilizer::roll_callback(const std_msgs::Float64& roll){
  roll_data = roll.data;
  if (roll_cnt == WIDTH) roll_cnt = 0;
  roll_sum -= buff_roll[roll_cnt];
  buff_roll[roll_cnt] = roll_data;
  roll_sum += buff_roll[roll_cnt];
  roll_cnt++;
  roll_LPF.data = roll_sum / WIDTH;
  pub_roll_LPF.publish(roll_LPF);
}

void Pupbot_stabilizer::pitch_callback(const std_msgs::Float64& pitch){
  pitch_data = pitch.data;
  if (pitch_cnt == WIDTH) pitch_cnt = 0;
  pitch_sum -= buff_pitch[pitch_cnt];
  buff_pitch[pitch_cnt] = pitch_data;
  pitch_sum += buff_pitch[pitch_cnt];
  pitch_cnt++;
  pitch_LPF.data = pitch_sum / WIDTH;
  pub_pitch_LPF.publish(pitch_LPF);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
void Pupbot_stabilizer::Kp_callback(const std_msgs::Float64& Kp){
  P = Kp.data;
}

void Pupbot_stabilizer::Ki_callback(const std_msgs::Float64& Ki){
  I = Ki.data;
}

void Pupbot_stabilizer::Kd_callback(const std_msgs::Float64& Kd){
  D = Kd.data;
}

void Pupbot_stabilizer::PID_callback(const std_msgs::Bool& PID){
  if(PID_on){
    PID_on = false;
  }else{
    PID_on = true;
  }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

  if(PID_on){
    M_leftfront_leg = M1_leftfront_leg + P * (e_leftfront_leg - e1_leftfront_leg) + I * e_leftfront_leg + D * ((e_leftback_leg - e1_leftback_leg) - (e1_leftback_leg - e2_leftback_leg));
    M_leftback_leg = M1_leftback_leg + P * (e_leftback_leg - e1_leftback_leg) + I * e_leftback_leg + D * ((e_leftback_leg - e1_leftback_leg) - (e1_leftback_leg - e2_leftback_leg));
    M_rightfront_leg = M1_rightfront_leg + P * (e_rightfront_leg - e1_rightfront_leg) + I * e_rightfront_leg + D * ((e_rightfront_leg - e1_rightfront_leg) - (e1_rightfront_leg - e2_rightfront_leg));
    M_rightback_leg = M1_rightback_leg + P * (e_rightback_leg - e1_rightback_leg) + I * e_rightback_leg + D * ((e_rightback_leg - e1_rightback_leg) - (e1_rightback_leg - e2_rightback_leg));
  }else{
    M_leftfront_leg = M1_leftfront_leg + 0 * (e_leftfront_leg - e1_leftfront_leg) + 0 * e_leftfront_leg + 0 * ((e_leftback_leg - e1_leftback_leg) - (e1_leftback_leg - e2_leftback_leg));
    M_leftback_leg = M1_leftback_leg + 0 * (e_leftback_leg - e1_leftback_leg) + 0 * e_leftback_leg + 0 * ((e_leftback_leg - e1_leftback_leg) - (e1_leftback_leg - e2_leftback_leg));
    M_rightfront_leg = M1_rightfront_leg + 0 * (e_rightfront_leg - e1_rightfront_leg) + 0 * e_rightfront_leg + 0 * ((e_rightfront_leg - e1_rightfront_leg) - (e1_rightfront_leg - e2_rightfront_leg));
    M_rightback_leg = M1_rightback_leg + 0 * (e_rightback_leg - e1_rightback_leg) + 0 * e_rightback_leg + 0 * ((e_rightback_leg - e1_rightback_leg) - (e1_rightback_leg - e2_rightback_leg));
  }

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
