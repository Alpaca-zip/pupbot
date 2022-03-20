//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Sunday, March 20, 2022

#include "posture_stabilization.h"

/* ++++++++++++++++++++++++++++++++++
     posture_stabilization class
++++++++++++++++++++++++++++++++++ */
Posture_Stabilization::Posture_Stabilization(){
  init();
}

void Posture_Stabilization::init(){
  MV.data.resize(4);
  MV.data[0] = LF_LEG_Z_OFFSET;
  MV.data[1] = LR_LEG_Z_OFFSET;
  MV.data[2] = RR_LEG_Z_OFFSET;
  MV.data[3] = RF_LEG_Z_OFFSET;
  roll_LPF.data = 0.0;
  pitch_LPF.data = 0.0;
  pub_stabilization_variable = nh.advertise<std_msgs::Float64MultiArray>("/stabilization_variable", 10);
  pub_roll_LPF = nh.advertise<std_msgs::Float64>("/roll_LPF", 10);
  pub_pitch_LPF = nh.advertise<std_msgs::Float64>("/pitch_LPF", 10);
  roll_sub = nh.subscribe("/roll", 10, &Posture_Stabilization::roll_callback, this);
  pitch_sub = nh.subscribe("/pitch", 10, &Posture_Stabilization::pitch_callback, this);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  key_control_sub_Kp = nh.subscribe("/key_control_Kp", 10, &Posture_Stabilization::Kp_callback, this);
  key_control_sub_Ki = nh.subscribe("/key_control_Ki", 10, &Posture_Stabilization::Ki_callback, this);
  key_control_sub_Kd = nh.subscribe("/key_control_Kd", 10, &Posture_Stabilization::Kd_callback, this);
  key_control_sub_PID = nh.subscribe("/PID_on_off", 10, &Posture_Stabilization::PID_callback, this);
  PID_on = false;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  M_LF_leg = M_LR_leg = M_RR_leg = M_RF_leg = 0.0;
  M1_LF_leg = M1_LR_leg = M1_RR_leg = M1_RF_leg = 0.0;
  e_LF_leg = e_LR_leg = e_RR_leg = e_RF_leg = 0.0;
  e1_LF_leg = e1_LR_leg = e1_RR_leg = e1_RF_leg = 0.0;
  e2_LF_leg = e2_LR_leg = e2_RR_leg = e2_RF_leg = 0.0;

  P = 0.0;
  I = 0.0;
  D = 0.0;
  roll_sum = 0.0;
  pitch_sum = 0.0;
  roll_cnt = 0;
  pitch_cnt = 0;
}

void Posture_Stabilization::roll_callback(const std_msgs::Float64& roll){
  roll_data = roll.data;
  if (roll_cnt == WIDTH) roll_cnt = 0;
  roll_sum -= buff_roll[roll_cnt];
  buff_roll[roll_cnt] = roll_data;
  roll_sum += buff_roll[roll_cnt];
  roll_cnt++;
  roll_LPF.data = roll_sum/WIDTH;
  pub_roll_LPF.publish(roll_LPF);
}

void Posture_Stabilization::pitch_callback(const std_msgs::Float64& pitch){
  pitch_data = pitch.data;
  if (pitch_cnt == WIDTH) pitch_cnt = 0;
  pitch_sum -= buff_pitch[pitch_cnt];
  buff_pitch[pitch_cnt] = pitch_data;
  pitch_sum += buff_pitch[pitch_cnt];
  pitch_cnt++;
  pitch_LPF.data = pitch_sum/WIDTH;
  pub_pitch_LPF.publish(pitch_LPF);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
void Posture_Stabilization::Kp_callback(const std_msgs::Float64& Kp){
  P = Kp.data;
}

void Posture_Stabilization::Ki_callback(const std_msgs::Float64& Ki){
  I = Ki.data;
}

void Posture_Stabilization::Kd_callback(const std_msgs::Float64& Kd){
  D = Kd.data;
}

void Posture_Stabilization::PID_callback(const std_msgs::Bool& PID){
  if(PID_on){
    PID_on = false;
  }else{
    PID_on = true;
  }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Posture_Stabilization::controlLoop(){
  M1_LF_leg = M_LF_leg;
  M1_LR_leg = M_LR_leg;
  M1_RR_leg = M_RR_leg;
  M1_RF_leg = M_RF_leg;
  e2_LF_leg = e1_LF_leg;
  e2_LR_leg = e1_LR_leg;
  e2_RR_leg = e1_RR_leg;
  e2_RF_leg = e1_RF_leg;
  e1_LF_leg = e_LF_leg;
  e1_LR_leg = e_LR_leg;
  e1_RR_leg = e_RR_leg;
  e1_RF_leg = e_RF_leg;

  e_LF_leg = -Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)+X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_LR_leg = -Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)-X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_RR_leg = Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)-X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);
  e_RF_leg = Y_OFFSET*tan(roll_LPF.data/180.0*M_PI)+X_OFFSET*tan(pitch_LPF.data/180.0*M_PI);

  if(PID_on){
    M_LF_leg = M1_LF_leg+P*(e_LF_leg-e1_LF_leg)+I*e_LF_leg+D*((e_LF_leg-e1_LF_leg)-(e1_LF_leg-e2_LF_leg));
    M_LR_leg = M1_LR_leg+P*(e_LR_leg-e1_LR_leg)+I*e_LR_leg+D*((e_LR_leg-e1_LR_leg)-(e1_LR_leg-e2_LR_leg));
    M_RR_leg = M1_RR_leg+P*(e_RR_leg-e1_RR_leg)+I*e_RR_leg+D*((e_RR_leg-e1_RR_leg)-(e1_RR_leg-e2_RR_leg));
    M_RF_leg = M1_RF_leg+P*(e_RF_leg-e1_RR_leg)+I*e_RF_leg+D*((e_RF_leg-e1_RF_leg)-(e1_RF_leg-e2_RF_leg));
  }else{
    M_LF_leg = M1_LF_leg+0*(e_LF_leg-e1_LF_leg)+0*e_LF_leg+0*((e_LF_leg-e1_LF_leg)-(e1_LF_leg-e2_LF_leg));
    M_LR_leg = M1_LR_leg+0*(e_LR_leg-e1_LR_leg)+0*e_LR_leg+0*((e_LR_leg-e1_LR_leg)-(e1_LR_leg-e2_LR_leg));
    M_RR_leg = M1_RR_leg+0*(e_RR_leg-e1_RR_leg)+0*e_RR_leg+0*((e_RR_leg-e1_RR_leg)-(e1_RR_leg-e2_RR_leg));
    M_RF_leg = M1_RF_leg+0*(e_RF_leg-e1_RF_leg)+0*e_RF_leg+0*((e_RF_leg-e1_RF_leg)-(e1_RF_leg-e2_RF_leg));
  }

  MV.data[0] = LF_LEG_Z_OFFSET+M_LF_leg;
  MV.data[1] = LR_LEG_Z_OFFSET+M_LR_leg;
  MV.data[2] = RR_LEG_Z_OFFSET+M_RR_leg;
  MV.data[3] = RF_LEG_Z_OFFSET+M_RF_leg;

  for(int i=0;i<4;i++){
    if(MV.data[i] > 160.0){
      MV.data[i]  = 160.0;
    }else if(MV.data[i] < 15.0){
      MV.data[i]  = 15.0;
    }
  }

  pub_stabilization_variable.publish(MV);
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "posture_stabilization");
  Posture_Stabilization posture_stabilization;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    posture_stabilization.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
