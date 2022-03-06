//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Saturday, March 5, 2022

#include "standing_motion.h"

/* ++++++++++++++++++++++++++++++++++
       standing_motion class
++++++++++++++++++++++++++++++++++ */
Standing_Motion::Standing_Motion(){
  init();
}

void Standing_Motion::init(){
  pub_leg_position = nh.advertise<std_msgs::Float64MultiArray>("/leg_position", 10);
  pub_stop_signal = nh.advertise<std_msgs::Bool>("/stop_signal", 10);
  sub_standing_motion = nh.subscribe("/standing_motion", 10, &Standing_Motion::standing_motion_callback, this);
  stop.data = true;
  leg_position.data.resize(12);
  leg_position.data[0] = X_OFFSET;
  leg_position.data[3] = X_OFFSET;
  leg_position.data[6] = X_OFFSET;
  leg_position.data[9] = X_OFFSET;
  z_offset_LF_leg = LF_LEG_Z_OFFSET;
  z_offset_LR_leg = LR_LEG_Z_OFFSET;
  z_offset_RR_leg = RR_LEG_Z_OFFSET;
  z_offset_RF_leg = RF_LEG_Z_OFFSET;
}

void Standing_Motion::standing_motion_callback(const std_msgs::Bool& stand){
if(stand.data){
  leg_position.data[2] = 30;
  leg_position.data[5] = 30;
  leg_position.data[8] = 30;
  leg_position.data[11] = 30;
  for(int i=0;i<50;i++){
    leg_position.data[2] += (z_offset_LF_leg-30)/50;
    leg_position.data[5] += (z_offset_LR_leg-30)/50;
    leg_position.data[8] += (z_offset_RR_leg-30)/50;
    leg_position.data[11] += (z_offset_RF_leg-30)/50;
    pub_leg_position.publish(leg_position);
    ros::Duration(0.05).sleep();
    }
  stop.data = false;
  pub_stop_signal.publish(stop);
  }else{
    stop.data = true;
    pub_stop_signal.publish(stop);
    leg_position.data[2] = z_offset_LF_leg;
    leg_position.data[5] = z_offset_LR_leg;
    leg_position.data[8] = z_offset_RR_leg;
    leg_position.data[11] = z_offset_RF_leg;
    for(int i=0;i<50;i++){
      leg_position.data[2] -= (z_offset_LF_leg-30)/50;
      leg_position.data[5] -= (z_offset_LR_leg-30)/50;
      leg_position.data[8] -= (z_offset_RR_leg-30)/50;
      leg_position.data[11] -= (z_offset_RF_leg-30)/50;
      pub_leg_position.publish(leg_position);
      ros::Duration(0.05).sleep();
    }
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "standing_motion");
  Standing_Motion standing_motion;
  ros::spin();
  return 0;
}