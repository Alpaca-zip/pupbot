//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Saturday, April 16, 2022

#include "inverse_kinematics.h"

/* ++++++++++++++++++++++++++++++++++
      Inverse_Kinematics class
++++++++++++++++++++++++++++++++++ */
Inverse_Kinematics::Inverse_Kinematics(){
  init();
}

void Inverse_Kinematics::init(){
  bone_length = BONE_LENGTH;
  pub_LF_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftfront_leg_controller/command", 10);
  pub_LR_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftback_leg_controller/command", 10);
  pub_RF_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightfront_leg_controller/command", 10);
  pub_RR_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightback_leg_controller/command", 10);
  sub_leg_position = nh.subscribe("/leg_position", 10, &Inverse_Kinematics::inverse_kinematics_callback, this);
  
  LF_leg.joint_names.resize(3);
  LF_leg.points.resize(1);
  LF_leg.points[0].positions.resize(3);
  LF_leg.joint_names[0] = "leftfront_leg_shoulder_joint";
  LF_leg.joint_names[1] = "leftfront_leg_upper_joint";
  LF_leg.joint_names[2] = "leftfront_leg_lower_joint";
  LR_leg.joint_names.resize(3);
  LR_leg.points.resize(1);
  LR_leg.points[0].positions.resize(3);
  LR_leg.joint_names[0] = "leftback_leg_shoulder_joint";
  LR_leg.joint_names[1] = "leftback_leg_upper_joint";
  LR_leg.joint_names[2] = "leftback_leg_lower_joint";
  RR_leg.joint_names.resize(3);
  RR_leg.points.resize(1);
  RR_leg.points[0].positions.resize(3);
  RR_leg.joint_names[0] = "rightback_leg_shoulder_joint";
  RR_leg.joint_names[1] = "rightback_leg_upper_joint";
  RR_leg.joint_names[2] = "rightback_leg_lower_joint";
  RF_leg.joint_names.resize(3);
  RF_leg.points.resize(1);
  RF_leg.points[0].positions.resize(3);
  RF_leg.joint_names[0] = "rightfront_leg_shoulder_joint";
  RF_leg.joint_names[1] = "rightfront_leg_upper_joint";
  RF_leg.joint_names[2] = "rightfront_leg_lower_joint";
}

void Inverse_Kinematics::inverse_kinematics_callback(const std_msgs::Float64MultiArray& leg_position){
  for(l=0; l<4; l++){
    x = leg_position.data[3*l];
    y = leg_position.data[3*l+1];
    z = leg_position.data[3*l+2];

    a0 = (y*y+z*z-LENGTH*LENGTH+x*x-2*bone_length*bone_length)/(2*bone_length*bone_length);

    if(l == 0){
	  angle1 = -atan2(-z, y)-atan2(sqrt(y*y+z*z-LENGTH*LENGTH), LENGTH);
      angle3 = atan2(sqrt(1-a0*a0), a0);
      angle2 = atan2(x, sqrt(y*y+z*z-LENGTH*LENGTH))-atan2(bone_length*sin(angle3), bone_length*(1+cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      LF_leg.points[0].positions[0] = target_leg_shoulder_joint;
      LF_leg.points[0].positions[1] = target_L_leg_upper_joint;
      LF_leg.points[0].positions[2] = target_L_leg_lower_joint;
      LF_leg.header.stamp = ros::Time::now();
      LF_leg.points[0].time_from_start = ros::Duration(0.008);
      pub_LF_leg.publish(LF_leg);
    }else if(l == 1){
	  angle1 = -atan2(-z, y)-atan2(sqrt(y*y+z*z-LENGTH*LENGTH), LENGTH);
      angle3 = atan2(sqrt(1-a0*a0), a0);
      angle2 = atan2(x, sqrt(y*y+z*z-LENGTH*LENGTH))-atan2(bone_length*sin(angle3), bone_length*(1+cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      LR_leg.points[0].positions[0] = target_leg_shoulder_joint;
      LR_leg.points[0].positions[1] = target_L_leg_upper_joint;
      LR_leg.points[0].positions[2] = target_L_leg_lower_joint;
      LR_leg.header.stamp = ros::Time::now();
      LR_leg.points[0].time_from_start = ros::Duration(0.008);
      pub_LR_leg.publish(LR_leg);
    }else if(l == 2){
	  angle1 = -atan2(-z, y)-atan2(sqrt(y*y+z*z-LENGTH*LENGTH), -LENGTH);
      angle3 = atan2(sqrt(1-a0*a0), a0);
      angle2 = atan2(x, sqrt(y*y+z*z-LENGTH*LENGTH))-atan2(bone_length*sin(angle3), bone_length*(1+cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      RR_leg.points[0].positions[0] = target_leg_shoulder_joint;
      RR_leg.points[0].positions[1] = target_R_leg_upper_joint;
      RR_leg.points[0].positions[2] = target_R_leg_lower_joint;
      RR_leg.header.stamp = ros::Time::now();
      RR_leg.points[0].time_from_start = ros::Duration(0.008);
      pub_RR_leg.publish(RR_leg);
    }else if(l == 3){
	  angle1 = -atan2(-z, y)-atan2(sqrt(y*y+z*z-LENGTH*LENGTH), -LENGTH);
      angle3 = atan2(sqrt(1-a0*a0), a0);
      angle2 = atan2(x, sqrt(y*y+z*z-LENGTH*LENGTH))-atan2(bone_length*sin(angle3), bone_length*(1+cos(angle3)));
      target_leg_shoulder_joint = angle1;
      target_L_leg_upper_joint = -angle2;
      target_L_leg_lower_joint = angle3;
      target_R_leg_upper_joint = angle2;
      target_R_leg_lower_joint = -angle3;
      RF_leg.points[0].positions[0] = target_leg_shoulder_joint;
      RF_leg.points[0].positions[1] = target_R_leg_upper_joint;
      RF_leg.points[0].positions[2] = target_R_leg_lower_joint;
      RF_leg.header.stamp = ros::Time::now();
      RF_leg.points[0].time_from_start = ros::Duration(0.008);
      pub_RF_leg.publish(RF_leg);
    }
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "inverse_kinematics");
  Inverse_Kinematics inverse_kinematics;
  ros::spin();
  return 0;
}