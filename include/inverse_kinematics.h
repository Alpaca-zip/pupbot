//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Saturday, March 5, 2022

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"

#define BONE_LENGTH 83.0
#define LENGTH 21.0

class Inverse_Kinematics{
  public:
  Inverse_Kinematics();
  
  private:
  int l;
  double x, y, z, a0, a1, b0;
  double angle1, angle2, angle3;
  double bone_length;
  double target_leg_shoulder_joint, target_L_leg_upper_joint, target_L_leg_lower_joint, target_R_leg_upper_joint, target_R_leg_lower_joint;
  
  ros::NodeHandle nh;
  ros::Publisher pub_LF_leg;
  ros::Publisher pub_LR_leg;
  ros::Publisher pub_RF_leg;
  ros::Publisher pub_RR_leg;
  ros::Subscriber sub_leg_position;
  trajectory_msgs::JointTrajectory LF_leg, LR_leg, RF_leg, RR_leg;

  void init();
  void inverse_kinematics_callback(const std_msgs::Float64MultiArray& leg_position);
};
