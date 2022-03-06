//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Saturday, March 5, 2022

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define X_OFFSET 61
#define Y_OFFSET 94
#define LF_LEG_Z_OFFSET 120
#define LR_LEG_Z_OFFSET 120
#define RR_LEG_Z_OFFSET 120
#define RF_LEG_Z_OFFSET 120
#define WIDTH 27

class Posture_Stabilization{
  public:
  Posture_Stabilization();
  void controlLoop();

  private:
  double roll_data;
  double pitch_data;
  double M_LF_leg, M_LR_leg, M_RR_leg, M_RF_leg;
  double M1_LF_leg, M1_LR_leg, M1_RR_leg, M1_RF_leg;
  double e_LF_leg, e_LR_leg, e_RR_leg, e_RF_leg;
  double e1_LF_leg, e1_LR_leg, e1_RR_leg, e1_RF_leg;
  double e2_LF_leg, e2_LR_leg, e2_RR_leg, e2_RF_leg;
  double P;
  double I;
  double D;
  double buff_roll[WIDTH], buff_pitch[WIDTH];
  double roll_sum, pitch_sum;
  int roll_cnt, pitch_cnt;

  ros::NodeHandle nh;
  ros::Publisher pub_stabilization_variable;
  ros::Publisher pub_roll_LPF;
  ros::Publisher pub_pitch_LPF;
  ros::Subscriber roll_sub;
  ros::Subscriber pitch_sub;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  ros::Subscriber key_control_sub_Kp;
  ros::Subscriber key_control_sub_Ki;
  ros::Subscriber key_control_sub_Kd;
  ros::Subscriber key_control_sub_PID;
  void Kp_callback(const std_msgs::Float64& Kp);
  void Ki_callback(const std_msgs::Float64& Ki);
  void Kd_callback(const std_msgs::Float64& Kd);
  void PID_callback(const std_msgs::Bool& PID);
  bool PID_on;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  std_msgs::Float64 roll_LPF, pitch_LPF;
  std_msgs::Float64MultiArray MV;

  void init();
  void roll_callback(const std_msgs::Float64& roll);
  void pitch_callback(const std_msgs::Float64& pitch);
};
