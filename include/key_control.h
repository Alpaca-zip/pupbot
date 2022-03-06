//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Thursday, March 3, 2022

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "termios.h"

class Key_Control{
  public:
  Key_Control();
  void controlLoop();

  private:
  ros::NodeHandle nh;
  ros::Publisher trot_foward_motion_pub;
  ros::Publisher trot_turn_motion_pub;
  ros::Publisher standing_motion_pub;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  ros::Publisher key_control_pub_Kp;
  ros::Publisher key_control_pub_Ki;
  ros::Publisher key_control_pub_Kd;
  ros::Publisher key_control_PID;
  std_msgs::Float64 Kp;
  std_msgs::Float64 Ki;
  std_msgs::Float64 Kd;
  std_msgs::Bool PID;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  std_msgs::Float64 direction_x;
  std_msgs::Float64 turn;
  std_msgs::Bool stand;

  void init();
  int getch();
};