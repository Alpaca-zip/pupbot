//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Friday, March 4, 2022

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"

#define X_OFFSET -10.0
#define LF_LEG_Z_OFFSET 120
#define LR_LEG_Z_OFFSET 120
#define RR_LEG_Z_OFFSET 120
#define RF_LEG_Z_OFFSET 120

class Standing_Motion{
  public:
  Standing_Motion();
  
  private:
  double z_offset_LF_leg, z_offset_LR_leg, z_offset_RR_leg, z_offset_RF_leg;
  bool stop_signal;
  
  ros::NodeHandle nh;
  ros::Publisher pub_leg_position;
  ros::Publisher pub_stop_signal;
  ros::Subscriber sub_standing_motion;
  std_msgs::Float64MultiArray leg_position;
  std_msgs::Bool stop;

  void init();
  void standing_motion_callback(const std_msgs::Bool& stand);
};