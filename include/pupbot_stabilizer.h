#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#define X_OFFSET 61
#define Y_OFFSET 94
#define LEFTFRONTLEG_Z_OFFSET 117
#define LEFTBACKLEG_Z_OFFSET 118
#define RIGHTFRONTLEG_Z_OFFSET 119
#define RIGHTBACKLEG_Z_OFFSET 119

class Pupbot_stabilizer{
  public:
  Pupbot_stabilizer();
  void controlLoop();

  private:
  double roll_data;
  double pitch_data;
  double M_leftfront_leg, M_leftback_leg, M_rightfront_leg, M_rightback_leg;
  double M1_leftfront_leg, M1_leftback_leg, M1_rightfront_leg, M1_rightback_leg;
  double e_leftfront_leg, e_leftback_leg, e_rightfront_leg, e_rightback_leg;
  double e1_leftfront_leg, e1_leftback_leg, e1_rightfront_leg, e1_rightback_leg;
  double e2_leftfront_leg, e2_leftback_leg, e2_rightfront_leg, e2_rightback_leg;
  double P;
  double I;
  double D;

  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg_z_offset;
  ros::Publisher pub_leftback_leg_z_offset;
  ros::Publisher pub_rightfront_leg_z_offset;
  ros::Publisher pub_rightback_leg_z_offset;
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
  std_msgs::Float64 leftfront_leg_z_offset, leftback_leg_z_offset, rightfront_leg_z_offset, rightback_leg_z_offset;

  void init();
  void roll_callback(const std_msgs::Float64& roll);
  void pitch_callback(const std_msgs::Float64& pitch);
};
