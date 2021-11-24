#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define X_OFFSET 61
#define Y_OFFSET 94
#define LEFTFRONTLEG_Z_OFFSET 113
#define LEFTBACKLEG_Z_OFFSET 120
#define RIGHTFRONTLEG_Z_OFFSET 115
#define RIGHTBACKLEG_Z_OFFSET 118

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
  double Kp;
  double Ki;
  double Kd;

  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg_z_offset;
  ros::Publisher pub_leftback_leg_z_offset;
  ros::Publisher pub_rightfront_leg_z_offset;
  ros::Publisher pub_rightback_leg_z_offset;
  ros::Subscriber roll_sub;
  ros::Subscriber pitch_sub;
  std_msgs::Float64 leftfront_leg_z_offset, leftback_leg_z_offset, rightfront_leg_z_offset, rightback_leg_z_offset;

  void init();
  void roll_callback(const std_msgs::Float64& roll);
  void pitch_callback(const std_msgs::Float64& pitch);
};
