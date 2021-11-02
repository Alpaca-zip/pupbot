#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define X_OFFSET 61
#define Y_OFFSET 116
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
