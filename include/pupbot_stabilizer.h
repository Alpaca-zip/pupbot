#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define Z_OFFSET 120

class Pupbot_stabilizer{
  public:
  Pupbot_stabilizer();
  void controlLoop();

  private:
  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg_z_offset;
  ros::Publisher pub_leftback_leg_z_offset;
  ros::Publisher pub_rightfront_leg_z_offset;
  ros::Publisher pub_rightback_leg_z_offset;
  std_msgs::Float64 leftfront_leg_z_offset, leftback_leg_z_offset, rightfront_leg_z_offset, rightback_leg_z_offset;

  void init();
};