#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#define Z_OFFSET 120

class Pupbot_stabilizer{
  public:
  Pupbot_stabilizer();
  void controlLoop();

  private:
  int leftfront_leg_state, leftback_leg_state, rightfront_leg_state, rightback_leg_state;
  double average;

  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg_z_offset;
  ros::Publisher pub_leftback_leg_z_offset;
  ros::Publisher pub_rightfront_leg_z_offset;
  ros::Publisher pub_rightback_leg_z_offset;
  ros::Subscriber sub_dynamixel_state_leftfront_leg;
  ros::Subscriber sub_dynamixel_state_leftback_leg;
  ros::Subscriber sub_dynamixel_state_rightfront_leg;
  ros::Subscriber sub_dynamixel_state_rightback_leg;
  std_msgs::Float64 leftfront_leg_z_offset, leftback_leg_z_offset, rightfront_leg_z_offset, rightback_leg_z_offset;

  void init();
  void monitor_leftfront_leg_load_callback(const std_msgs::Int32& leftfront_leg_load);
  void monitor_leftback_leg_load_callback(const std_msgs::Int32& leftback_leg_load);
  void monitor_rightfront_leg_load_callback(const std_msgs::Int32& rightfront_leg_load);
  void monitor_rightback_leg_load_callback(const std_msgs::Int32& rightback_leg_load);
};