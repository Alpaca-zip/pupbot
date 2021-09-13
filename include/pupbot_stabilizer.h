#include "ros/ros.h"
#include "std_msgs/Int32.h"

class Pupbot_stabilizer{
  public:
  Pupbot_stabilizer();
  void controlLoop();

  private:
  int leftfront_leg_state, leftback_leg_state, rightfront_leg_state, rightback_leg_state;

  ros::NodeHandle nh;
  ros::Subscriber sub_dynamixel_state_leftfront_leg;
  ros::Subscriber sub_dynamixel_state_leftback_leg;
  ros::Subscriber sub_dynamixel_state_rightfront_leg;
  ros::Subscriber sub_dynamixel_state_rightback_leg;

  void init();
  void monitor_leftfront_leg_load_callback(const std_msgs::Int32& leftfront_leg_load);
  void monitor_leftback_leg_load_callback(const std_msgs::Int32& leftback_leg_load);
  void monitor_rightfront_leg_load_callback(const std_msgs::Int32& rightfront_leg_load);
  void monitor_rightback_leg_load_callback(const std_msgs::Int32& rightback_leg_load);
};