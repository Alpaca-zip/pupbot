#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

class Pupbot_Controller_ds4{
  public:
  Pupbot_Controller_ds4();

  private:
  ros::NodeHandle nh;
  ros::Publisher key_control_pub1;
  ros::Publisher key_control_pub2;
  ros::Publisher key_control_pub3;
  ros::Subscriber joy_sub;
  std_msgs::Float64 direction_x;
  std_msgs::Float64 turn;
  std_msgs::Bool startup_shutdown;
  
  void init();
  void joy_callback(const sensor_msgs::Joy& joy_msg);
};