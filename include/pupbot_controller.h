#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "termios.h"

class Pupbot_Controller{
  public:
  Pupbot_Controller();
  void controlLoop();

  private:
  ros::NodeHandle nh;
  ros::Publisher key_control_pub1;
  ros::Publisher key_control_pub2;
  ros::Publisher key_control_pub3;
  std_msgs::Float64 direction_x;
  std_msgs::Float64 turn;
  std_msgs::Bool startup_shutdown;
  
  void init();
  int getch();
};