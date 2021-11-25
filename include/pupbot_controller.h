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
  ros::Publisher key_control_pub4;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//This has been deprecated, and could be removed in a future release.
  ros::Publisher key_control_pub_Kp;
  ros::Publisher key_control_pub_Ki;
  ros::Publisher key_control_pub_Kd;
  std_msgs::Float64 Kp;
  std_msgs::Float64 Ki;
  std_msgs::Float64 Kd;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  std_msgs::Float64 direction_x;
  std_msgs::Float64 turn;
  std_msgs::Bool startup_shutdown;
  std_msgs::Bool gait_state;

  void init();
  int getch();
};