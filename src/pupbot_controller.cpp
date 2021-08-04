#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <termios.h>

int getch() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt); 
  newt = oldt;
  newt.c_lflag &= ~(ICANON);       
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);

  int c = getchar(); 

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pupbot_controller");
  ros::NodeHandle nh;
  ros::Publisher key_control_pub1 = nh.advertise<std_msgs::Float64>("key_control1", 10);
  ros::Publisher key_control_pub2 = nh.advertise<std_msgs::Bool>("key_control2", 10);
  ros::Publisher key_control_pub3 = nh.advertise<std_msgs::Float64>("key_control3", 10);
  ros::Rate loop_rate(10);
  std_msgs::Float64 direction_x,turn;
  std_msgs::Bool startup_shutdown;
  direction_x.data=0.0;
  turn.data=0.0;
  startup_shutdown.data=false;

  std::cout << "w : Increases the value of direction in the x axis (+0.25)" << std::endl;
  std::cout << "a : Increases the value of turn (+0.25)" << std::endl;
  std::cout << "s : Decreases the value of direction in the x axis (-0.25)" << std::endl;
  std::cout << "d : Decreases the value of turn (-0.25)" << std::endl;
  std::cout << "q : Startup,Shutdown" << std::endl;

  while (ros::ok()) {
    int c = getch();
    if (c == 'w'){
      direction_x.data += 0.25;
      std::cout << "x:" << direction_x.data << std::endl;
      key_control_pub1.publish(direction_x);
    }else if (c == 'a') {
      turn.data += 0.25;
      std::cout << "turn:" << turn.data << std::endl;
      key_control_pub3.publish(turn);
    }else if (c == 's') {
      direction_x.data -= 0.25;
      std::cout << "x:" << direction_x.data << std::endl;
      key_control_pub1.publish(direction_x);
    }else if (c == 'd') {
      turn.data -= 0.25;
      std::cout << "turn:" << turn.data << std::endl;
      key_control_pub3.publish(turn);
    }else if (c == 'q') {
      startup_shutdown.data=true;
      std::cout << "Power" << std::endl;
      key_control_pub2.publish(startup_shutdown);
      startup_shutdown.data=false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}