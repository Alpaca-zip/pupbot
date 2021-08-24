#include "pupbot_controller_ds4.h"

/* ++++++++++++++++++++++++++++++++++
      Pupbot_Controller_ds4 class
++++++++++++++++++++++++++++++++++ */
Pupbot_Controller_ds4::Pupbot_Controller_ds4(){
  init();
  printf("Control PupBot with DUALSHOCK 4\n");
  printf("○ : Startup/Shutdown\n");
  printf("Left stick : Move forward/backward\n");
  printf("Right stick : Turn left/right\n");
}

void Pupbot_Controller_ds4::init(){
  key_control_pub1 = nh.advertise<std_msgs::Float64>("key_control1", 10);
  key_control_pub2 = nh.advertise<std_msgs::Bool>("key_control2", 10);
  key_control_pub3 = nh.advertise<std_msgs::Float64>("key_control3", 10);
  joy_sub = nh.subscribe("/joy", 10, &Pupbot_Controller_ds4::joy_callback, this);
  direction_x.data = 0.0;
  turn.data = 0.0;
  startup_shutdown.data = false;
}

void Pupbot_Controller_ds4::joy_callback(const sensor_msgs::Joy& joy_msg){
  direction_x.data = joy_msg.axes[1];
  key_control_pub1.publish(direction_x);
  std::cout << "x:" << direction_x.data << std::endl;

  turn.data = joy_msg.axes[2];
  key_control_pub3.publish(turn);
  std::cout << "turn:" << turn.data << std::endl;

  startup_shutdown.data = joy_msg.buttons[13];
  if(startup_shutdown.data == 1){
    key_control_pub2.publish(startup_shutdown);
    std::cout << "Power" << std::endl;
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc, argv, "pupbot_controller_ds4");
  Pupbot_Controller_ds4 PupBot;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}