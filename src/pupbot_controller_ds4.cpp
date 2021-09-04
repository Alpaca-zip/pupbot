#include "pupbot_controller_ds4.h"

/* ++++++++++++++++++++++++++++++++++
      Pupbot_Controller_ds4 class
++++++++++++++++++++++++++++++++++ */
Pupbot_Controller_ds4::Pupbot_Controller_ds4(){
  init();
  std::cout << "Control PupBot with DUALSHOCK 4" << std::endl;
  std::cout << "○ : Startup/Shutdown" << std::endl;
  std::cout << "△ : Crawl gait/Trot gait" << std::endl;
  std::cout << "Left stick : Move forward/backward" << std::endl;
  std::cout << "Right stick : Turn left/right" << std::endl;
}

void Pupbot_Controller_ds4::init(){
  key_control_pub1 = nh.advertise<std_msgs::Float64>("key_control1", 10);
  key_control_pub2 = nh.advertise<std_msgs::Bool>("key_control2", 10);
  key_control_pub3 = nh.advertise<std_msgs::Float64>("key_control3", 10);
  key_control_pub4 = nh.advertise<std_msgs::Bool>("key_control4", 10);
  joy_sub = nh.subscribe("/joy", 10, &Pupbot_Controller_ds4::joy_callback, this);
  direction_x.data = 0.0;
  turn.data = 0.0;
  startup_shutdown.data = false;
  gait_state.data = false;
}

void Pupbot_Controller_ds4::joy_callback(const sensor_msgs::Joy& joy_msg){
  direction_x.data = joy_msg.axes[leftstick_up_and_down];
  key_control_pub1.publish(direction_x);
  std::cout << "x:" << direction_x.data << std::endl;

  turn.data = joy_msg.axes[rightstick_left_and_right];
  key_control_pub3.publish(turn);
  std::cout << "turn:" << turn.data << std::endl;

  if(joy_msg.buttons[circle_button] == 1){
    startup_shutdown.data=true;
    key_control_pub2.publish(startup_shutdown);
    startup_shutdown.data=false;
    std::cout << "Power" << std::endl;
  }

  if(joy_msg.buttons[triangle_button] == 1){
    if(gait_state.data == false){
      gait_state.data = true;
      std::cout << "Trot gait" << std::endl;
      key_control_pub4.publish(gait_state);
    }else{
      gait_state.data = false;
      std::cout << "Crawl gait" << std::endl;
      key_control_pub4.publish(gait_state);
    }
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
