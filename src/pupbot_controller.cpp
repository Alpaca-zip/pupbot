#include "pupbot_controller.h"

/* ++++++++++++++++++++++++++++++++++
      Pupbot_Controller class
++++++++++++++++++++++++++++++++++ */
Pupbot_Controller::Pupbot_Controller(){
  init();
  printf("q : Startup,Shutdown\n");
  printf("w : Increases the value of direction in the x axis (+0.25)\n");
  printf("a : Increases the value of turn (+0.25)\n");
  printf("s : Decreases the value of direction in the x axis (-0.25)\n");
  printf("d : Decreases the value of turn (-0.25)\n");
}

void Pupbot_Controller::init(){
  key_control_pub1 = nh.advertise<std_msgs::Float64>("key_control1", 10);
  key_control_pub2 = nh.advertise<std_msgs::Bool>("key_control2", 10);
  key_control_pub3 = nh.advertise<std_msgs::Float64>("key_control3", 10);
  direction_x.data = 0.0;
  turn.data = 0.0;
  startup_shutdown.data = false;
}

int Pupbot_Controller::getch(){
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); 
  newt = oldt;
  newt.c_lflag &= ~(ICANON);       
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar(); 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void Pupbot_Controller::controlLoop(){
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
    key_control_pub2.publish( startup_shutdown);
    startup_shutdown.data = false;
  }
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc, argv, "pupbot_controller");
  Pupbot_Controller PupBot;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    PupBot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}