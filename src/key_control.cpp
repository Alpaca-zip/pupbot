//     _     _                                         _
//    / \   | | _ __    __ _   ___   __ _         ____(_) _ __
//   / _ \  | || '_ \  / _` | / __| / _` | _____ |_  /| || '_ \
//  / ___ \ | || |_) || (_| || (__ | (_| ||_____| / / | || |_) |
// /_/   \_\|_|| .__/  \__,_| \___| \__,_|       /___||_|| .__/
//             |_|                                       |_|
//
// Last updated: Thursday, March 3, 2022

#include "key_control.h"

/* ++++++++++++++++++++++++++++++++++
      Pupbot_Controller class
++++++++++++++++++++++++++++++++++ */
Key_Control::Key_Control(){
  init();
  std::cout << "Q : Stand up/lie down" << std::endl;
  std::cout << "W : Increases the value of direction in the x axis (+0.25)" << std::endl;
  std::cout << "A : Increases the value of turn (+0.25)" << std::endl;
  std::cout << "S : Decreases the value of direction in the x axis (-0.25)" << std::endl;
  std::cout << "D : Decreases the value of turn (-0.25)" << std::endl;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  std::cout << "F : Increases the value of P" << std::endl;
  std::cout << "G : Decreases the value of P" << std::endl;
  std::cout << "H : Increases the value of I" << std::endl;
  std::cout << "J : Decreases the value of I" << std::endl;
  std::cout << "K : Increases the value of D" << std::endl;
  std::cout << "L : Decreases the value of D" << std::endl;
  std::cout << "X : PID control on/off" << std::endl;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

void Key_Control::init(){
  trot_foward_motion_pub = nh.advertise<std_msgs::Float64>("/trot_foward_motion", 10);
  trot_turn_motion_pub = nh.advertise<std_msgs::Float64>("/trot_turn_motion", 10);
  standing_motion_pub = nh.advertise<std_msgs::Bool>("/standing_motion", 10);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.
  key_control_pub_Kp = nh.advertise<std_msgs::Float64>("key_control_Kp", 10);
  key_control_pub_Ki = nh.advertise<std_msgs::Float64>("key_control_Ki", 10);
  key_control_pub_Kd = nh.advertise<std_msgs::Float64>("key_control_Kd", 10);
  key_control_PID = nh.advertise<std_msgs::Bool>("PID_on_off", 10);
  Kp.data = Ki.data = Kd.data = 0.0;
  PID.data = false;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  direction_x.data = 0.0;
  turn.data = 0.0;
  stand.data = false;

/* ++++++++++++++++++++++++++++++++++
       Pose initialization
++++++++++++++++++++++++++++++++++ */
  std::cout << "+++++++++++++++" << std::endl;
  std::cout << "Initializing..." << std::endl;
  std::cout << "+++++++++++++++" << std::endl;
  ros::Duration(2.0).sleep();
}

int Key_Control::getch(){
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); 
  newt = oldt;
  newt.c_lflag &= ~(ICANON);       
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar(); 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void Key_Control::controlLoop(){
  int c = getch();
  if(c == 'w'){
    if(direction_x.data < 1.25){
      direction_x.data += 0.25;
    }
    std::cout << " ==> x:" << direction_x.data << std::endl;
    trot_foward_motion_pub.publish(direction_x);
  }else if(c == 'a'){
    if(turn.data < 1.0){
      turn.data += 0.25;
    }
    std::cout << " ==> turn:" << turn.data << std::endl;
    trot_turn_motion_pub.publish(turn);
  }else if(c == 's'){
    if(direction_x.data > -1.25){
      direction_x.data -= 0.25;
    }
    std::cout << " ==> x:" << direction_x.data << std::endl;
    trot_foward_motion_pub.publish(direction_x);
  }else if(c == 'd'){
    if(turn.data > -1.0){
      turn.data -= 0.25;
    }
    std::cout << " ==> turn:" << turn.data << std::endl;
    trot_turn_motion_pub.publish(turn);
  }else if(c == 'q'){
    if(stand.data){
      stand.data = false;
      std::cout << " ==> lie down" << std::endl;
    }else{
      stand.data = true;
      std::cout << " ==> Stand up" << std::endl;
    }
    standing_motion_pub.publish(stand);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//PID control section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//This has been deprecated, and could be removed in a future release.  
  }else if(c == 'f'){
    Kp.data += 0.05;
    std::cout << " ==> P:" << Kp.data << std::endl;
    key_control_pub_Kp.publish(Kp);
  }else if(c == 'g'){
    Kp.data -= 0.05;
    std::cout << " ==> P:" << Kp.data << std::endl;
    key_control_pub_Kp.publish(Kp);
  }else if(c == 'h'){
    Ki.data += 0.05;
    std::cout << " ==> I:" << Ki.data << std::endl;
    key_control_pub_Ki.publish(Ki);
  }else if(c == 'j'){
    Ki.data -= 0.05;
    std::cout << " ==> I:" << Ki.data << std::endl;
    key_control_pub_Ki.publish(Ki);
  }else if(c == 'k'){
    Kd.data += 0.05;
    std::cout << " ==> D:" << Kd.data << std::endl;
    key_control_pub_Kd.publish(Kd);
  }else if(c == 'l'){
    Kd.data -= 0.05;
    std::cout << " ==> D:" << Kd.data << std::endl;
    key_control_pub_Ki.publish(Kd);
  }else if(c == 'x'){
    PID.data = true;
    std::cout << " ==> PID" << std::endl;
    key_control_PID.publish(PID);
    PID.data = false;
  }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc, argv, "key_control");
  Key_Control key_control;

  ros::Rate loop_rate(10);

  while (ros::ok()){
    key_control.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}