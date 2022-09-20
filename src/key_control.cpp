/**
 * Copyright (C) 2022  Alpaca-zip
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "key_control.h"

/* ++++++++++++++++++++++++++++++++++
         Key_Control class
++++++++++++++++++++++++++++++++++ */
Key_Control::Key_Control(){
  init();
  std::cout << "Q : Stand up/lie down" << std::endl;
  std::cout << "W : Increases the value of direction in the x axis (+0.25)" << std::endl;
  std::cout << "A : Increases the value of turn (-0.25)" << std::endl;
  std::cout << "S : Decreases the value of direction in the x axis (-0.25)" << std::endl;
  std::cout << "D : Decreases the value of turn (+0.25)" << std::endl;
  std::cout << "X : Posture control on/off" << std::endl;
}

void Key_Control::init(){
  trot_foward_motion_pub = nh.advertise<std_msgs::Float64>("/trot_foward_motion", 10);
  trot_turn_motion_pub = nh.advertise<std_msgs::Float64>("/trot_turn_motion", 10);
  standing_motion_pub = nh.advertise<std_msgs::Bool>("/standing_motion", 10);
  posture_control_pub = nh.advertise<std_msgs::Bool>("/posture_control", 10);
  posture_control.data = false;
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
    if(turn.data > -1.0){
      turn.data -= 0.25;
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
    if(turn.data < 1.0){
      turn.data += 0.25;
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
  }else if(c == 'x'){
    if(posture_control.data){
      posture_control.data = false;
      std::cout << " ==> Posture control disable" << std::endl;
    }else{
      posture_control.data = true;
      std::cout << " ==> Posture control enable" << std::endl;
    }
    posture_control_pub.publish(posture_control);
  }
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
