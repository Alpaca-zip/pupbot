/**
 * Copyright (C) 2021-2023  Alpaca-zip
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

keyControl::keyControl(){
  _trot_foward_motion_pub = _nh.advertise<std_msgs::Float64>("trot_foward_motion", 10);
  _trot_turn_motion_pub = _nh.advertise<std_msgs::Float64>("trot_turn_motion", 10);
  _standing_motion_pub = _nh.advertise<std_msgs::Bool>("standing_motion", 10);
  _posture_control_pub = _nh.advertise<std_msgs::Bool>("posture_control", 10);
  _posture_control.data = false;
  _direction_x.data = 0.0;
  _turn.data = 0.0;
  _stand.data = false;
  std::cout << "+++++++++++++++" << std::endl;
  std::cout << "Initializing..." << std::endl;
  std::cout << "+++++++++++++++" << std::endl;
  std::cout << "Q : Stand up/lie down" << std::endl;
  std::cout << "W : Increases the value of direction in the x axis (+0.25)" << std::endl;
  std::cout << "A : Increases the value of turn (-0.25)" << std::endl;
  std::cout << "S : Decreases the value of direction in the x axis (-0.25)" << std::endl;
  std::cout << "D : Decreases the value of turn (+0.25)" << std::endl;
  std::cout << "X : Posture control on/off" << std::endl;
  ros::Duration(2.0).sleep();
}

int keyControl::getch(){
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); 
  newt = oldt;
  newt.c_lflag &= ~(ICANON);       
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar(); 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void keyControl::controlLoop(){
  int c = getch();
  if(c == 'w'){
    if(_direction_x.data < 1.25){
      _direction_x.data += 0.25;
    }
    std::cout << " ==> x:" << _direction_x.data << std::endl;
    _trot_foward_motion_pub.publish(_direction_x);
  }else if(c == 'a'){
    if(_turn.data > -1.0){
      _turn.data -= 0.25;
    }
    std::cout << " ==> turn:" << _turn.data << std::endl;
    _trot_turn_motion_pub.publish(_turn);
  }else if(c == 's'){
    if(_direction_x.data > -1.25){
      _direction_x.data -= 0.25;
    }
    std::cout << " ==> x:" << _direction_x.data << std::endl;
    _trot_foward_motion_pub.publish(_direction_x);
  }else if(c == 'd'){
    if(_turn.data < 1.0){
      _turn.data += 0.25;
    }
    std::cout << " ==> turn:" << _turn.data << std::endl;
    _trot_turn_motion_pub.publish(_turn);
  }else if(c == 'q'){
    if(_stand.data){
      _stand.data = false;
      std::cout << " ==> lie down" << std::endl;
    }else{
      _stand.data = true;
      std::cout << " ==> Stand up" << std::endl;
    }
    _standing_motion_pub.publish(_stand);
  }else if(c == 'x'){
    if(_posture_control.data){
      _posture_control.data = false;
      std::cout << " ==> Posture control disable" << std::endl;
    }else{
      _posture_control.data = true;
      std::cout << " ==> Posture control enable" << std::endl;
    }
    _posture_control_pub.publish(_posture_control);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "key_control");
  keyControl KC;
  ros::Rate loop_rate(10);
  while (ros::ok()){
    KC.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
