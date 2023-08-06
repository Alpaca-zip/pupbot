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

keyControl::keyControl()
{
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
  ros::Duration(2.0).sleep();

  std::cout << "\033[32mQ : Stand up/lie down\033[0m" << std::endl;
  std::cout << "\033[32mW : Increases the value of direction in the x axis (+0.25)\033[0m"
            << std::endl;
  std::cout << "\033[32mA : Increases the value of turn (-0.25)\033[0m" << std::endl;
  std::cout << "\033[32mS : Decreases the value of direction in the x axis (-0.25)\033[0m"
            << std::endl;
  std::cout << "\033[32mD : Decreases the value of turn (+0.25)\033[0m" << std::endl;
  std::cout << "\033[32mX : Posture control on/off\033[0m" << std::endl;
}

int keyControl::getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

void keyControl::controlLoop()
{
  int c = getch();
  if (c == 'w') {
    if (_direction_x.data < 1.25) {
      _direction_x.data += 0.25;
    }
    std::cout << "\033[32m ==> x:\033[0m" << _direction_x.data << std::endl;
    _trot_foward_motion_pub.publish(_direction_x);
  } else if (c == 'a') {
    if (_turn.data > -1.0) {
      _turn.data -= 0.25;
    }
    std::cout << "\033[32m ==> turn:\033[0m" << _turn.data << std::endl;
    _trot_turn_motion_pub.publish(_turn);
  } else if (c == 's') {
    if (_direction_x.data > -1.25) {
      _direction_x.data -= 0.25;
    }
    std::cout << "\033[32m ==> x:\033[0m" << _direction_x.data << std::endl;
    _trot_foward_motion_pub.publish(_direction_x);
  } else if (c == 'd') {
    if (_turn.data < 1.0) {
      _turn.data += 0.25;
    }
    std::cout << "\033[32m ==> turn:\033[0m" << _turn.data << std::endl;
    _trot_turn_motion_pub.publish(_turn);
  } else if (c == 'q') {
    if (_stand.data) {
      _stand.data = false;
      std::cout << "\033[32m ==> lie down\033[0m" << std::endl;
    } else {
      _stand.data = true;
      std::cout << "\033[32m ==> Stand up\033[0m" << std::endl;
    }
    _direction_x.data = 0.0;
    _turn.data = 0.0;
    _trot_foward_motion_pub.publish(_direction_x);
    _trot_turn_motion_pub.publish(_turn);
    _standing_motion_pub.publish(_stand);
  } else if (c == 'x') {
    if (_posture_control.data) {
      _posture_control.data = false;
      std::cout << "\033[32m ==> Posture control disable\033[0m" << std::endl;
    } else {
      _posture_control.data = true;
      std::cout << "\033[32m ==> Posture control enable\033[0m" << std::endl;
    }
    _posture_control_pub.publish(_posture_control);
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "key_control");
  keyControl KC;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    KC.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
