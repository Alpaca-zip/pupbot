# PupBot

![pupbot](https://user-images.githubusercontent.com/84959376/191180426-5900f05b-1c27-4ff8-8bd7-21559404e883.png)

[![](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg)](https://github.com/Alpaca-zip/pupbot)
[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/pupbot)

Open source ROS package for quadruped robot PupBot. 

Dockerfiles:https://github.com/Alpaca-zip/docker-pupbot

## Basic Components
- Dynamixel AX-12A (×12)
- OpenCR1.0 (×1)
- Raspberry Pi 4 Model B (×1)
- LIPO Battery 11.1V 1,800mAh (×1)
- 3P Extension PCB (×1)

## Posture control with PupBot
The posture control feature provides a stable gait on sloping terrain. This algorithm plays an important role in stabilizing the robot body during gait by dynamically controlling the toe position based on the filtered roll and pitch angles. 

![posture_control](https://user-images.githubusercontent.com/84959376/191177606-0fdff183-3349-40da-a78a-2da9e3d32d73.gif)
![FSx-DpyVIAAEWa5](https://user-images.githubusercontent.com/84959376/191180942-1104cf41-3f2a-4d45-b8f5-ec9582013b9b.jpg)

## Control the PupBot with laptop(without gazebo)
### 1 From your laptop PC, open the terminal and connect to the Raspberry Pi with its IP address.
Before starting, please install ROS on your Raspberry Pi.
```
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
```
### 2 start PupBot applications.
```
$ roslaunch pupbot move_pupbot.launch
```

Q : Startup/Shutdown  
W : Increases the value of direction in the x axis (+0.25)  
A : Increases the value of turn (-0.25)  
S : Decreases the value of direction in the x axis (-0.25)  
D : Decreases the value of turn (+0.25)  
X : Posture control on/off  

### Launch Gazebo(for simulation)

```
$ roslaunch pupbot pupbot_gazebo.launch
```

### Launch Rviz

```
$ roslaunch pupbot pupbot_rviz.launch
```

