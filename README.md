# pupbot_ver1

<img src="https://github.com/Alpaca-zip/pupbot_ver1/blob/main/pupbot_ver1.png">

[![](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg)](https://github.com/Alpaca-zip/pupbot_ver1)
[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/pupbot_ver1)

Open source ROS package for quadruped robot PupBot Ver1.

Dockerfiles:https://github.com/Alpaca-zip/docker-pupbot_ver1

## Components
- Dynamixel AX-12A (×12)
- OpenCR1.0 (×1)
- Raspberry Pi 3 Model B+ (×1)
- LIPO Battery 11.1V 1,800mAh (×1)
- TB3 Waffle Plate (×8)
- Plate Support M3×45mm (×4)
- FP04-F2 (×4)
- FP04-F3 (×16)
- 3P Extension PCB (×1)
- Raspberry Pi Power Cable (×1)
- Li-Po Battery Extension Cable (×1)
- DYNAMIXEL Robot cable (×13)
- USB Cable (×1)

## Control the PupBot with laptop

```
roslaunch pupbot_ver1 move_pupbot.launch
```

Q : Startup/Shutdown  
W : Increases the value of direction in the x axis (+0.25)  
A : Increases the value of turn (+0.25)  
S : Decreases the value of direction in the x axis (-0.25)  
D : Decreases the value of turn (-0.25)  
X : PID control on/off  
F : Increases the value of P  
G : Decreases the value of P  
H : Increases the value of I  
J : Decreases the value of I  
K : Increases the value of D  
L : Decreases the value of D  

## Launch Gazebo

```
roslaunch pupbot_ver1 pupbot_gazebo.launch
```

## Launch Rviz

```
roslaunch pupbot_ver1 pupbot_rviz.launch
```

