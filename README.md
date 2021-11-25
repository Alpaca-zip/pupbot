# PupBot

<img src="https://github.com/Alpaca-zip/PupBot/blob/main/PupBot.png">

[![](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg)](https://github.com/Alpaca-zip/PupBot)

[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/PupBot)

Open source ROS package for quadruped robot PupBot.

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

## Control the PupBot with DualShock 4

```
roslaunch pupbot move_pupbot_ds4.launch
```
○ : Startup/Shutdown  
△ : Crawl gait/Trot gait  
Left stick : Move forward/backward  
Right stick : Turn left/right  

## Control the PupBot with laptop

```
roslaunch pupbot move_pupbot.launch
```

Q : Startup/Shutdown  
E : Crawl gait/Trot gait  
W : Increases the value of direction in the x axis (+0.25)  
A : Increases the value of turn (+0.25)  
S : Decreases the value of direction in the x axis (-0.25)  
D : Decreases the value of turn (-0.25)  

## Launch Gazebo

```
roslaunch pupbot pupbot_gazebo.launch
```

## Launch Rviz

```
roslaunch pupbot pupbot_rviz.launch
```
