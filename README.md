# PupBot

<img src="https://github.com/Alpaca-zip/PupBot/blob/main/PupBot.png">

[![](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg)](https://github.com/Alpaca-zip/PupBot)

[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/PupBot)

ROS package for quadruped robot PupBot.

## Launch Gazebo

```
roslaunch pupbot pupbot_empty_world.launch
```

## Launch Walking Simulation

```
roslaunch pupbot move_pupbot.launch
```

q : Startup,Shutdown  
w : Increases the value of direction in the x axis (+0.25)  
a : Increases the value of turn (+0.25)  
s : Decreases the value of direction in the x axis (-0.25)  
d : Decreases the value of turn (-0.25)

## Launch Rviz

```
roslaunch pupbot pupbot_rviz.launch
```