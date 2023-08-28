# PupBot [![ROS-noetic Build Check](https://github.com/Alpaca-zip/pupbot/actions/workflows/ros1-build-check-bot.yml/badge.svg?event=pull_request)](https://github.com/Alpaca-zip/pupbot/actions/workflows/ros1-build-check-bot.yml) [![Docker Build Check](https://github.com/Alpaca-zip/pupbot/actions/workflows/docker-build-check-bot.yml/badge.svg?event=pull_request)](https://github.com/Alpaca-zip/pupbot/actions/workflows/docker-build-check-bot.yml)
ROS package for quadruped robot PupBot.

[![dockeri.co](https://dockerico.blankenship.io/image/alpacazip/pupbot)](https://hub.docker.com/r/alpacazip/pupbot)

~~https://github.com/Alpaca-zip/docker-pupbot~~ is no longer supported!  
Check out DockerHub from the banner above.

<img src="https://github.com/Alpaca-zip/pupbot/assets/84959376/aeeeb110-bc89-4cfb-9514-b253e039affc" width="330px"> 　<img src="https://github.com/Alpaca-zip/pupbot/assets/84959376/9bd9c814-8c15-4010-a0dd-d9faf234b706" width="400px">

## Basic Components
- Dynamixel AX-12A (×12)
- OpenCR1.0 (×1)
- Raspberry Pi 4 Model B (×1)
- LIPO Battery 11.1V 1,800mAh (×1)
- 3P Extension PCB (×1)

## Posture control with PupBot
The posture control feature provides a stable gait on sloping terrain. This algorithm plays an important role in stabilizing the robot body during gait by dynamically controlling the toe position based on the filtered roll and pitch angles. 

![posture_control](https://user-images.githubusercontent.com/84959376/191177606-0fdff183-3349-40da-a78a-2da9e3d32d73.gif)
<img src="https://user-images.githubusercontent.com/84959376/191180942-1104cf41-3f2a-4d45-b8f5-ec9582013b9b.jpg" width="800px">

## Usage
### 1. Setup ROS package
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Alpaca-zip/pupbot.git
$ wstool merge pupbot/pupbot.rosinstall
$ wstool update
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build
```

### 2. Control the PupBot with simulation
```
$ roslaunch pupbot pupbot_simulation.launch debug:=true
```

### 3. Extra information
It is helpful to see `pupbot_bringup.launch` to get an understanding of how the robot works on the actual machine.
