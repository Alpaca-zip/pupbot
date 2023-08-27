FROM tiryoh/ros-desktop-vnc:noetic
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    python3-pip \
    ros-noetic-serial \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*
RUN mkdir -p ~/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws/src ; catkin_init_workspace" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash ; cd ~/catkin_ws && catkin build" && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN cd ~/catkin_ws && \
    wstool init src && \
    cd src && \
    git clone -b noetic-devel https://github.com/Alpaca-zip/pupbot.git && \
    wstool merge pupbot/pupbot.rosinstall && \
    wstool update && \
    cd ~/catkin_ws && \
    rosdep install -r -y -i --from-paths . && \
    catkin build
