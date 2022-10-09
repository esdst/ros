# ROS

- ROS Noetic

# Requirement

- Ubuntu 20.04

# Installation

1. Setup the sources.list

```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'```.

2. Set up keys

```sudo apt install curl```
```curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -```

3. Update

```sudo apt update```

4. Install ROS `noetic`version

```sudo apt install```
```ros-noetic-desktop-full```






