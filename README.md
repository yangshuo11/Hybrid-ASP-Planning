# Hybrid-ASP-Planning

This repository presents the Hybrid-ASP planning approach that facilitates task planning with missing task-relevant information. It integrates
a deterministic task planner POPF for generating an initially valid task plan for task achievement and a logic-based Answer Set Planner to make observation plans. In the following, we present a brief tutorial for running the demonstrating example of indoor navigation by a Turtlebot3 robot in the Gazebo simulation environment. 

# Requirements
Ubuntu 16.04 + ROS Kinetic 

# Installation

1. Install essential ros packages for Turtlebot3 waffle simulation in Gazebo

The Turtlebot3 waffle robot is a wheeled robot that equips with camera and laser sensors, as well as the move base for moving around. We select this robot model to perform the indoor navigation task in the Gazebo simulator.

sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

A detailed description regarding the installation and configuration of Turtlebot3 simulation in Gazebo can be referred to https://www.ncnynl.com/turtlebot3.html

2. Install essential ros packages for ROSPlan framework

sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-kinetic-tf2-bullet

3. Essential libraries for ROSClingo 

sudo apt-get install gcc-4.8 g++-4.8
sudo apt-get install bison re2c scons

4. Install gringo python module(4.5.4) (https://sourceforge.net/projects/potassco/files/gringo/4.5.4/).
Download the file and unpack the archive, and do the following steps:

scons configure --build-dir=release
scons --build-dir=release pyclingo
add to ~/.bashrc the line - replacing ${GRINGO_PATH} with the path to the gringo-4.5.4 source folder:
PYTHONPATH=$PYTHONPATH:${GRINGO_PATH}/build/release/python/

# Running steps

1. launch the turtlebo3 gazebo controller
roslaunch rosplan_turtlebot3_demo turtlebot_main.launch
2. launch the ASP solver
rosrun rosoclingo-1.0 run-1.0.py --file ../mailbot/instances/graph_wp.lp --file ../mailbot/turtlebot3_asp_model.lp

The demonstration video of experimental details is as follows:

[![Experimental demo for Hybrid-ASP planning](https://res.cloudinary.com/marcomontalbano/image/upload/v1632130243/video_to_markdown/images/youtube--2WkFsKWoYWI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/2WkFsKWoYWI "Experimental demo for Hybrid-ASP planning")

Contact email: 1024809808@qq.com

