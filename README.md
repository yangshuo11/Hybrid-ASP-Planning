# Hybrid-ASP-Planning

This repository presents the Hybrid-ASP planning approach that facilitates task planning with missing task-relevant information. It integrates
a deterministic task planner POPF for generating an initially valid task plan for task achievement and a logic-based Answer Set Planner to make observation plans. In the following, we present a brief tutorial for running the demonstrating example of indoor navigation by a Turtlebot3 robot in the Gazebo simulation environment. 

# Requirements
Ubuntu 16.04 + ROS Kinetic 

# Installation

## 1. Install essential ros packages for Turtlebot3 waffle simulation in Gazebo

The Turtlebot3 waffle robot is a wheeled robot that equips with camera and laser sensors, as well as the move base for moving around. We select this robot model to perform the indoor navigation task in the Gazebo simulator.

```bash
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

A detailed description regarding the installation and configuration of Turtlebot3 simulation in Gazebo can be referred to https://www.ncnynl.com/turtlebot3.html

## 2. Install essential ros packages for ROSPlan framework

The ROSPlan framework provides a generic method for task planning in a ROS system. We utilize the POPF planner that embedded as one PlannerInterface for task-achievement goal planning. It is thus essential to install the basic packages of ROSPlan.

The detailed description regarding the core library of ROSPlan framework can be found at https://github.com/KCL-Planning/ROSPlan, and the repository for ROSPlan demos of turtlebot3 navigation can be found at https://github.com/KCL-Planning/rosplan_demos. Both repository need to be installed for futher steps. 
Note: In our repository, we have modified the ActionInterface and the DispatchInterface for implementing the extra plan execution monitoring functions.

## 3. Install the essential libraries for ros(o)clingo 

The ros(o)clingo provides a generic way by which an ASP program may be used within the popular open-source Robot Operating System (ROS).To be more precise, the ros(o)cling package integrates the ASP solver clingo 4 into the ROS service and actionlib architecture. The ros(o)cling package is specialized for the purpose of (interactive) task planning for robots. By running as a ROS actionlib ROSoClingo provides an elegant way to processes observations and requests issued. ros(o)cling provides a number of solving modes specialized for different user scenarios and allows for an easy customation of the solving precedure.

### Setup necessary libraries

#### Install the basic compilers and parsers
```bash
sudo apt-get install gcc-4.8 <br>
sudo apt-get install g++-4.8
sudo apt-get install bison
sudo apt-get install re2c
sudo apt-get install scons
```
#### Download and build gringo.so python module
To get the gringo python module (4.5.4), please visit https://sourceforge.net/projects/potassco/files/gringo/4.5.4/ and download the gringo files. Unpack the archive, cd into the unpacked folder (gringo-4.5.4-source), and do the following:
```bash
scons configure --build-dir=release
```
Then configure the build/release.py file. Assuming you are using python2.7 and it is located in /usr/include/python2.7 modify the following variables in build/release.py:
```bash
CPPPATH = ['/usr/include/python2.7']
WITH_PYTHON = 'python2.7'
WITH_TBB = 'tbb'
```
Now run:
```bash
scons --build-dir=release pyclingo
```
Finally, add to ~/.bashrc the line - replacing ${GRINGO_PATH} with the path to the gringo-4.5.4 source folder:
```bash
PYTHONPATH=$PYTHONPATH:${GRINGO_PATH}/build/release/python/
```

#### Build ROSClingo
```bash
cd ${CATKIN_PATH}/src
wget https://www.cs.uni-potsdam.de/wv/projects/rosoclingo/rosclingo-2.0.tar.xz
tar -zxvf rosclingo.tar.gz
cd ..
catkin_make
```
#### Install ROSoClingo

After ROSClingo is installed correctly download and install ROSoClingo - replacing ${CATKIN_PATH} with the path for your catkin workspace:
```bash
cd ${CATKIN_PATH}/src
wget https://www.cs.uni-potsdam.de/wv/projects/rosoclingo/rosoclingo-2.0.tar.xz
tar -zxvf rosoclingo.tar.gz
cd ..
catkin_make
```
For more detailed description regarding the installation and usages of ros(o)clingo, please refer to https://www.cs.uni-potsdam.de/wv/rosoclingo/.

# Running steps

1. launch the turtlebo3 gazebo controller
roslaunch rosplan_turtlebot3_demo turtlebot_main.launch
2. launch the ASP solver
rosrun rosoclingo-1.0 run-1.0.py --file ../mailbot/instances/graph_wp.lp --file ../mailbot/turtlebot3_asp_model.lp


# Demonstration videos

1. The demonstration video of experimental details for the indoor navigation task domain is as follows:

[![Experimental demo for Hybrid-ASP planning](https://res.cloudinary.com/marcomontalbano/image/upload/v1632130243/video_to_markdown/images/youtube--2WkFsKWoYWI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/2WkFsKWoYWI "Experimental demo for Hybrid-ASP planning")

2. The demonstration video of new task domain of object pickup is as follows:

[![Tiago object pickup task demonstration](https://res.cloudinary.com/marcomontalbano/image/upload/v1636131521/video_to_markdown/images/youtube--54F45bk8yRY-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/54F45bk8yRY "Tiago object pickup task demonstration")

Contact email: 1024809808@qq.com

