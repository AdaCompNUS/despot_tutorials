# DESPOT Tutorials

An example of using [DESPOT](https://github.com/AdaCompNUS/despot) with real-robots via ROS. We implement a slightly modified version of the Laser Tag problem: a robot chases another robot with noisy laser-sensor observations. The simulation consists of two holonomic robots ([KUKA Youbot](http://www.youbot-store.com/)) inside a [Gazebo](http://gazebosim.org/) environment resembling the problem world described in [1] (Page 20).


[1] N. Ye, A. Somani, D. Hsu, and W. Lee. [**DESPOT: Online POMDP planning with regularization**](http://bigbird.comp.nus.edu.sg/m2ap/wordpress/wp-content/uploads/2017/08/jair14.pdf). J. Artificial Intelligence Research, 58:231–266, 2017.

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).

## Requirements

Tested Operating Systems:

| Ubuntu 14.04     
| :-------------: 
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)    

Dependencies: DESPOT, ROS Indigo, Boost 1.55, Gazebo

## Prerequisites

Install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu).
We recommend `ros-indigo-desktop-full` version which includes Gazebo.  

Install DESPOT using [CMakeLists](https://github.com/AdaCompNUS/despot#cmakelists). Make sure that DESPOT binaries and header files are installed.
```bash
$ cd <latest_despot_repo>
$ mkdir build; cd build

$ cmake -DCMAKE_BUILD_TYPE=Release ../ 
$ make
$ sudo make install
```

Install BOOST libraries with `sudo apt-get install libboost-all-dev` 

## Installation

If you haven't sourced your ROS environment, run:
```bash
$ source /opt/ros/indigo/setup.bash
```

Setup a fresh catkin workspace for **despot_tutorials**:

```bash
$ mkdir -p ~/despot_ws/src
$ cd ~/despot_ws/
$ catkin_make
$ source devel/setup.bash
```

Clone the repository:
```bash
$ cd ~/despot_ws/src
$ git clone https://github.com/AdaCompNUS/despot_tutorials.git
```

Compile:
```bash
cd ~/despot_ws
catkin_make
```

## Usage

Launch the Gazebo environment and robot controllers:
```bash
$ roslaunch laser_tag laser_tag.launch R1_noise:=0.5
```

On a separate terminal, run the POMDP planner:
```bash
$ rosrun laser_tag pomdp_planner
```

You should see a 3D 7x11 grid world with two Youbots. The green robot should chase the red robot until 'Tag' is called. The `R1_noise` parameter specifies the gaussian noise (standard deviation in meters) of the green robot's laser range finder.  