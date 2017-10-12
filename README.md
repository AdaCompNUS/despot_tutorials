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

Install DESPOT using [CMakeLists](https://github.com/AdaCompNUS/despot#cmakelists). Make sure that DESPOT binaries and header files are installed with `sudo make install` 

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
$ roslaunch laser_tag laser_tag.launch
```
You should see a 3D 7x11 grid world with two Youbots.  

#### (Optional)
#### Test Controllers
Run a test to see if you can control the robots:
```bash
$ rosrun laser_tag test_laser_tag_controller
```
The first robot (bottom left) should move North. And the other robot should try and run away. After executing the action, you should see a print-out of the laser-readings:

```
[ INFO] [1507790509.778345756, 1131.223000000]: Laser Observations
[ INFO] [1507790509.778420017, 1131.223000000]: North: 0
[ INFO] [1507790509.778456571, 1131.223000000]: East: 7
[ INFO] [1507790509.778491221, 1131.223000000]: South: 2
[ INFO] [1507790509.778523052, 1131.223000000]: West: 3
[ INFO] [1507790509.778550735, 1131.223000000]: NorthEast: 4
[ INFO] [1507790509.778578519, 1131.223000000]: SouthEast: 4
[ INFO] [1507790509.778604918, 1131.223000000]: SouthWest: 4
[ INFO] [1507790509.778631267, 1131.223000000]: NorthWest: 5
```

See [test_laser_tag_controller.cpp](examples/laser_tag/test/test_laser_tag_controller.cpp) for details on giving an action to the robot, and receiving observations.

#### Command-Line Interface
You can also send actions to the robot directly using rosservice
```bash
$ rosservice call /laser_tag_action_obs "action: 1"
```

#### Tuning Noise
The raw laser sensor readings are perturbed by a Gaussian noise of stddev `0.5m`. You can change this in the robot description [URDF file](/robots/youbot/youbot_description/robots/youbot_base_laser.urdf.xacro), by changing the `noise_stddev` parameter in:
```xml
  <xacro:hokuyo_urg04_laser name="base_laser_front" parent="base" ros_topic="laser_scan" update_rate="10" min_angle="-3.14159" max_angle="3.14159" noise_stddev="0.5">
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
  </xacro:hokuyo_urg04_laser>
```

## TODO

Integrating DESPOT