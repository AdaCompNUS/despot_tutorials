# DESPOT Tutorials

An example of using [DESPOT](https://github.com/AdaCompNUS/despot) with real-robots via ROS. We implement a slightly modified version of the Laser Tag problem: a robot chases another robot with noisy laser-sensor observations. The simulation consists of two holonomic robots ([KUKA Youbot](http://www.youbot-store.com/)) inside a [Gazebo](http://gazebosim.org/) environment


[1] N. Ye, A. Somani, D. Hsu, and W. Lee. [**DESPOT: Online POMDP planning with regularization**](http://bigbird.comp.nus.edu.sg/m2ap/wordpress/wp-content/uploads/2017/08/jair14.pdf). J. Artificial Intelligence Research, 58:231–266, 2017.

[Copyright &copy; 2014-2017 by National University of Singapore](http://motion.comp.nus.edu.sg/).

## Requirements

Tested Operating Systems:

| Ubuntu 14.04     
| :-------------: 
|[![Build Status](https://semaphoreapp.com/api/v1/projects/d4cca506-99be-44d2-b19e-176f36ec8cf1/128505/shields_badge.svg)](https://semaphoreapp.com/boennemann/badges)    

Dependencies: DESPOT, ROS Indigo, Boost 1.55, Gazebo

## Installation

Install ROS Indigo. We recommend `sudo apt-get install ros-indigo-desktop-full`. 