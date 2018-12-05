# Package-Identification-using-Turtlebot

[![Build Status](https://travis-ci.org/Mayavan/Package_Identification_using_Turtlebot.svg?branch=master)](https://travis-ci.org/Mayavan/Package_Identification_using_Turtlebot)

[![Coverage Status](https://coveralls.io/repos/github/Mayavan/Package_Identification_using_Turtlebot/badge.svg?branch=master)](https://coveralls.io/github/Mayavan/Package_Identification_using_Turtlebot?branch=master)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Authors 
* Adarsh Jagan Sathyamoorthy
* RajendraMayavan Rajendran Sathyam

## Overview

Robots have been used to improve efficiency in a various tasks in warehouses such as package handling, identification and organization. This project aims to simulate the use of Turtlebot in identifying packages with QR codes, in a warehouse setting. 

The simulation is implemented in Gazebo. ROS packages such as gmapping, and teleop_twist_keyboard are used for mapping the environment in the simulation, while move_base is used for navigation. 

## SIP

The product backlog, iteration log and time log sheets can be found [here.](https://docs.google.com/spreadsheets/d/1RWIvnbdE3t9a1EoGMhIvIEiinyssCJ5bO6Itf2WrIy8/edit?usp=sharing)

The sprint planning and review document can be viewed [here.](https://docs.google.com/document/d/1Zp-uh8ouf0MiTm6qED7pZ7B7P1d_7mmyq6RMpuOTYXM/edit?usp=sharing)

## Dependencies

This project is run on top of Ubuntu 16.04 with ROS Kinetic and Gazebo 7.14. 

This project requires the following packages to be installed, they can be installed by running:

Ensure that packages such as move_base, actionlib, and gazebo_ros packages are installed:
```
sudo apt-get install ros-kinetic-move-base ros-kinetic-actionlib ros-kinetic-gazebo-ros
```
Turtlebot packages required are:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
For the vision aspects of the project, we use OpenCV 3.3.1. It can be installed from [here](https://www.learnopencv.com/install-opencv3-on-ubuntu/)


## Run

## Test

## Running Demo

## Known Issues and Bugs

## API and other documentation
