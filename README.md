# ENPM 808X Final - Package-Identification-using-Turtlebot

[![Build Status](https://travis-ci.org/Mayavan/Package_Identification_using_Turtlebot.svg?branch=master)](https://travis-ci.org/Mayavan/Package_Identification_using_Turtlebot)

[![Coverage Status](https://coveralls.io/repos/github/Mayavan/Package_Identification_using_Turtlebot/badge.svg?branch=master)](https://coveralls.io/github/Mayavan/Package_Identification_using_Turtlebot?branch=master)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

Robots have been used to improve efficiency in a various tasks in warehouses such as package handling, identification and organization. This project aims to simulate the use of Turtlebot in identifying packages with QR codes, in a warehouse setting. The simulation is implemented in Gazebo. ROS packages such as gmapping, and teleop_twist_keyboard are used for mapping the environment in the simulation, while move_base is used for navigation.

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/navspeed.gif?raw=true" alt="Turtlebot navigating towards packages."/>
</p>

 Our main focus in this project is QR-code decoding. Image stream from Turtlebot's simulated camera is subscribed and processed to obtain a clear front view of the QR code. The masking and encoding type of the QR code is detected, and it is unmasked to extract a bit stream, which is then coverted to a string. This resultant string is the package ID. Each package ID must have a valid prefix to ensure that appropriate packages have arrived at the warehouse. In our case we use "pack" as a valid prefix. 

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/transform.gif?raw=true" alt="Decoding the QR code."/>
</p>

The scenario is that some new packages of unknown identities have arrived at the warehouse and are stored in a location allotted to "new arrivals". This location is generally known, along with the floor plan of the warehouse. The Turtlebot starts from another corner of the warehouse and moves towards the packages avoiding all "mapped" obstacles as well as "unmapped" obstacles (the dustbin in front of one of the packages is unmapped.) The Turtlebot then sequentially moves from one package to another identifying and storing the package IDs and locations from which each package can be accessed. 

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/moving_between_packages.gif?raw=true" alt="Moving to next package when current package is identified."/>
</p>

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

```
roslaunch package_identification_using_turtlebot demo.launch
```

## Known Issues and Bugs

## API and other documentation

## Authors

* Adarsh Jagan Sathyamoorthy Major: Robotics, E-Mail: asathyam@umd.edu
* RajendraMayavan Rajendran Sathyam Major: Robotics, E-Mail: mayavan@umd.edu
