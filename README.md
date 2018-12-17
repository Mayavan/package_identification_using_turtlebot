# ENPM 808X Final - Package-Identification-using-Turtlebot

[![Build Status](https://travis-ci.org/Mayavan/package_identification_using_turtlebot.svg?branch=master)](https://travis-ci.org/Mayavan/package_identification_using_turtlebot)

[![Coverage Status](https://coveralls.io/repos/github/Mayavan/package_identification_using_turtlebot/badge.svg?branch=master)](https://coveralls.io/github/Mayavan/package_identification_using_turtlebot?branch=master)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

Robots have been used to improve efficiency in a various tasks in warehouses such as package handling, identification and organization. This project aims to simulate the use of Turtlebot in identifying packages with QR codes, in a warehouse setting. The simulation is implemented in Gazebo. ROS packages such as gmapping, and teleop_twist_keyboard are used for mapping the environment in the simulation, while move_base is used for navigation.

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/navspeed.gif?raw=true" alt="Turtlebot navigating towards packages."/>
</p>

 Our main focus in this project is QR-code decoding. Image stream from Turtlebot's simulated camera is subscribed and processed to obtain a clear front view of the QR code. The masking and encoding type of the QR code is detected, and it is unmasked to extract a bit stream, which is then coverted to a string. This resultant string is the package ID. Each package ID must have a valid prefix to ensure that appropriate packages have arrived at the warehouse. In our case we use "pack" as a valid prefix. Since there can be errors in odometry in the Turtlebot, it may not always reliably orient itself near the packages which results in a slanted perspective.To ensure that the package identification is robust to such odometry errors, we threshold and isolate the QR code first and then detect Harris corners. These corner points are then used to warp the perspective of the QR code. Only then the decoding is applyed to the image.

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/transform.gif?raw=true" alt="Decoding the QR code."/>
</p>

The scenario is that some new packages of unknown identities have arrived at the warehouse and are stored in a location allotted to "new arrivals". This location is generally known, along with the floor plan of the warehouse. The Turtlebot starts from another corner of the warehouse and moves towards the packages avoiding all "mapped" obstacles as well as "unmapped" obstacles (the dustbin in front of one of the packages is unmapped.) The Turtlebot then sequentially moves from one package to another identifying and storing the package IDs and locations from which each package can be accessed. 

<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/moving_between_packages.gif?raw=true" alt="Moving to next package when current package is identified."/>
</p>

## Presentation
[Here](https://youtu.be/TZWnTCO4xOE) is the link to our video presentation and [here](https://docs.google.com/presentation/d/1z3j81NSBstdXP-oASMYSJyp5DKBPQ6vCecCbNV_1284/edit?usp=sharing) is the link to the presentation slides.

## SIP

The product backlog, iteration log and time log sheets can be found [here.](https://docs.google.com/spreadsheets/d/1RWIvnbdE3t9a1EoGMhIvIEiinyssCJ5bO6Itf2WrIy8/edit?usp=sharing)

The sprint planning and review document can be viewed [here.](https://docs.google.com/document/d/1Zp-uh8ouf0MiTm6qED7pZ7B7P1d_7mmyq6RMpuOTYXM/edit?usp=sharing)

## Dependencies

This project is run on top of Ubuntu 16.04 with ROS Kinetic and Gazebo 7.14. 

To install ROS Kinetic in Ubuntu 16.04, follow the steps given [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Catkin can be installed from [here](http://wiki.ros.org/catkin).

ROS packages such as move_base, actionlib, and gazebo_ros packages should be installed as:

```
sudo apt-get install ros-kinetic-move-base ros-kinetic-actionlib ros-kinetic-gazebo-ros
```

Turtlebot packages that are required can be installed as:

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

For the vision aspects of the project, we use OpenCV 3.3.1. It can be installed from [here](https://www.learnopencv.com/install-opencv3-on-ubuntu/)

## Download
Before building the package, ensure that your catkin workspace is set up properly. In a new terminal, type the following commands:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
For more details follow the steps [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 

Download this repository inside src directory inside your catkin workspace.
```
cd ~/catkin_ws/src/
git clone https://github.com/Mayavan/Package_Identification_using_Turtlebot.git
```

## Build
After cloning, you should see a package inside src. To build this package, when you're inside the src folder:
```
cd ..
catkin_make
source devel/setup.bash
```

## Running Demo
To run a fullfledged demo of the Turtlebot in the warehouse setting identifying packages, give:

```
roslaunch package_identification_using_turtlebot demo.launch
```

In Gazebo, the Turtlebot spawns at (-3, -3) and starts navigating towards the packages. Once a goal is reached, ROS info messages will display the state if the identification (whether package is IDed or Turtlebot is waiting for proper identification). When the four packages are detected, the Turtlebot moves to the origin. The package IDs and their locations are then printed and code exits.

Additionally, the navigation, global and local costmaps can be viewed in Rviz.
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
For additional information see [here](http://learn.turtlebot.com/2015/02/03/8/).
<p align="center">
  <img src="https://github.com/Mayavan/Package_Identification_using_Turtlebot/blob/master/images/rviz.png?raw=true" alt="Turtlebot Rviz."/>
</p>

The code related to this demo can be found in the src, launch and include folders.

## Test
To run the test cases, run:
```
 rostest package_identification_using_turtlebot Qbot.test
```

All test cases in your catkin workspace can be run while compiling by:
```
catkin_make run_tests
```

This runs 12 test cases for various methods in the PathPlanner and QReader classes. The code for the tests can be found in the test folder.

## Doxygen Documentation

Click [here](https://mayavan.github.io/package_identification_using_turtlebot/) to view documentation.

To install Doxygen:
```
sudo apt-get install doxygen
``` 
To generate Doxygen documentation, run:
```
doxygen Doxyfile
```
 

## Known Issues and Bugs
* In very rare situations, the Turtlebot might get stuck for sometime in detecting QR codes if 4 corner points of the QR code are not detected. This is mainly due to the simulated errors in odometry in Gazebo which results in very slanted perspectives. Another solution is to further tune the parameters in Harris corner detection to detect more points (which is kept conservative as of now).

* The move_base action client in the code has not been tested due to dependency issues for the turtlebot_gazebo packages in Travis. Testing this code would increase coverage to 97-98%.


## License

```
Copyright (c) 2018, Mayavan,  Adarsh Jagan Sathyamoorthy 
 
Redistribution and use in source and binary forms, with or without  
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the   
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
```

## Authors

* Adarsh Jagan Sathyamoorthy Major: I am a 2nd year Robotics graduate student at UMD College Park. I have a Bachelors degree in Electronics and Communication from National Institute of Technology- Tiruchirapalli, India. My primary interests are Planning, computer vision, and robot learning.  For more about me, please see my [linkedin](https://www.linkedin.com/in/adarsh-jagan-sathyamoorthy-6b6726b3/). Email: asathyam@umd.edu
* RajendraMayavan Rajendran Sathyam: I am doing my second year master's in Robotics. I completed my Bachelors in Electrical and Electronics engineering in Anna University, Chennai, India. I am interested in Industrial Automation and computer vision. E-Mail: mayavan@umd.edu
