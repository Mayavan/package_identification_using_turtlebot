/**
 * @file  PathPlanner.cpp
 * @brief File With the definitions of the PathPlanner class
 * methods.
 *
 * @author RajendraMayavan
 * @author Adarsh Jagan
 * @copyright BSD-3-Clause License
 * Copyright (c) 2018, Mayavan,  Adarsh Jagan Sathyamoorthy
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "package_identification_using_turtlebot/PathPlanner.hpp"
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include "package_identification_using_turtlebot/QReader.hpp"

PathPlanner::PathPlanner(std::vector<std::vector<double> > points) {
  initPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1000);
  move_base_msgs::MoveBaseGoal targetPose;
  // Initialize the intial pose
  initialPose.header.frame_id = "map";
  initialPose.pose.pose.position.x = 0.0;
  initialPose.pose.pose.position.y = 0.0;
  initialPose.pose.pose.orientation.w = 1.0;
  counter = 0;

  // Add the points to the goal vector
  targetPose.target_pose.header.frame_id = "map";
  targetPose.target_pose.header.stamp = ros::Time::now();
  for (auto i = points.begin(); i != points.end() && counter < points.size();
       i++) {
    targetPose.target_pose.pose.position.x = (*i).at(0);
    targetPose.target_pose.pose.position.y = (*i).at(1);
    targetPose.target_pose.pose.orientation.z = (*i).at(2);
    targetPose.target_pose.pose.orientation.w = (*i).at(3);
    goal.push_back(targetPose);
  }
}

PathPlanner::~PathPlanner() {}

ros::Publisher PathPlanner::returnPublisher() { return initPosePub; }

std::vector<double> PathPlanner::publishInitPose(double x, double y, double w) {
  ROS_DEBUG_STREAM("Inside init pose publisher");
  // Update initial pose from arguments
  initialPose.pose.pose.position.x = x;
  initialPose.pose.pose.position.y = y;
  initialPose.pose.pose.orientation.w = w;
  // Publish the new pose
  initPosePub.publish(initialPose);

  std::vector<double> initPose;
  initPose.push_back(x);
  initPose.push_back(y);
  initPose.push_back(w);
  return initPose;
}

std::vector<double> PathPlanner::callPublisher(double x, double y, double w) {
  // Calls the private class (For rostest)
  return publishInitPose(x, y, w);
}

std::string PathPlanner::waitPackageDetection(std::string str) {
  // wait until package is detected
  while (str.substr(0, 4) != "pack") {
    ros::spinOnce();
    std::vector<uint8_t> result = reader.returnBytes();
    str.assign(result.begin(), result.end());
    // Check if QR code is detected properly
    if (str.substr(0, 4) == "pack") {
      ROS_INFO("QR code detected! \n");
      ROS_INFO("Package ID is: %s \n", str.c_str());
      break;
    }
  }
  return str;
}

std::vector<std::string> PathPlanner::callVision(
    std::vector<std::string> packID) {
  // Call image callback
  ros::spinOnce();
  // Get the updated result
  std::vector<uint8_t> result = reader.returnBytes();
  std::string str;
  str.assign(result.begin(), result.end());
  ROS_INFO("Package ID is: %s \n", str.c_str());

  packID.push_back(waitPackageDetection(str));

  return packID;
}

std::vector<std::string> PathPlanner::sendGoals() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);

  std::vector<std::string> packID;
  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  // Publish initial position (x, y) and orientation quaternion of the
  // Turtlebot. In this case (x, y) = (-3.0, -3.0) and w of quaternion = 1.0 (0
  // degrees) All other parameters are 0.0 by default
  publishInitPose(-3.0, -3.0, 1.0);
  for (auto i = goal.begin(); i != goal.end() && counter < goal.size(); i++) {
    ROS_INFO("Moving to goal %d \n", counter + 1);
    // Send goal to move base
    ac.sendGoal(*i);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      counter = counter + 1;
      ROS_INFO_STREAM("Reached goal %d \n" << counter);
      if (counter == goal.size()) {
        break;
      } else {
        packID = callVision(packID);
      }
    } else {
      ROS_WARN_STREAM("Failed to reach goal");
    }
  }
  return packID;
}

int PathPlanner::findPackage(std::vector<std::string> packID) {
  auto j = goal.begin();
  std::string result =
      "\n*****************************\n******Package "
      "Locations******\n*****************************\n";
  for (auto i = packID.begin(); i != packID.end() && j != goal.end();
       i++, j++) {
    result += (*i) + " is in position x=" +
              std::to_string((*j).target_pose.pose.position.x) +
              " y = " + std::to_string((*j).target_pose.pose.position.y) + "\n";
  }
  ROS_INFO_STREAM(result);
  ros::Duration(10).sleep();
  return 0;
}
