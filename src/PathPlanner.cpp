/*
 * PathPlanner.cpp
 *
 *  Created on: Dec 8, 2018
 *      Author: root
 */
#include "package_identification_using_turtlebot/PathPlanner.hpp"

PathPlanner::PathPlanner(std::vector<std::vector<double>> points) {
  initPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1000);
  initialPose.header.frame_id = "map";
  goal.target_pose.header.frame_id = "map";
  initialPose.pose.pose.position.x = 0.0;
  initialPose.pose.pose.position.y = 0.0;
  initialPose.pose.pose.orientation.w = 1.0;
  goalPoints = points;  //
  counter = 0;
}

PathPlanner::~PathPlanner() {}

ros::Publisher PathPlanner::returnPublisher() {
  return initPosePub;
}

std::vector<double> PathPlanner::publishInitPose(double x, double y, double w) {
  std::cout << "Inside init pose publisher" << std::endl;
  initialPose.pose.pose.position.x = x;
  initialPose.pose.pose.position.y = y;
  initialPose.pose.pose.orientation.w = w;
  initPosePub.publish(initialPose);
  std::vector<double> initPose;
  initPose.push_back(x);
  initPose.push_back(y);
  initPose.push_back(w);
  return initPose;
}

std::vector<double> PathPlanner::callPublisher(double x, double y, double w) {
  std::cout << "Inside call publisher function" << std::endl;
  return publishInitPose(x, y, w);
}

std::vector<std::string> PathPlanner::callVision(
    QReader& reader, std::vector<std::string> packID) {
  // Call image callback
  ros::spinOnce();
  std::vector<uint8_t> result = reader.returnBytes();
  std::string str;
  str.assign(result.begin(), result.end());
  ROS_INFO("Package ID is: %s \n", str.c_str());
  if (str.substr(0, 4) == "pack") {
    packID.push_back(str);
  }
  // wait until package is detected
  while (str.substr(0, 4) != "pack") {
    ros::spinOnce();
    result = reader.returnBytes();
    str.assign(result.begin(), result.end());
    if (str.substr(0, 4) == "pack") {
      ROS_INFO("QR code detected! \n");
      ROS_INFO("Package ID is: %s \n", str.c_str());
      packID.push_back(str);
      break;
    }
    ROS_INFO("Waiting for QR code to be detected \n");
  }
  return packID;
}

std::vector<std::string> PathPlanner::sendGoals() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);
  QReader reader;
  std::vector<std::string> packID;
  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  // Publish initial position (x, y) and orientation quaternion of the
  // Turtlebot. In this case (x, y) = (-3.0, -3.0) and w of quaternion = 1.0 (0
  // degrees) All other parameters are 0.0 by default
  publishInitPose(-3.0, -3.0, 1.0);
  goal.target_pose.header.stamp = ros::Time::now();
  for (auto i = goalPoints.begin();
      i != goalPoints.end() && counter < goalPoints.size(); i++) {
    ROS_INFO("Moving to goal %d \n", counter + 1);
    goal.target_pose.pose.position.x = (*i).at(0);
    goal.target_pose.pose.position.y = (*i).at(1);
    goal.target_pose.pose.orientation.z = (*i).at(2);
    goal.target_pose.pose.orientation.w = (*i).at(3);
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      counter = counter + 1;
      ROS_INFO("Reached goal %d \n", counter);
      if (counter == goalPoints.size()) {
        break;
      } else {
        packID = callVision(reader, packID);
      }
    } else
      ROS_INFO("Failed to reach goal");
  }
  return packID;
}

int PathPlanner::findPackage(std::vector<std::string> packID) {
  auto j = goalPoints.begin();
  std::cout << "*****************************" << std::endl;
  std::cout << "******Package Locations******" << std::endl;
  std::cout << "*****************************" << std::endl;
  for (auto i = packID.begin(); i != packID.end() && j != goalPoints.end();
      i++, j++) {
    std::cout << (*i) << " is in position x=" << (*j).at(0) << " y = "
              << (*j).at(1) << std::endl;
  }
  ros::Duration(10).sleep();
  return 0;
}
