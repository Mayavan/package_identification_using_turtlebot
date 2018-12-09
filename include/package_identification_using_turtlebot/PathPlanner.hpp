/*
 * PathPlanner.hpp
 *
 *  Created on: Dec 8, 2018
 *      Author: root
 */

#ifndef PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_
#define PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PathPlanner {
 private:
  ros::NodeHandle nh;
  ros::Publisher initPosePub;
  geometry_msgs::PoseWithCovarianceStamped initialPose;
  move_base_msgs::MoveBaseGoal goal;
  std::vector<std::vector<double>> goalPoints;
  int counter;
  bool reachedGoal;
  void publishInitPose(double x, double y, double w);

 public:
  PathPlanner();
  ~PathPlanner();
  void sendGoals();
  bool returnReachedGoal();
};




#endif /* PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_ */
