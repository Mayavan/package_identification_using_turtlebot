/*
 * PathPlanner.cpp
 *
 *  Created on: Dec 8, 2018
 *      Author: root
 */
#include "package_identification_using_turtlebot/PathPlanner.hpp"
#include "package_identification_using_turtlebot/QReader.hpp"

PathPlanner::PathPlanner() {
  initPosePub = nh
      .advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1000);
  initialPose.header.frame_id = "map";
  goal.target_pose.header.frame_id = "map";
  initialPose.pose.pose.position.x = 0.0;
  initialPose.pose.pose.position.y = 0.0;
  initialPose.pose.pose.orientation.w = 1.0;
  goalPoints = { {0, 0}, {1, 1}, {2, 1},
    { 2, 2}};
  counter = 0;
  reachedGoal = false;

}

PathPlanner::~PathPlanner() {

}

void PathPlanner::publishInitPose(double x, double y, double w) {
  initialPose.pose.pose.position.x = x;
  initialPose.pose.pose.position.y = y;
  initialPose.pose.pose.orientation.w = w;
  initPosePub.publish(initialPose);
}

void PathPlanner::sendGoals() {
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);
  QReader reader;

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Publish initial position (x, y) and orientation quaternion of the Turtlebot.
  // In this case (x, y) = (-3.0, -3.0) and w of quaternion = 1.0 (0 degrees)
  // All other parameters are 0.0 by default
  publishInitPose(-3.0, -3.0, 1.0);

  goal.target_pose.header.stamp = ros::Time::now();
  for (int i = 0; i < goalPoints.size() && counter < goalPoints.size(); i++) {
    std::cout << goalPoints[i][0] << "  " << goalPoints[i][1] << std::endl;
    goal.target_pose.pose.position.x = goalPoints[i][0];
    goal.target_pose.pose.position.y = goalPoints[i][1];
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    ros::spinOnce();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      counter = counter + 1;
      ROS_INFO("Goal reached.");
      reachedGoal = true;
      std::vector<uint8_t> bytes = reader.decodeQR();
      for (auto i : bytes)
        std::cout << i;
      std::cout << std::endl;
    } else
      ROS_INFO("Failed to reach goal");
  }
}

bool PathPlanner::returnReachedGoal() {
  return reachedGoal;
}




