/**
 * @file PathPlanner.hpp
 * @brief File that has declarations for the PathPlanner class
 *
 * @author RajendraMayavan
 * @author Adarsh Jagan
 *
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

#ifndef PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_
#define PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include "package_identification_using_turtlebot/QReader.hpp"

/**
 * @brief Class PathPlanner has methods send goal points and initial pose of the
 * turtlebot to move_base package. Once the packages are identified, this class
 * prints the package IDs and locations.
 */
class PathPlanner {
 private:
  /**
   * @brief Nodehandle to manage the ROS node
   */
  ros::NodeHandle nh;
  /**
   * @brief Publisher object to broadcast initial pose of
   * the turtlebot
   */
  ros::Publisher initPosePub;
  /**
   * @brief Stores the initial pose of the turtlebot. Will be published.
   */
  geometry_msgs::PoseWithCovarianceStamped initialPose;
  /**
   * @brief Stores the goal positions and orientations of the turtlebot.
   */
  move_base_msgs::MoveBaseGoal goal;
  /**
   * @brief Vector that stores a list of goal position.x, position.y
   * orientation.z, and orientation.w. (in that order)
   */
  std::vector<std::vector<double>> goalPoints;
  /**
   * @brief Stores the number of goals that have been reached
   */
  int counter;
  /**
   * @brief Private Method that publishes initial pose of turtlebot
   * @param x of type double
   * @param y of type double
   * @param w of type double
   * @return initPose of type std::vector<double>
   */
  std::vector<double> publishInitPose(double x, double y, double w);

 public:
  /**
   * @brief Constructor of the PathPlanner class
   * @param points of type std::vector<std::vector<double>>
   * @return none
   */
  PathPlanner(std::vector<std::vector<double>>);
  /**
   * @brief Destructor of the PathPlanner class
   * @param none
   * @return none
   */
  ~PathPlanner();
  /**
   * @brief Method that returns publisher object
   * @param none
   * @return initPosePub of type ros::Publisher
   */
  ros::Publisher returnPublisher();
  /**
   * @brief Method to call publishInitPose
   * @param x of type double
   * @param y of type double
   * @param w of type double
   * @return initPose of type std::vector<double>
   */
  std::vector<double> callPublisher(double, double, double);
  /**
   * @brief  Method to call callback function from QReader class
   * @param  object of QReader class
   * @param  packID of type std::vector<std::string>
   * @return packID of type std::vector<std::string>
   */
  std::vector<std::string> callVision(QReader&, std::vector<std::string>);
  /**
   * @brief  Method to send goals to move_base action server
   * @param  none
   * @return packID of type std::vector<std::string>
   */
  std::vector<std::string> sendGoals();
  /**
   * @brief  Method to print the packIDs and their location where to find them
   * @param  packID of type std::vector<std::string>
   * @return exit status of type int
   */
  int findPackage(std::vector<std::string>);
  /**
   * @brief  Method to try again if the package detection is faulty
   * @param QReader object of QReader class
   * @param  str of type string
   * @return none
   */
  std::string waitPackageDetection(QReader&, std::string);
};

#endif /* PACKAGE_IDENTIFICATION_USING_TURTLEBOT_INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_PATHPLANNER_HPP_ \
        */
