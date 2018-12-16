/**
 * @file PathPlannerTest.cpp
 * @brief file to test the class PathPlanner
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

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <ros/connection_manager.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <vector>
#include "package_identification_using_turtlebot/PathPlanner.hpp"

void testCb(const geometry_msgs::PoseWithCovarianceStamped msg) {}

/**
 * @brief      Test to check intiPose publisher
 * @param[in]  TESTSuite
 * @param[in]  test
 * @return     none
 */
TEST(publishInitPoseTest, testPlanner1) {
  ros::NodeHandle nh;
  PathPlanner planner({{0.0, 0.0, 0.0, 1.0}});
  ros::Publisher pub = planner.returnPublisher();
  ros::Subscriber sub = nh.subscribe("/initialpose", 1, &testCb);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  std::vector<double> res = planner.callPublisher(2.0, 1.0, 1.0);
  ASSERT_EQ(res.at(0), 2.0);
  ASSERT_EQ(res.at(1), 1.0);
  ASSERT_EQ(res.at(2), 1.0);
}

/**
 * @brief      Test if findPackage executes successfully
 * @param[in]  TESTSuite
 * @param[in]  test
 * @return     none
 */
TEST(findPackageTest, testPlanner2) {
  PathPlanner planner({{-2.0, -3.0, 0.0, 1.0}, {-1.0, -2.0, 0.0, 1.0}});
  std::vector<std::string> packID;
  packID.push_back("pack#1");
  packID.push_back("pack#2");
  int i = planner.findPackage(packID);
  ASSERT_EQ(i, 0);
}

/**
 * @brief      Test to check if QR code is properly detected
 * @param[in]  TESTSuite
 * @param[in]  test
 * @return     none
 */
TEST(callVisionTest, testPlanner3) {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Advertise a known image on camera/rgb/image_raw topic
  image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/pack4.png";
  cv::Mat image = cv::imread(fileLocation);
  cv::waitKey(3);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  ros::Rate loop_rate(5);
  PathPlanner planner({{-2.0, -3.0, 0.0, 1.0}, {-1.0, -2.0, 0.0, 1.0}});
  std::vector<std::string> packID;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(15.0);
  // If the test is not completed within timeout the test fails
  while (ros::Time::now() - start_time < timeout) {
    pub.publish(msg);
    packID = planner.callVision(packID);
    if (packID.at(0).substr(0, 4) == "pack") break;
    loop_rate.sleep();
  }

  ASSERT_STREQ("pack#4", (packID.at(0)).c_str());
}

/**
 * @brief      Test to check if a slanted QR code is properly detected
 * @param[in]  TESTSuite
 * @param[in]  test
 * @return     none
 */
TEST(waitPackageDetection, detectionCorrection) {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Advertise a known image on camera/rgb/image_raw topic
  image_transport::Publisher pub = it.advertise("camera/rgb/image_raw", 1);
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/pack3_slant.png";
  cv::Mat image = cv::imread(fileLocation);
  cv::waitKey(3);
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  ros::Rate loop_rate(5);
  PathPlanner planner({{-2.0, -3.0, 0.0, 1.0}, {-1.0, -2.0, 0.0, 1.0}});
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(15.0);
  std::string result;
  // If the test is not completed within timeout the test fails
  while (ros::Time::now() - start_time < timeout) {
    pub.publish(msg);
    std::string incorrectDetection = "hello";
    result = planner.waitPackageDetection(incorrectDetection);
    if (result.substr(0, 4) == "pack") break;
    loop_rate.sleep();
  }
  ASSERT_STREQ("pack#3", result.c_str());
}

/**
 * @brief      Test to check navigation of turtlebot
 * @param[in]  TESTSuite
 * @param[in]  test
 * @return     none
 */
TEST(sendGoals, navigator) {
  // Initialization
  std::vector<std::vector<double> > points = {{2.4, 0.30, 0.1, 0.9},
                                             {0.0, 0.0, 0.0, 1.0}};
  PathPlanner planner(points);
  std::vector<std::string> packID = planner.sendGoals();
  ASSERT_STREQ("pack#4", (*packID.begin()).c_str());
}
