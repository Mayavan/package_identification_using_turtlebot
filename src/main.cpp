/**
 * @file main.cpp
 * @brief file with main function to start the execution
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
#include "package_identification_using_turtlebot/QReader.hpp"

#include <ros/package.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "QBot");
  ros::NodeHandle node;
  // Initialization
  std::vector<std::vector<double>> points = {{2.4, 0.30, 0.1, 0.9},
                                             {2.5, 1.808, 0.0, 1.0},
                                             {2.4, 3.175, 0.0, 1.0},
                                             {2.4, 4.36, 0.0, 1.0},
                                             {0.0, 0.0, 0.0, 1.0}};
  PathPlanner planner(points);
  std::vector<std::string> packID = planner.sendGoals();
  int k = planner.findPackage(packID);
  // std::string str;
  // std::string fileLocation =
  //     ros::package::getPath("package_identification_using_turtlebot") +
  //     "/data/sym.png";
  // cv::Mat image = cv::imread(fileLocation);
  // QReader reader;
  // reader.setImage(image);
  // std::vector<uint8_t> result = reader.decodeQR();
  // str.assign(result.begin(), result.end());
  // ROS_INFO_STREAM(str.c_str());
  return 0;
}
