/**
 * @file QReaderTest.cpp
 * @brief file to test the class QReader.cpp
 *
 * @author RajendraMayavan, Adarsh Jagan
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

#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <vector>
#include "package_identification_using_turtlebot/QReader.hpp"

/**
 * @brief      Testing if the decoder function handles unknown encoding properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, unsupportedEncoding) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/numeric_encoding.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("unknown", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 2
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask2) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/apple.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("Apple", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 0
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask0) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/pack1.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("pack#1", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 1
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask1) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/pack5.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("pack#5", str.c_str());
}

/**
 * @brief      Testing if the decoder function to check handling of unknown
 * images
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, unknownImage) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/unknown.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("unknown", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 6
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask6) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/random.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("gfgdWcvd", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 3
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask3) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/mask3.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("unknown", str.c_str());
}

/**
 * @brief      Testing if the decoder function can apply mask 7
 *
 * @param[in]     TESTSuite
 * @param[in]     testService
 *
 * @return     none
 */
TEST(decodeQR, mask7) {
  std::string str;
  std::string fileLocation =
      ros::package::getPath("package_identification_using_turtlebot") +
      "/data/mask7.png";
  cv::Mat image = cv::imread(fileLocation);
  QReader reader;
  reader.setImage(image);
  std::vector<uint8_t> result = reader.decodeQR();
  str.assign(result.begin(), result.end());
  ASSERT_STREQ("unknown", str.c_str());
}
