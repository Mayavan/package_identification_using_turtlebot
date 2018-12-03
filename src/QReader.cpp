/**
 * @file QReader.cpp
 * @brief Class to get the image and decode the QR code in the
 * image to extract data of package ID.
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

#include "Package_Identification_using_Turtlebot/QReader.hpp"

/**
 * @brief  Constructs a object
 */
QReader::QReader() {}

/**
 * @brief Destroys the object
 */
QReader::~QReader() {}

std::vector<uint8_t> QReader::decodeQR(cv::Mat) {
  cv::Mat imgBW = captureImage();
  bool found = checkQCodeExists(imgBW);
}

cv::Mat QReader::captureImage() {
  std::string fileLocation = "../Test/QRCode.png";
  cv::Mat img = cv::imread(fileLocation);
  cv::Mat imgBW;
  cvtColor(img, imgBW, CV_BGR2GRAY);
  adaptiveThreshold(imgBW, imgBW, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                    CV_THRESH_BINARY, 51, 0);
  return imgBW;
}

/**
 * @brief A function to check if QR code exists in the image.
 *
 * @detail The QR code has 3 finder patterns. This functions looks for a
 * sequence of pixels which transitions from white to black.
 *
 * @param img Input image to check for QR code
 *
 * @return true if the image has a QR code in it, else returns false
 */
bool QReader::checkQCodeExists(cv::Mat &img) {
  possibleCenters.clear();
  estimatedModuleSize.clear();

  int skipRows = 3;
  std::vector<int> stateCount(5, 0);
  int currentState = 0;
  for (int row = skipRows - 1; row < img.rows; row += skipRows) {
    stateCount = {0, 0, 0, 0, 0};
    currentState = 0;
    const uchar *ptr = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; col++) {
      if (ptr[col] < 128) {
        // Found Black Pixel
        if ((currentState & 0x1) == 1) {
          // If transitioned from white pixel
          currentState++;
        }

        /* Add the count of number of pixel in the current state
         * (new state if transitioned form white)
         */
        stateCount[currentState]++;
      } else {
        // Found white pixel
        if ((currentState & 0x1) == 1) {
          // If the previuos state was also white (we can tell because the
          // currentState is an odd number)
          stateCount[currentState]++;
        } else {
          // If the previous state was black
          if (currentState == 4) {
            // Check if the previous state count satisfies the expected ratio
            // Do processing for it here
            if (checkRatio(stateCount)) {
              // Do more checks is see if it actually is a finder pattern

              // Dummy implementation to check algorithm
              possibleCenters.push_back(cv::Point2f(0.3f, 0.f));

            } else {
              // If the ratio is not satisfied, swift the states two step front
              currentState = 3;
              stateCount.erase(stateCount.begin(), stateCount.begin() + 2);
              stateCount.push_back(1);
              stateCount.push_back(0);
              continue;
            }
            currentState = 0;
            stateCount = {0, 0, 0, 0, 0};
          } else {
            // If transitioned from black to white and we are still not at the
            // final state
            currentState++;
            stateCount[currentState]++;
          }
        }
      }
    }
  }
  if (possibleCenters.size() != 3) {
    return false;
  } else {
    return true;
  }
}

/**
 * @brief checks if the number of pixels in each states are in the ratio
 * 1:1:3:1:1
 * @param stateCount containing the number of pixels with black or white pixels
 * @return true if the number of pixels in each states are in the ratio
 * 1:1:3:1:1 or false
 */
bool QReader::checkRatio(std::vector<int> stateCount) {
  int totalCount = 0;
  // find the total cols in stateCount
  for (auto &count : stateCount) {
    totalCount += count;
  }

  if (totalCount < 7) return false;

  // Do normalization for counting ratio
  int moduleSize = ceil(totalCount / 7.0);
  int tolerance = moduleSize / 2;

  // Check if each module size of 1:1:3:1:1 ratio is within tolerance
  return ((abs(moduleSize - (stateCount[0])) < tolerance) &&
          (abs(moduleSize - (stateCount[1])) < tolerance) &&
          (abs(3 * moduleSize - (stateCount[2])) < 3 * tolerance) &&
          (abs(moduleSize - (stateCount[3])) < tolerance) &&
          (abs(moduleSize - (stateCount[4])) < tolerance));
}

cv::Mat warpToCode(cv::Mat) {}
std::vector<uint8_t> extractBits(cv::Mat) {}

std::vector<char> decodeBits(std::vector<uint8_t>) {}
