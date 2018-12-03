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
bool QReader::checkQCodeExists(cv::Mat& img) {
  possibleCenters.clear();
  estimatedModuleSize.clear();

  int skipRows = 3;
  std::vector<int> stateCount(5, 0);
  int currentState = 0;
  for (int row = skipRows - 1; row < img.rows; row += skipRows) {
    stateCount = {0, 0, 0, 0, 0};
    currentState = 0;
    const uchar* ptr = img.ptr<uchar>(row);
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
              // Do more checks is see if it actually is a finder pattern and
              // add the position and module size of the patterns
              isFinderPattern(img, stateCount, row, col);
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
 * @param stateCount vector containing the number of pixels with black or white
 * pixels
 * @return true if the number of pixels in each states are in the ratio
 * 1:1:3:1:1 or false
 */
bool QReader::checkRatio(std::vector<int> stateCount) {
  int totalCount = 0;
  // find the total cols in stateCount
  for (auto& count : stateCount) {
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

/**
 * @brief Ensure if the given part of the image has finder pattern
 *
 * @param img input image
 * @param stateCount vector of number of pixels in each transitions from white
 * to black or black to white pixels
 * @param row row number of the pixel in the image
 * @param col column number of the pixel in the image
 *
 * @return false if the row or column goes out of limits or true if it is not
 */
bool QReader::isFinderPattern(const cv::Mat& img, std::vector<int> stateCount,
                              int row, int col) {
  int totalCount = 0;
  for (auto& count : stateCount) {
    totalCount += count;
  }
  // Get a rough estimate of center
  float centerCol = columnCenterEstimate(stateCount, col);

  // Find the refined center in vertical direction
  float centerRow =
      checkVertical(img, row, centerCol, stateCount[2], totalCount);
  if (std::isnan(centerRow)) {
    return false;
  }
  // Find the refined center in horizontal direction
  centerCol =
      checkHorizontal(img, centerRow, centerCol, stateCount[2], totalCount);
  if (std::isnan(centerCol)) {
    return false;
  }

  // Cross check along the diagonal with the new center row and col
  bool validPattern =
      checkDiagonal(img, centerRow, centerCol, stateCount[2], totalCount);
  if (!validPattern) {
    return false;
  }

  cv::Point2f ptNew(centerCol, centerRow);
  float newModuleSize = totalCount / 7.0f;
  bool found = false;
  int idx = 0;

  // Check if the finder pattern is already found
  for (cv::Point2f pt : possibleCenters) {
    cv::Point2f diff = pt - ptNew;
    float dist = sqrt(diff.dot(diff));
    // If the distance between two centers is less than 10px, they're the same.
    if (dist < 10) {
      // If two centers are very close by, we improve the estimate of the center
      // point and module size by taking their means
      pt = pt + ptNew;
      pt.x /= 2.0f;
      pt.y /= 2.0f;
      estimatedModuleSize[idx] =
          (estimatedModuleSize[idx] + newModuleSize) / 2.0f;
      found = true;
      break;
    }
    idx++;
  }
  if (!found) {
    possibleCenters.push_back(ptNew);
    estimatedModuleSize.push_back(newModuleSize);
  }
  return true;
}

/**
 * @brief Find the center point in the vertical direction
 *
 * @param img input image
 * @param startRow Row number of the pixel in the image
 * @param centerCol Estimate of center pixel in the column
 * @param centerCount Number of pixels in the length of center square
 * @param totalCount Number of pixels in the length of the whole finder pattern
 *
 * @return The center of the finder pattern in the vertical direction
 */
float QReader::checkVertical(const cv::Mat& img, int startRow, int centerCol,
                             int centerCount, int totalCount) {
  int maxRows = img.rows;
  std::vector<int> checkStateCount(5, 0);
  int row = startRow;
  // Traverse above the center
  while (row >= 0 && img.at<uchar>(row, centerCol) < 128) {
    checkStateCount[2]++;
    row--;
  }
  if (row < 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (row >= 0 && img.at<uchar>(row, centerCol) >= 128 &&
         checkStateCount[1] < centerCount) {
    checkStateCount[1]++;
    row--;
  }
  if (row < 0 || checkStateCount[1] >= centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (row >= 0 && img.at<uchar>(row, centerCol) < 128 &&
         checkStateCount[0] < centerCount) {
    checkStateCount[0]++;
    row--;
  }
  if (row < 0 || checkStateCount[0] >= centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  // Traverse down the center
  row = startRow + 1;
  while (row < maxRows && img.at<uchar>(row, centerCol) < 128) {
    checkStateCount[2]++;
    row++;
  }
  if (row == maxRows) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (row < maxRows && img.at<uchar>(row, centerCol) >= 128 &&
         checkStateCount[3] < centerCount) {
    checkStateCount[3]++;
    row++;
  }
  if (row == maxRows || checkStateCount[3] >= totalCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (row < maxRows && img.at<uchar>(row, centerCol) < 128 &&
         checkStateCount[4] < centerCount) {
    checkStateCount[4]++;
    row++;
  }
  if (row == maxRows || checkStateCount[4] >= centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  int totalCheckcount = 0;
  for (auto& count : checkStateCount) {
    totalCheckcount += count;
  }

  if (5 * abs(totalCheckcount - totalCount) >= 2 * totalCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  double centerRow = columnCenterEstimate(checkStateCount, row);
  if (checkRatio(checkStateCount)) {
    return centerRow;
  } else {
    return std::numeric_limits<float>::quiet_NaN();
  }
}

/**
 * @brief Find the center point in the horizontal direction
 *
 * @param img input image
 * @param centerRow Center of the finder pattern in the vertical direction
 * @param startCol Column number of the pixel in the image
 * @param centerCol Estimate of center pixel in the column
 * @param centerCount Number of pixels in the length of center square
 * @param totalCount Number of pixels in the length of the whole finder pattern
 *
 * @return The center of the finder pattern in the horizontal direction
 */
float QReader::checkHorizontal(const cv::Mat& img, int centerRow, int startCol,
                               int centerCount, int totalCount) {
  int maxCols = img.cols;
  std::vector<int> stateCount(5, 0);
  int col = startCol;
  const uchar* ptr = img.ptr<uchar>(centerRow);
  // traverse towards the left of finder pattern
  while (col >= 0 && ptr[col] < 128) {
    stateCount[2]++;
    col--;
  }
  if (col < 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (col >= 0 && ptr[col] >= 128 && stateCount[1] < centerCount) {
    stateCount[1]++;
    col--;
  }
  if (col < 0 || stateCount[1] == centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (col >= 0 && ptr[col] < 128 && stateCount[0] < centerCount) {
    stateCount[0]++;
    col--;
  }
  if (col < 0 || stateCount[0] == centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  col = startCol + 1;
  // traverse towards the right of finder pattern
  while (col < maxCols && ptr[col] < 128) {
    stateCount[2]++;
    col++;
  }
  if (col == maxCols) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (col < maxCols && ptr[col] >= 128 && stateCount[3] < centerCount) {
    stateCount[3]++;
    col++;
  }
  if (col == maxCols || stateCount[3] == centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  while (col < maxCols && ptr[col] < 128 && stateCount[4] < centerCount) {
    stateCount[4]++;
    col++;
  }
  if (col == maxCols || stateCount[4] == centerCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  int totalCheckcount = 0;
  for (auto& count : stateCount) {
    totalCheckcount += count;
  }

  if (5 * abs(totalCount - totalCheckcount) >= totalCount) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  double centerCol = columnCenterEstimate(stateCount, col);
  if (checkRatio(stateCount)) {
    return centerCol;
  } else {
    return std::numeric_limits<float>::quiet_NaN();
  }
}

/**
 * @brief Find the center point in the horizontal direction
 *
 * @param img input image
 * @param centerRow Center of the finder pattern in the vertical direction
 * @param centerCol Center of the finder pattern in the horizontal direction
 * @param maxCount Number of pixels in the length of center square
 * @param totalCount Number of pixels in the length of the whole finder pattern
 *
 * @return The center of the finder pattern in the horizontal direction
 */
bool QReader::checkDiagonal(const cv::Mat& img, float centerRow,
                            float centerCol, int maxCount, int totalCount) {
  std::vector<int> stateCount(5, 0);
  int col = centerCol;
  int row = centerRow;
  // traverse towards the top left corner
  while (col >= 0 && row >= 0 && img.at<uchar>(row, col) < 128) {
    stateCount[2]++;
    row--;
    col--;
  }
  if (row < 0 || col < 0) {
    return false;
  }
  while (row >= 0 && col >= 0 && img.at<uchar>(row, col) >= 128 &&
         stateCount[1] <= maxCount) {
    stateCount[1]++;
    row--;
    col--;
  }
  if (row < 0 || col < 0 || stateCount[1] > maxCount) {
    return false;
  }
  while (row >= 0 && col >= 0 && img.at<uchar>(row, col) < 128 &&
         stateCount[0] <= maxCount) {
    stateCount[0]++;
    row--;
    col--;
  }
  if (stateCount[0] > maxCount) {
    return false;
  }
  int maxCols = img.cols;
  int maxRows = img.rows;
  col = centerCol + 1;
  row = centerRow + 1;
  // traverse towards the bottom right corner
  while (row < maxRows && col < maxCols && img.at<uchar>(row, col) < 128) {
    stateCount[2]++;
    row++;
    col++;
  }
  if (row >= maxRows || col >= maxCols) {
    return false;
  }
  while (row < maxRows && col < maxCols && img.at<uchar>(row, col) >= 128 &&
         stateCount[3] < maxCount) {
    stateCount[3]++;
    row++;
    col++;
  }
  if (row >= maxRows || col >= maxCols || stateCount[3] > maxCount) {
    return false;
  }
  while (row < maxRows && col < maxCols && img.at<uchar>(row, col) < 128 &&
         stateCount[4] < maxCount) {
    stateCount[4]++;
    row++;
    col++;
  }
  if (row >= maxRows || col >= maxCols || stateCount[4] > maxCount) {
    return false;
  }
  int newTotalcount = 0;
  for (auto& count : stateCount) {
    newTotalcount += count;
  }
  bool possible = (abs(totalCount - newTotalcount) < 2 * totalCount) &&
                  checkRatio(stateCount);
  // return true if ratio holds
  return possible;
}

/**
 * @brief Calculate center center pixel in the column of finder pattern
 * @param stateCount Vector containing the number of pixels with black or white
 * pixels
 * @param end Last column
 * @return Estimate of the center of the finder pattern in the column
 */
float QReader::columnCenterEstimate(std::vector<int> stateCount, int end) {
  float mid = (end - stateCount[4] - stateCount[3]) - stateCount[2] / 2.0f;
  return mid;
}

cv::Mat QReader::warpToCode(cv::Mat) {}
std::vector<uint8_t> QReader::extractBits(cv::Mat) {}

std::vector<char> QReader::decodeBits(std::vector<uint8_t>) {}
