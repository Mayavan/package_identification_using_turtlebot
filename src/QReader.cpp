/**
 * @file QReader.cpp
 * @brief File which has the definitions of QReader class methods
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

#include <cmath>
#include "package_identification_using_turtlebot/PathPlanner.hpp"

QReader::QReader() : it(nh) {
  std::string str = "unknown";
  std::vector<uint8_t> bytes(str.begin(), str.end());
  ROS_INFO("Inside QReader Constructor");
  imgSub = it.subscribe("/camera/rgb/image_raw", 1, &QReader::imageCb, this);
  // cv::namedWindow("Image Window");
  //  cv::namedWindow("Output Window");
  //  cv::namedWindow("Resized Window");
}

QReader::~QReader() {
  // cv::destroyWindow("Image Window");
  //  cv::destroyWindow("Output Window");
  //  cv::destroyWindow("Resized Window");
}

std::vector<uint8_t> QReader::returnBytes() { return bytes; }

void QReader::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ROS_INFO("Inside image callback");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img = cvPtr->image;
  img = processFrame();

  // cv::imshow("Image Window", img);
  // cv::waitKey(3);

  // Decode the QR code in the image
  bytes = decodeQR();
}

cv::Mat QReader::processFrame() {
  cv::Mat temp;
  cv::cvtColor(img, temp, CV_BGR2HSV);
  inRange(temp, cv::Scalar(0, 0, 200, 0), cv::Scalar(180, 255, 255, 0), temp);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
               cv::Point(0, 0));
  std::vector<std::vector<cv::Point> > contours_poly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());
  cv::Mat roi(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat dst = cv::Mat::zeros(roi.size(), CV_32FC1);
  cv::Mat dst_norm, dst_norm_scaled;
  cv::Mat result;
  std::vector<cv::Point2f> src;
  src.clear();

  // Detect corners of the QR code for appropriate warping
  for (int i = 0; i < contours.size(); i++) {
    if (contourArea(contours[i]) > 40000) {
      drawContours(roi, contours, i, cv::Scalar(255, 255, 255), CV_FILLED);
      cv::cvtColor(roi, roi, CV_BGR2GRAY);
      cornerHarris(roi, dst, 2, 3, 0.06);
      normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
      convertScaleAbs(dst_norm, dst_norm_scaled);
      for (int x = 0; x < dst_norm.rows; x++) {
        for (int y = 0; y < dst_norm.cols; y++) {
          if (static_cast<int>(dst_norm.at<float>(x, y)) > 200) {
            src.push_back(cv::Point2f(y, x));
          }
        }
      }
    }
  }
  ROS_INFO("Number of corner points detected: %d",
           static_cast<int>(src.size()));
  std::vector<cv::Point2f> srcPoints;
  cv::Point2f topLeft, topRight, bottomRight, bottomLeft;

  if (src.size() == 4) {
    std::sort(
        src.begin(), src.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

    if (src.at(0).x + src.at(0).y < src.at(1).x + src.at(1).y) {
      topLeft = src.at(0);
      bottomLeft = src.at(1);
    } else {
      topLeft = src.at(1);
      bottomLeft = src.at(0);
    }
    if (src.at(2).x + src.at(2).y < src.at(3).x + src.at(3).y) {
      topRight = src.at(2);
      bottomRight = src.at(3);
    } else {
      topRight = src.at(3);
      bottomRight = src.at(2);
    }
    srcPoints = {topLeft, topRight, bottomLeft, bottomRight};

    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(0.0, 0.0), cv::Point2f(199.0, 0.0), cv::Point2f(0.0, 199.0),
        cv::Point2f(199.0, 199.0)};
    cv::Mat transform = getPerspectiveTransform(srcPoints, dstPoints);
    warpPerspective(img, img, transform, cv::Size(200, 200), CV_INTER_LINEAR);
  }
  return img;
}

std::vector<uint8_t> QReader::decodeQR() {
  ros::Duration(3.0).sleep();
  possibleCenters.clear();
  estimatedModuleSize.clear();
  ROS_INFO_STREAM("Decoding Image");
  // Convert image to black and white
  cv::cvtColor(img, img, CV_BGR2HSV);
  inRange(img, cv::Scalar(0, 0, 200, 0), cv::Scalar(180, 255, 255, 0), img);
  //  cv::imshow("Image Window", img);
  //  cv::waitKey(3);
  ROS_INFO_STREAM("Checking QR code existence");
  bool found = checkQCodeExists(img);
  std::vector<uint8_t> bytes = {0x75, 0x6E, 0x6B, 0x6E, 0x6F, 0x77, 0x6E};
  if (found) {
    ROS_INFO_STREAM("QR Code exists");
    cv::Mat QR = warpToCode(img);
    std::vector<std::vector<bool> > bitMatrix = extractBits(QR);

    unmask(bitMatrix);

    bytes = decodeArray(bitMatrix);
  }
  return bytes;
}

bool QReader::checkQCodeExists(cv::Mat& img) {
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
    ROS_INFO_STREAM("Cannot find three centers");
    return false;
  } else {
    return true;
  }
}

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

float QReader::columnCenterEstimate(std::vector<int> stateCount, int end) {
  float mid = (end - stateCount[4] - stateCount[3]) - stateCount[2] / 2.0f;
  return mid;
}

cv::Mat QReader::warpToCode(cv::Mat& img) {
  // Sort centers based on positions to find the corners
  std::sort(
      possibleCenters.begin(), possibleCenters.end(),
      [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });
  cv::Point2f topRight = possibleCenters[2];

  std::sort(
      possibleCenters.begin(), possibleCenters.end(),
      [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

  cv::Point2f bottomLeft = possibleCenters[2];
  cv::Point2f topLeft = possibleCenters[0].x < possibleCenters[1].x
                            ? possibleCenters[0]
                            : possibleCenters[1];

  cv::Point2f bottomRight = topRight - topLeft + bottomLeft;

  std::vector<cv::Point2f> corners;
  corners.push_back(topLeft);
  corners.push_back(topRight);
  corners.push_back(bottomLeft);
  corners.push_back(bottomRight);

  int sum_of_elems = 0;
  for (auto& n : estimatedModuleSize) sum_of_elems += n;

  int moduleSize = sum_of_elems / estimatedModuleSize.size();
  float factor = 3.5f * moduleSize;
  int dimensionQR = dimension * moduleSize;

  std::vector<cv::Point2f> src;
  src.push_back(cv::Point2f(factor, factor));
  src.push_back(cv::Point2f(dimensionQR - factor, factor));
  src.push_back(cv::Point2f(factor, dimensionQR - factor));
  src.push_back(cv::Point2f(dimensionQR - factor, dimensionQR - factor));

  // Warp the image to only the QR code
  cv::Mat transform = getPerspectiveTransform(corners, src);
  cv::Mat output;
  warpPerspective(img, output, transform, cv::Size(dimensionQR, dimensionQR));
  cv::adaptiveThreshold(output, output, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                        CV_THRESH_BINARY, 51, 0);
  //  cv::imshow("Output Window", output);
  //  cv::waitKey(3);

  cv::resize(output, output, cv::Size(dimension, dimension), CV_INTER_LANCZOS4);
  //  cv::imshow("Resized Window", output);
  //  cv::waitKey(3);

  return output;
}

std::vector<std::vector<bool> > QReader::extractBits(cv::Mat& marker) {
  const int width = marker.cols;
  const int height = marker.rows;
  std::vector<std::vector<bool> > code;
  for (int y = 0; y < height; y++) {
    const uchar* ptr = marker.ptr<uchar>(y);
    std::vector<bool> row;
    for (int x = 0; x < width; x++) {
      if (ptr[x] > 128) {
        row.push_back(true);
      } else {
        row.push_back(false);
      }
    }

    code.push_back(row);
  }
  return code;
}

void QReader::unmask(std::vector<std::vector<bool> >& code) {
  // Find the mask used by accessing (8,2),(8,3) & (8,4)
  int mask;
  mask = code[8][2] ? 4 : 0;
  mask += code[8][3] ? 2 : 0;
  mask += code[8][4] ? 1 : 0;

  for (int i = 0; i < 21; i++) {
    for (int j = 0; j < 21; j++) {
      switch (mask) {
        case 0:
          if (j % 3 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 1:
          if ((i + j) % 3 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 2:
          if ((i + j) % 2 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 3:
          if (i % 2 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 4:
          if (((i * j) % 3 + i * j) % 2 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 5:
          if (((i * j) % 3 + i + j) % 2 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 6:
          if ((i / 2 + j / 3) % 2 != 0) {
            code[i][j] = !code[i][j];
          }
          break;
        case 7:
          if (((i * j) % 2 + (i * j) % 3) != 0) {
            code[i][j] = !code[i][j];
          }
          break;
      }
    }
  }
}

std::vector<uint8_t> QReader::decodeArray(
    const std::vector<std::vector<bool> >& code) {
  std::vector<bool> bitstream;
  // Converting the code into a bit stream

  // Get transpose of the code matrix
  std::vector<std::vector<bool> > transpose;
  for (int i = 0; i < 21; i++) {
    std::vector<bool> column;
    for (auto row : code) column.push_back(*(row.begin() + i));
    transpose.push_back(column);
  }
  auto index = transpose.size() - 1;
  // loop every consecutive columns from the last column
  for (auto col = transpose.rbegin(); col != (transpose.rend() - 11);
       col += 2, index -= 2) {
    auto colPrevious = col + 1;
    // If half of column index is even, traverse up from bottom
    if ((index / 2) % 2 == 0) {
      auto leftBitIterator = (*colPrevious).rbegin();
      for (auto rightBitIterator = (*col).rbegin();
           rightBitIterator != ((*col).rend() - 9); rightBitIterator++) {
        bitstream.push_back(*rightBitIterator);
        bitstream.push_back(*leftBitIterator);
        leftBitIterator++;
      }
    } else {  // If half of column index is odd, traverse down from top
      auto leftBitIterator = (*colPrevious).begin() + 9;
      for (auto rightBitIterator = (*col).begin() + 9;
           rightBitIterator != (*col).end(); rightBitIterator++) {
        bitstream.push_back(*rightBitIterator);
        bitstream.push_back(*leftBitIterator);
        leftBitIterator++;
      }
    }
  }

  // Bitstream iterator for decoding
  auto currentBit = bitstream.begin();
  int encoding = getNumberValue(currentBit, 4);

  ROS_INFO_STREAM("Encoding Used in QR code:" << encoding);
  if (encoding != 4) {
    ROS_INFO_STREAM("Encoding not supported");
    std::vector<uint8_t> unknown(7, 0);
    // Return decoded value to unknown
    unknown = {0x75, 0x6E, 0x6B, 0x6E, 0x6F, 0x77, 0x6E};
    return unknown;
  }
  int length = getNumberValue(currentBit, 8);
  ROS_INFO_STREAM("Length of data in QR code:" << length);

  std::vector<uint8_t> bytes;
  while (length != 0) {
    bytes.push_back(getNumberValue(currentBit, 8));
    length--;
  }
  return bytes;
}

uint8_t QReader::getNumberValue(std::vector<bool>::iterator& currentBit,
                                unsigned int length) {
  uint8_t value = 0;

  for (int i = length - 1; i >= 0; i--) {
    value += (*currentBit) * std::pow(2, i);
    currentBit++;
  }
  return value;
}

void QReader::setImage(cv::Mat image) { img = image; }
