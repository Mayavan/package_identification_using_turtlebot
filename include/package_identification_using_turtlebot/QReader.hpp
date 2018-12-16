/**
 * @file QReader.hpp
 * @brief File that has declarations for the QReader class
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
#ifndef INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_QREADER_HPP_
#define INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_QREADER_HPP_

#include <image_transport/image_transport.h>
#include <vector>
#include <opencv2/opencv.hpp>

/**
 * @brief Class QReader has methods to get the image and decode the QR code in
 * the image to extract data of package ID.
 */
class QReader {
 public:
  /**
   * @brief  Constructs a object
   */
  QReader();

  /**
   * @brief Destroys the object
   */
  ~QReader();

  /**
   * @brief A function to get the raw image, from the camera and convert it to
   * cv::Mat format using cv bridge.
   *
   * @param The message to subcribe to get the image from the turtlebot camera.
   *
   * @return None
   */
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  cv::Mat processFrame();
  /**
   * @brief A function to get the raw image, find the QR code, unmask the QR
   * code and decode the QR code in the image to extract data of package ID.
   *
   * @param none
   *
   * @return Bytes containing the data in QR code in UTF-8 format
   */
  std::vector<uint8_t> decodeQR();

  /**
   * @brief Function to get the extracted data array.
   *
   * @param none
   *
   * @return array of bytes of data stored in the QR code
   */
  std::vector<uint8_t> returnBytes();

  /**
   * @brief Sets the value for the private value img
   *
   * @param cv::Mat to set as value of img.
   *
   * @return none
   */
  void setImage(cv::Mat image);

 private:
  /**
   * Subscriber to get the raw image from camera
   */
  image_transport::Subscriber imgSub;
  /**
   * @brief The decoded bytes from the Qr code
   */
  std::vector<uint8_t> bytes;
  /**
   * @brief Node handle to manage the ros node
   */
  ros::NodeHandle nh;
  /**
   * @brief Image of the QR code
   */
  cv::Mat img;
  /**
   * @brief Estimated centers of a finder pattern
   */
  std::vector<cv::Point2f> possibleCenters;
  /**
   * @brief Number of pixels in each bit of the QR code
   */
  std::vector<float> estimatedModuleSize;
  /**
   * @brief Length/Width of QR code
   */
  int dimension = 21;
  /**
   * @brief Image Transport object
   */
  image_transport::ImageTransport it;

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
  bool checkQCodeExists(cv::Mat&);
  /**
   * @brief checks if the number of pixels in each states are in the ratio
   * 1:1:3:1:1
   * @param stateCount vector containing the number of pixels with black or
   * white pixels
   * @return true if the number of pixels in each states are in the ratio
   * 1:1:3:1:1 or false
   */
  bool checkRatio(std::vector<int>);
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
  bool isFinderPattern(const cv::Mat&, std::vector<int>, int, int);
  /**
   * @brief Find the center point in the vertical direction
   *
   * @param img input image
   * @param startRow Row number of the pixel in the image
   * @param centerCol Estimate of center pixel in the column
   * @param centerCount Number of pixels in the length of center square
   * @param totalCount Number of pixels in the length of the whole finder
   * pattern
   *
   * @return The center of the finder pattern in the vertical direction
   */
  float checkVertical(const cv::Mat&, int, int, int, int);
  /**
   * @brief Find the center point in the horizontal direction
   *
   * @param img input image
   * @param centerRow Center of the finder pattern in the vertical direction
   * @param startCol Column number of the pixel in the image
   * @param centerCol Estimate of center pixel in the column
   * @param centerCount Number of pixels in the length of center square
   * @param totalCount Number of pixels in the length of the whole finder
   * pattern
   *
   * @return The center of the finder pattern in the horizontal direction
   */
  float checkHorizontal(const cv::Mat&, int, int, int, int);
  /**
   * @brief Find the center point in the horizontal direction
   *
   * @param img input image
   * @param centerRow Center of the finder pattern in the vertical direction
   * @param centerCol Center of the finder pattern in the horizontal direction
   * @param maxCount Number of pixels in the length of center square
   * @param totalCount Number of pixels in the length of the whole finder
   * pattern
   *
   * @return The center of the finder pattern in the horizontal direction
   */
  bool checkDiagonal(const cv::Mat&, float, float, int, int);
  /**
   * @brief Calculate center center pixel in the column of finder pattern
   * @param stateCount Vector containing the number of pixels with black or
   * white pixels
   * @param end Last column
   * @return Estimate of the center of the finder pattern in the column
   */
  float columnCenterEstimate(std::vector<int>, int);

  /**
   * @brief Warps the QR code to a flat perspective of only the QR Code.
   *
   * @param img The image to be warped.
   *
   * @return Returns the warped QR code.
   */
  cv::Mat warpToCode(cv::Mat&);
  /**
   * @brief Warped image of the QR code.
   *
   * @param marker The image from which bits have to be extracted.
   *
   * @return Returns the warped QR code.
   */
  std::vector<std::vector<bool> > extractBits(cv::Mat&);
  /**
   * @brief Finds the mask used in the code and unmask the QR code
   * @param code The masked QR code in binary form
   * @return none
   */
  void unmask(std::vector<std::vector<bool> >&);

  /**
   * @brief Decodes the data from the unmasked QR code array
   * @param Code The unmasked QR code in binary form
   * @return Byte array of the data stored in QR code
   */
  std::vector<uint8_t> decodeArray(const std::vector<std::vector<bool> >&);

  /**
   * @brief Converts the bitstream of given length starting from the given
   * iterator to integer and also advances the iterator past the used bits.
   *
   * @param currentBit Iterator of the starting bit.
   *
   * @param length number of bit to use from the bit stream
   *
   * @return Integer value of the bitstream
   */
  uint8_t getNumberValue(std::vector<bool>::iterator&, unsigned int);
};

#endif  // INCLUDE_PACKAGE_IDENTIFICATION_USING_TURTLEBOT_QREADER_HPP_
