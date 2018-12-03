#include <math.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class QReader {
  QReader();
  ~QReader();
  std::vector<uint8_t> decodeQR(cv::Mat);
  cv::Mat captureImage();
  bool checkQCodeExists(cv::Mat&);

 private:
  std::vector<cv::Point2f> possibleCenters;
  std::vector<float> estimatedModuleSize;

  bool checkRatio(std::vector<int>);
  cv::Mat warpToCode(cv::Mat);
  std::vector<uint8_t> extractBits(cv::Mat);
  std::vector<char> decodeBits(std::vector<uint8_t>);
};
