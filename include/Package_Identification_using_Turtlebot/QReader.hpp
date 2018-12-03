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
  bool isFinderPattern(const cv::Mat&, std::vector<int>, int, int);
  float checkVertical(const cv::Mat&, int, int, int, int);
  float checkHorizontal(const cv::Mat&, int, int, int, int);
  bool checkDiagonal(const cv::Mat&, float, float, int, int);

  float columnCenterEstimate(std::vector<int>, int);
  cv::Mat warpToCode(cv::Mat);
  std::vector<uint8_t> extractBits(cv::Mat);
  std::vector<char> decodeBits(std::vector<uint8_t>);
};
