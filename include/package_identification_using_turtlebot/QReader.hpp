#include <math.h>
#include <ros/console.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class QReader {
 public:
  QReader();
  ~QReader();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  std::vector<uint8_t> decodeQR();
  image_transport::Subscriber imgSub;

 private:
  ros::NodeHandle nh;
  cv::Mat img;
  std::vector<cv::Point2f> possibleCenters;
  std::vector<float> estimatedModuleSize;
  int dimension = 21;
  image_transport::ImageTransport it;


  cv::Mat captureImage();
  bool checkQCodeExists(cv::Mat&);
  bool checkRatio(std::vector<int>);
  bool isFinderPattern(const cv::Mat&, std::vector<int>, int, int);
  float checkVertical(const cv::Mat&, int, int, int, int);
  float checkHorizontal(const cv::Mat&, int, int, int, int);
  bool checkDiagonal(const cv::Mat&, float, float, int, int);

  float columnCenterEstimate(std::vector<int>, int);
  cv::Mat warpToCode(cv::Mat&);
  std::vector<std::vector<bool> > extractBits(cv::Mat&);
  std::vector<char> decodeBits(std::vector<uint8_t>);
  void unmask(std::vector<std::vector<bool> >&);
  std::vector<uint8_t> decodeArray(const std::vector<std::vector<bool> >&);
  uint8_t getNumberValue(std::vector<bool>::iterator&, unsigned int);
};
