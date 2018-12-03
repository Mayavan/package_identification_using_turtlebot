#include "package_identification_using_turtlebot/QReader.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "QBot");

  ros::NodeHandle node;

  // Initialization
  QReader reader;
  reader.decodeQR();

  return 0;
}
