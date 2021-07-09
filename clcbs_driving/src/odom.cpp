#include "WheelOdometer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "CPP_TEST");
  ros::NodeHandle nh;
  WheelOdometer odom("hunter");
  ros::spin();
}