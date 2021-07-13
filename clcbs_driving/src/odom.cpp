#include "WheelOdometer.h"

int main(int argc, char** argv) {
  // An instance to use odometer
  ros::init(argc, argv, "ODOM_TEST");
  ros::NodeHandle nh;
  WheelOdometer odom("hunter");
  ros::spin();
}