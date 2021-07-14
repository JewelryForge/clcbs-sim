#include <string>
#include "ros/ros.h"
namespace Constants {

static double MAXIMUM_TURNING_ANGLE = 30.0 / 180 * 3.141592653589;
static double MAXIMUM_LINEAR_VELOCITY = 1.2;
static double WHEEL_RADIUS = 0.164;
static double WHEEL_BASE = 0.65;
static double CAR_WIDTH = 0.718;
static double CAR_LENGTH = 0.980;
static double MAP_SIZE_X = 10.0;
static double MAP_SIZE_Y = 10.0;

static void loadFromRosParam() {
  ros::NodeHandle nh;
  std::vector<double> map_size;
  nh.getParam("/config/map/dimensions", map_size);
  MAP_SIZE_X = map_size[0];
  MAP_SIZE_Y = map_size[1];
}
static void loadFromYamlFile(std::string) {}
}

template <typename T>
int sign(T n) {
  if (n == 0) return 0;
  if (n < 0) return -1;
  return 1;
}