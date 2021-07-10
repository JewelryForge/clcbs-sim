#include <string>

namespace Constants {

static double MAXIMUM_TURNING_ANGLE = 30.0 / 180 * 3.141592653589;
static double MAXIMUM_LINEAR_VELOCITY = 1.2;
static double WHEEL_RADIUS = 0.164;
static double WHEEL_BASE = 0.65;
static double CAR_WIDTH = 0.718;
static double CAR_LENGTH = 0.980;
static double MAP_SIZE_X = 10.0;
static double MAP_SIZE_Y = 10.0;

void loadFromRosParam();
void loadFromYamlFile(std::string);
}

template <typename T>
int sign(T n) {
  if (n == 0) return 0;
  if (n < 0) return -1;
  return 1;
}