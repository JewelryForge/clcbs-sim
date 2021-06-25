#include <string>

namespace Constants {

static double MINIMUM_ROTATION_RADIUS = 1.5;
static double MAXIMUM_LINEAR_VELOCITY = 4.0;
static double WHEEL_RADIUS = 0.5;
static double CAR_WIDTH = 2.0;
static double CAR_LENGTH = 3.0;
static double MAP_SIZE_X = 50.0;
static double MAP_SIZE_Y = 50.0;

void loadFromRosParam();
void loadFromYamlFile(std::string);
}

template <typename T>
int sign(T n) {
  if (n == 0) return 0;
  if (n < 0) return -1;
  return 1;
}