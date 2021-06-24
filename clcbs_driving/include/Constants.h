#include <string>


namespace Constants {

static double MINIMUM_ROTATION_RADIUS = 1.0;
static double MAXIMUM_LINEAR_VELOCITY = 4.0;
static double WHEEL_RADIUS = 0.5;
static double CAR_WIDTH = 1.0;
static double CAR_LENGTH = 2.0;
static double MAP_SIZE_X = 50.0;
static double MAP_SIZE_Y = 50.0;

void loadFromRosParam();
void loadFromYamlFile(std::string);
}