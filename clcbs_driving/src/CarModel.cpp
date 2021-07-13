#include "CarModel.h"
#include <tuple>
#include <cmath>

CarModel::CarModel() : CarModel(Constants::CAR_WIDTH, Constants::WHEEL_BASE,
                                Constants::MAXIMUM_LINEAR_VELOCITY,
                                Constants::MAXIMUM_TURNING_ANGLE) {}

CarModel::CarModel(double car_width, double wheel_base, double max_linear_velocity, double max_turning_angle) :
    car_width_(car_width),
    wheel_base_(wheel_base),
    max_linear_(max_linear_velocity),
    max_turning_(max_turning_angle),
    min_rotation_radius_(wheel_base / std::tan(max_turning_angle) + car_width / 2),
    linear_(0.), turning_(0.) {
}

void CarModel::setLinearVelocity(double vx) {
  linear_ = std::max(std::min(vx, max_linear_), -max_linear_);
}

void CarModel::setTurningAngle(double turning) {
  turning_ = std::max(std::min(turning, max_turning_), -max_turning_);
}

void CarModel::setAngularVelocity(double vw) {
  if (vw == 0.0) {
    setTurningAngle(0.0);
    return;
  }
  double radius = std::max(getLinearVelocity() / std::abs(vw), min_rotation_radius_);
  double turning_angle = std::atan2(wheel_base_, radius - car_width_ / 2);
  setTurningAngle(sign(vw) * sign(linear_) * turning_angle);
}

void CarModel::reset() {
  setLinearVelocity(0);
  setTurningAngle(0);
}

double CarModel::getLinearVelocity() const {
  return linear_;
}

double CarModel::getTurningAngle() const {
  return turning_;
}

double CarModel::getAngularVelocity() const {
  return sign(turning_) * getLinearVelocity() / (wheel_base_ / std::tan(std::abs(turning_)) + car_width_ / 2);
}

std::pair<double, double> CarModel::getWheelVelocity() const {
  double vx = getLinearVelocity(), vw = getAngularVelocity();
  return {vx - vw * car_width_ / 2, vx + vw * car_width_ / 2};
}

