#include "CarModel.h"
#include <iostream>
#include <tuple>

CarModel::CarModel() : CarModel(Constants::MINIMUM_ROTATION_RADIUS) {}

CarModel::CarModel(double rotation_radius) : vx_max_(Constants::MAXIMUM_LINEAR_VELOCITY),
                                             rot_radius_(rotation_radius), width_(Constants::CAR_WIDTH),
                                             ort_(0.), thr_(0.) {}

void CarModel::setThr(double thr) {
  thr_ = std::max(std::min(thr, 1.0), -1.0);
}
void CarModel::setVx(double vx) {
  setThr(vx / vx_max_);
}

void CarModel::setOrt(double ort) {
  ort_ = std::max(std::min(ort, 1.0), -1.0);
}

void CarModel::setVw(double vw) {
  vw == 0 ? setOrt(0.) : setRad(vx() / vw);
}

void CarModel::setRad(double radius) {
  if (radius == 0.0) setOrt(1);
  else if (radius == -0.0) setOrt(-1);
  setOrt(rot_radius_ / radius);
}

void CarModel::reset() {
  setThr(0);
  setOrt(0);
}

double CarModel::vx() const {
  return thr_ * vx_max_;
}

double CarModel::vw() const {
  return ort_ * vx() / rot_radius_;
}

std::pair<double, double> CarModel::getVelocity() const {
  return {vx() - vw() * width_ / 2, vx() + vw() * width_ / 2};
}

