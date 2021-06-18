#include "CarModel.h"
#include <iostream>
#include <tuple>

CarModel::CarModel(double min_rot_radius) : step_(1.0), vx_max_(10.0), rot_radius_(min_rot_radius), width_(1.0) {}

void CarModel::setThr(double thr) {
  if (thr > thr_ + vx_max_ * step_) thr_ += step_;
  else if (thr < thr_ - vx_max_ * step_) thr_ -= step_;
  else thr_ = thr;
  thr_ = std::max(std::min(thr_, 1.0), -1.0);
}

void CarModel::setOrt(double ort) {
  if (ort > ort_ * step_) ort_ += step_;
  else if (ort < ort_ * step_) ort_ -= step_;
  else ort_ = ort;
  ort_ = std::max(std::min(ort_, 1.0), -1.0);
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
