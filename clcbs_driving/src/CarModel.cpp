#include "CarModel.h"
#include <iostream>
#include <tuple>

CarModel::CarModel() : step_(0.1), vx_max_(10.0), rot_radius_(3.0) {}

void CarModel::restrain() {
  vx_ = std::max(std::min(vx_, vx_max_), -vx_max_);
  vw_ = std::max(std::min(vw_, vw_max_), -vw_max_);
}

void CarModel::acc() {
  vx_ += vx_max_ * step_;
  restrain();
  updateVWMax();
}

void CarModel::dec() {
  vx_ -= vx_max_ * step_;
  restrain();
  updateVWMax();
}

void CarModel::updateVWMax() {
  vw_max_ = std::abs(vx_) / rot_radius_;
}

void CarModel::ltn() {
  vw_ += vw_max_ * step_;
  restrain();
}

void CarModel::rtn() {
  vw_ -= vw_max_ * step_;
  restrain();
}

void CarModel::set_vx(double vx) {
  if (vx > vx_ + vx_max_ * step_) {
    acc();
  } else if (vx < vx_ - vx_max_ * step_) {
    dec();
  } else {
    vx_ = vx;
    updateVWMax();
    restrain();
  }
}

void CarModel::set_vw(double vw) {
  if (vw > vw_ + vw_max_ * step_) {
    ltn();
  } else if (vw < vw_ - vw_max_ * step_) {
    rtn();
  } else {
    vw_ = vw;
    restrain();
  }

}

void CarModel::reset() {
  std::tie(vx_, vw_, vw_max_) = std::make_tuple(0.0, 0.0, 0.0);
}


