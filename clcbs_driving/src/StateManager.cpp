#include "StateManager.h"
#include <cassert>
#include <iomanip>
#include <utility>
#include <cmath>

State::State(double x, double y, Angle yaw) : x(x), y(y), yaw(yaw) {}
State::State(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
State operator+(const State &s1, const State &s2) {
  return {s1.x + s2.x, s1.y + s2.y, s1.yaw + s2.yaw};
}
State operator-(const State &s1, const State &s2) {
  return {s1.x - s2.x, s1.y - s2.y, s1.yaw - s2.yaw};
}
State operator*(const State &s1, double p) {
  return {s1.x * p, s1.y * p, s1.yaw * p};
}
State operator*(double p, const State &s1) {
  return s1 * p;
}
State operator/(const State &s1, double p) {
  return s1 * (1 / p);
}
std::ostream &operator<<(std::ostream &os, const State &s) {
  return os << std::setprecision(2) << "ST<" << s.x << ' ' << s.y << ' ' << s.yaw << '>';
}
State State::interp(const State &s1, const State &s2, double ratio) {
  return {s1.x + (s2.x - s1.x) * ratio, s1.y + (s2.y - s1.y) * ratio, s1.yaw + (s2.yaw - s1.yaw) * ratio};
}
double State::norm() const {
  return std::hypot(x, y);
}
double State::diff() const {
  return yaw == 0.? norm() : norm() / (2 * sin(yaw / 2.)) * yaw;
}
std::tuple<double, double, Angle> State::asTuple() const {
  return {x, y, yaw};
}
Eigen::Vector2d State::asVector2() const {
  return {x, y};
}
Eigen::Vector3d State::asVector3() const {
  return {x, y, 0};
}
Eigen::Vector2d State::oritUnit2() const {
  return {cos(yaw), sin(yaw)};
}
Eigen::Vector3d State::oritUnit3() const {
  return {cos(yaw), sin(yaw), 0};
}
StateManager::StateManager(std::vector<std::pair<double, State>> states)
    : states_(std::move(states)), align([](const State &s) { return s; }) {
  assert(!states_.empty() and states_.front().first == 0);
}
void StateManager::setAlignmentParam(double x, double y) {
  align = [=](const State &s) { return State(s.x + x, s.y + y, s.yaw); };
}
State StateManager::operator()(double t) {
  return align(getState(t));
}

State StateManager::getState(double t) {
  if (t <= 0) {
    return states_.front().second;
  } else if (t <= states_.back().first) {
    for (auto iter = states_.begin(); iter != states_.end(); ++iter) {
      if (iter->first < t) continue;
      auto s_p = iter - 1, s_n = iter;
      double ratio = (t - s_p->first) / (s_n->first - s_p->first);
      return State::interp(s_p->second, s_n->second, ratio);
    }
  }
  finished = true;
  return states_.back().second;
}

std::tuple<double, double> StateManager::getInstruction(double t) {
  if (t <= 0) {
    return {0, 0};
  } else if (t <= states_.back().first) {
    for (auto iter = states_.begin(); iter != states_.end(); ++iter) {
      if (iter->first < t) continue;
      auto s_p = iter - 1, s_n = iter;
      auto diff_state = s_n->second - s_p->second;
      double d_yaw = diff_state.yaw, v;
      if (d_yaw == 0.) v = diff_state.norm() / (s_n->first - s_p->first);
      else v = diff_state.norm() / (s_n->first - s_p->first) * d_yaw / (2 * sin(d_yaw / 2.));
      return {v, v * d_yaw};
    }
  }
  finished = true;
  return {0, 0};
}
