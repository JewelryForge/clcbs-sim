#include "StateManager.h"
#include <cassert>
#include <iomanip>
#include <utility>
#include <cmath>
#include "Constants.h"

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
  return yaw == 0. ? norm() : norm() / (2 * sin(yaw / 2.)) * yaw;
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
StateManager::StateManager(const std::vector<std::pair<double, State>> &states)
    : align([](const State &s) { return s; }) {
  assert(states.size() >= 2 and states.front().first == 0);
  transitions_.reserve(states.size());
  for (auto curr = states.begin(), next = curr + 1;; curr = next, ++next) {
    if (next == states.end()) {
      transitions_.emplace_back(curr->first, Transition(curr->second, {0.0, 0.0}, Move::STOP));
      break;
    }
    Transition t;
    t.state = curr->second;
    double dt = next->first - curr->first;
    auto diff_state = next->second - curr->second;
    if (diff_state.norm() == 0.0) {
      t.move = Move::STOP;
      t.x = t.v = {0.0, 0.0};
    } else if (diff_state.asVector2().dot(curr->second.oritUnit2()) > 0) {
      t.move = Move::FORWARD;
      t.x.first = t.x.second = diff_state.diff();
    } else {
      t.move = Move::BACK;
      t.x.first = t.x.second = diff_state.diff() * -1;
    }
    if (diff_state.yaw != 0) {
      double diff_x = diff_state.yaw * (Constants::CAR_WIDTH / 2);
      if (diff_state.yaw > 0) {
        t.move |= Move::LEFT_TURN;
        t.x.first -= diff_x;
        t.x.second += diff_x;
      } else {
        t.move |= Move::RIGHT_TURN;
        t.x.first -= diff_x;
        t.x.second += diff_x;
      }
    }

    if (curr == states.begin()) t.v = {0.0, 0.0};
    else t.v = {t.x.first / dt, t.x.second / dt};

    transitions_.emplace_back(curr->first, t);
  }
//  std::cout << transitions_ << std::endl;
}

void StateManager::setAlignmentParam(double x, double y) {
  align = [=](const State &s) { return State(s.x + x, s.y + y, s.yaw); };
}

const Instruction &StateManager::operator()(double t) {
  if (t <= 0) {
    instruction_.operation = Move::STOP;
    instruction_.des_state = transitions_.front().second.state;
    instruction_.des_velocity = {0.0, 0.0};
  } else if (t <= transitions_.back().first) {
    int idx = -1;
    for (auto iter = transitions_.begin(); iter != transitions_.end(); ++iter, ++idx) {
      if (iter->first < t) continue;
      auto s_p = iter - 1, s_n = iter;
      double period = s_n->first - s_p->first, dt = t - s_p->first;
      instruction_.des_state = State::interp(s_p->second.state, s_n->second.state, dt / period);
//      instruction_.des_velocity = s_n->second.v;
      interpolateVelocity(idx, dt, *s_p, *s_n);
      break;
    }
  } else {
    finished = true;
    instruction_.operation = Move::STOP;
    instruction_.des_state = transitions_.back().second.state;
    instruction_.des_velocity = {0.0, 0.0};
  }
  instruction_.des_state = align(instruction_.des_state);
  return instruction_;
}

//State StateManager::getState(double t) {
//  if (t <= 0) {
//    return states_.front().second;
//  } else if (t <= states_.back().first) {
//    for (auto iter = states_.begin(); iter != states_.end(); ++iter) {
//      if (iter->first < t) continue;
//      auto s_p = iter - 1, s_n = iter;
//      double ratio = (t - s_p->first) / (s_n->first - s_p->first);
//      return State::interp(s_p->second, s_n->second, ratio);
//    }
//  }
//  finished = true;
//  return states_.back().second;
//}
//
//std::tuple<double, double> StateManager::getInstruction(double t) { // vx, vw
//  if (t <= 0) {
//    return {0, 0};
//  } else if (t <= states_.back().first) {
//    for (auto iter = states_.begin(); iter != states_.end(); ++iter) {
//      if (iter->first < t) continue;
//      auto s_p = iter - 1, s_n = iter;
//      auto diff_state = s_n->second - s_p->second;
//      double d_yaw = diff_state.yaw, v = diff_state.diff();
//      if (diff_state.asVector2().dot(s_p->second.oritUnit2()) < 0) v *= -1;
//      return {v, d_yaw};
//    }
//  }
//  finished = true;
//  return {0, 0};
//}

std::string Move::move2str(Move::MoveType m) {
  switch (m) {
    case STOP: return "SP";
    case FORWARD: return "FW";
    case FORWARD | LEFT_TURN: return "LF";
    case FORWARD | RIGHT_TURN: return "RF";
    case BACK: return "BK";
    case BACK | LEFT_TURN: return "LB";
    case BACK | RIGHT_TURN: return "RB";
    default: throw std::invalid_argument("Invalid Move");
  }
}
