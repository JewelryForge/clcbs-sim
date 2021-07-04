#ifndef CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
#define CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_

#include <utility>
#include <vector>
#include <tuple>
#include <functional>
#include "Angle.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

class State {
 public:
  State() : x(0), y(0), yaw(0) {};
  State(double x, double y, Angle yaw);
  State(double x, double y, double yaw);
  friend State operator+(const State &s1, const State &s2);
  friend State operator-(const State &s1, const State &s2);
  friend State operator*(const State &s1, double p);
  friend State operator*(double p, const State &s1);
  friend State operator/(const State &s1, double p);
  friend std::ostream &operator<<(std::ostream &os, const State &s);
  std::tuple<double, double, Angle> asTuple() const;
  Eigen::Vector2d asVector2() const;
  Eigen::Vector3d asVector3() const;
  Eigen::Vector2d oritUnit2() const;
  Eigen::Vector3d oritUnit3() const;
  static State interp(const State &s1, const State &s2, double ratio);
  double norm() const;
  double diff() const;
  double x, y;
  Angle yaw;
};

namespace Move {
using MoveType = unsigned char;
static const MoveType STOP = 0x00;
static const MoveType FORWARD = 0x01;
static const MoveType BACK = 0x02;
static const MoveType LEFT_TURN = 0x10;
static const MoveType RIGHT_TURN = 0x20;

std::string move2str(MoveType m);
};

struct Instruction {
  Move::MoveType operation = Move::STOP;
  State des_state, local_dest, global_dest;
  Eigen::Vector2d des_velocity;
};

struct Transition {
  Transition() = default;
  explicit Transition(State state, Eigen::Vector2d velocity = Eigen::Vector2d::Zero(), Move::MoveType move = Move::STOP) :
      state(state), v(std::move(velocity)), move(move) {};
  friend std::ostream &operator<<(std::ostream &os, const Transition &t);;
  State state;
  Eigen::Vector2d x, v; // x_linear, x_angular, v_linear, v_angular
  Move::MoveType move = Move::STOP; // next move
};

class StateManager {
 public:
  explicit StateManager(const std::vector<std::pair<double, State>> &states);
  const Instruction &operator()(double t);
  bool finished = false;
 protected:
  virtual void interpolateVelocity(int idx, double dt, const std::pair<double, Transition> &s_p,
                                   const std::pair<double, Transition> &s_n);
  Instruction instruction_;
  std::vector<std::pair<double, Transition>> logs_;
};

class Poly3StateManager : public StateManager {
  void interpolateVelocity(int /* idx */, double dt, const std::pair<double, Transition> &s_p,
                           const std::pair<double, Transition> &s_n) override;
};

class RealStateManager : public StateManager {
 public:
  explicit RealStateManager(const std::vector<std::pair<double, State>> &states);
 private:
  void interpolateVelocity(int idx, double dt, const std::pair<double, Transition> &s_p,
                           const std::pair<double, Transition> &s_n) override;
  Angle init_yaw_;
  Eigen::VectorXd linear_params_, angular_params_;
};

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  if (v.empty()) return os << "[]";
  os << '[' << v.front();
  for (auto iter = v.begin() + 1; iter != v.end(); ++iter) {
    os << '\t' << *iter;
  }
  return os << ']';
}

template<typename T1, typename T2>
std::ostream &operator<<(std::ostream &os, const std::pair<T1, T2> &p) {
  return os << '<' << p.first << ", " << p.second << '>';
}
#endif //CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
