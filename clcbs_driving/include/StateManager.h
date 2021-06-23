#ifndef CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
#define CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_

#include <utility>
#include <vector>
#include <tuple>
#include <functional>
#include "Angle.h"
#include <Eigen/Core>

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

//using MoveType = unsigned char;
//struct Move {
//  static const MoveType STOP, FORWARD, BACK, LEFT_TURN, RIGHT_TURN;
//};
//
//const MoveType Move::STOP = 0x00;
//const MoveType Move::FORWARD = 0x01;
//const MoveType Move::BACK = 0x02;
//const MoveType Move::LEFT_TURN = 0x10;
//const MoveType Move::RIGHT_TURN = 0x20;

struct Instruction {
  Move::MoveType operation = Move::STOP;
  State des_state;
  std::pair<double, double> des_velocity;
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

class StateManager {
 public:
  explicit StateManager(const std::vector<std::pair<double, State>> &states, std::string mode = "Linear");
  void setAlignmentParam(double x, double y);
  const Instruction &operator()(double t);
//  State operator()(double t);
//  std::tuple<double, double> getInstruction(double t);
//  State getState(double t);

  bool finished = false;
 private:
  struct Transition {
    Transition() = default;
    Transition(State state, std::pair<double, double> velocity, Move::MoveType move) :
        state(state), v(std::move(velocity)), move(move) {};
    friend std::ostream &operator<<(std::ostream &os, const StateManager::Transition &t) {
      return os << "T<" << t.state << "," << t.v << "," << Move::move2str(t.move) << ">" << std::endl;
    };
    State state;
    std::pair<double, double> x, v;
    Move::MoveType move = Move::STOP; // next move
  };
  Instruction instruction_;
//  std::vector<std::pair<double, State>> states_;
  std::vector<std::pair<double, Transition>> transitions_;
  std::function<State(const State &)> align;
  std::string mode_;
};

#endif //CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
