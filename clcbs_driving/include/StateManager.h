#ifndef CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
#define CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_

#include <vector>
#include <tuple>
#include <functional>
#include "Angle.h"
#include <Eigen/Core>

class State {
 public:
  State(double x, double y, Angle yaw);
  State(double x, double y, double yaw);
  friend State operator+(const State &s1, const State &s2);
  friend State operator-(const State &s1, const State &s2);
  friend State operator*(const State &s1, double p);
  friend State operator*(double p, const State &s1);
  friend State operator/(const State &s1, double p);
  friend std::ostream &operator<<(std::ostream& os, const State& s);
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

class StateManager {
 public:
  explicit StateManager(std::vector<std::pair<double, State>> states);
  void setAlignmentParam(double x, double y);;
  State operator()(double t);
  std::tuple<double, double> getInstruction(double t);

  template<typename T>
  friend std::ostream &operator<<(std::ostream &os, const std::vector<T> &v);
  template<typename T1, typename T2>
  friend std::ostream &operator<<(std::ostream &os, const std::pair<T1, T2> &p);
  bool finished = false;
 private:
  State getState(double t);
  std::vector<std::pair<double, State>> states_;
  std::function<State(const State &)> align;
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
