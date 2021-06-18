#ifndef CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
#define CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_

#include <vector>
#include <tuple>
#include <functional>
#include "Angle.h"

class State {
 public:
  State(double x, double y, Angle yaw);
  friend State operator+(const State &s1, const State &s2);
  friend State operator-(const State &s1, const State &s2);
  friend State operator*(const State &s1, double p);
  friend State operator*(double p, const State &s1);
  friend State operator/(const State &s1, double p);
  friend std::ostream &operator<<(std::ostream& os, const State& s);
  static State interp(const State &s1, const State &s2, double ratio);
  double norm() const;
  double x, y;
  Angle yaw;
};

class StateManager {
 public:
  StateManager(std::string name, std::vector<std::pair<double, State>> states);
  void setAlignmentParam(double x, double y);;
  State operator()(double t);

  template<typename T>
  friend std::ostream &operator<<(std::ostream &os, const std::vector<T> &v);
  template<typename T1, typename T2>
  friend std::ostream &operator<<(std::ostream &os, const std::pair<T1, T2> &p);
  bool finished = false;
 private:
  State getState(double t);
  std::string name_;
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
