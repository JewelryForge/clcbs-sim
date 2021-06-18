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
  double x, y;
  Angle yaw;
};

class StateManager {
 public:
  StateManager(std::string name, std::vector<std::pair<double, State>> states);
  void setAlignmentParam(double x, double y);;
  State operator()(double t);

 private:
  State getState(double t);
  std::string name_;
  std::vector<std::pair<double, State>> states_;
  std::function<State(const State &)> align;
};

#endif //CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
