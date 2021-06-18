#include "CarModel.h"
#include "FeedbackContoller.h"
#include <iostream>
#include "PID.hpp"
#include "Angle.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "csignal"
#include "ctime"
#include "yaml-cpp/yaml.h"
using namespace std;
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

int main() {
  YAML::Node config;
  std::string file("/home/jewelry/catkin_ws/CLCBS/src/clcbs_driving/output.yaml");
  config = YAML::LoadFile(file);
  auto agent0 = config["schedule"]["agent0"];
  std::vector<std::pair<double, State>> t_states;
  for (auto s : agent0) {
    t_states.emplace_back(s["t"].as<double>(),
                          State(s["x"].as<double>(), s["y"].as<double>(), -s["yaw"].as<double>()));
  }
  cout << t_states << endl;
//  FeedbackContoller();
}