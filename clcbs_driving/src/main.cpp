#include "CarModel.h"
#include "FeedbackController.h"
#include <iostream>
#include "PID.hpp"
#include "Angle.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "csignal"
#include "ctime"
#include "yaml-cpp/yaml.h"
#include "StateManager.h"
using namespace std;

int main(int argc, char **argv) {
  YAML::Node config;
  std::string file("/home/jewelry/catkin_ws/CLCBS/src/clcbs_driving/output.yaml");
  config = YAML::LoadFile(file);
  auto agent0 = config["schedule"]["agent0"];
  std::vector<std::pair<double, State>> t_states;
  for (auto s : agent0) {
    t_states.emplace_back(s["t"].as<double>(),
                          State(s["x"].as<double>(), s["y"].as<double>(), -s["yaw"].as<double>()));
  }
  ros::init(argc, argv, "CPP_TEST");
  ros::NodeHandle nh;
  auto publisher = FeedbackController(nh, "agent0", t_states);
  publisher.spin();
}