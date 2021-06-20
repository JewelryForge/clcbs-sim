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
  ros::init(argc, argv, "CPP_TEST");
  ros::NodeHandle nh;
  config = YAML::LoadFile(file);
  auto schedule = config["schedule"];
  std::vector<std::unique_ptr<FeedbackController>> controllers;
  for (YAML::const_iterator iter = schedule.begin(); iter != schedule.end(); ++iter) {
    std::string key = iter->first.as<std::string>();
    std::vector<std::pair<double, State>> t_states;
    for (auto s : schedule[key]) {
      auto t = s["t"].as<double>(), x = s["x"].as<double>(), y = s["y"].as<double>(), yaw = -s["yaw"].as<double>();
      t_states.emplace_back(t, State(x, y, yaw));
    }
    controllers.push_back(std::make_unique<FeedbackController>(nh, key, t_states));
  }

  int PUBLISHING_FREQUENCY;
  nh.param("PUBLISHING_FREQUENCY", PUBLISHING_FREQUENCY, 50);
  auto rate = ros::Rate(PUBLISHING_FREQUENCY);
  while (ros::ok()) {
    for (auto &ptr : controllers) {
      ptr->spinOnce();
    }
    rate.sleep();
  }

}