#include "CarModel.h"
#include "FeedbackController.h"
#include <iostream>
#include "PID.hpp"
#include "Angle.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "csignal"
#include "ctime"
#include <yaml-cpp/yaml.h>
#include "StateManager.h"
#include <boost/program_options.hpp>
using namespace std;

int main(int argc, char **argv) {
  std::string schedule_file;
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("sch,s", boost::program_options::value(&schedule_file)->required(), "schedule file (yaml)");
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  YAML::Node config;
  ros::init(argc, argv, "CPP_TEST");
  ros::NodeHandle nh;
//  schedule_file = "/home/jewelry/docker_ws/ros-melodic-ws/CLCBS/src/clcbs_driving/output.yaml";
  config = YAML::LoadFile(schedule_file);
  auto schedule = config["schedule"];
  std::vector<std::unique_ptr<FeedbackController>> controllers;
  for (auto iter = schedule.begin(); iter != schedule.end(); ++iter) {
    std::string key = iter->first.as<std::string>();
    key = "agent4";
    std::vector<std::pair<double, State>> t_states;
    for (auto s : schedule[key]) {
      auto t = s["t"].as<double>(), x = s["x"].as<double>(), y = s["y"].as<double>(), yaw = -s["yaw"].as<double>();
      t_states.emplace_back(t, State(x, y, yaw));
    }
    controllers.push_back(std::make_unique<FeedbackController>(nh, key, t_states));
    break;
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