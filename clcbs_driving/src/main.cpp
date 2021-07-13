#include "CarModel.h"
#include "LocalPlanner.h"
#include <iostream>
#include "ctime"
#include <yaml-cpp/yaml.h>
#include "StateManager.h"
#include <boost/program_options.hpp>
#include "PlanVisualizer.h"
using namespace std;

int main(int argc, char **argv) {
  // Load command line parameters -- -s <SCHEDULE_FILE>
  std::string schedule_file;
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("sch,s", boost::program_options::value(&schedule_file) /* ->required() */, "schedule file (yaml)");
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  YAML::Node config = YAML::LoadFile(schedule_file);
  auto schedule = config["schedule"];

  // Load environment -- simulation, real or both
  ros::init(argc, argv, "CLCBS_DRIVING");
  ros::NodeHandle nh;
  bool is_sim, is_real;
  std::string which_agent;
  nh.param("IS_SIM", is_sim, true);
  nh.param("IS_REAL", is_real, false);
  nh.param("WHICH_REAL_AGENT", which_agent, std::string("agent0"));

  // Construct controllers with loaded schedules
  std::vector<std::unique_ptr<LocalPlannerBase>> controllers;
  PlanVisualizer visualizer;
  for (auto iter = schedule.begin(); iter != schedule.end(); ++iter) {
    std::string key = iter->first.as<std::string>();
    std::vector<std::pair<double, State>> t_states;
    for (auto s : iter->second) {
      auto t = s["t"].as<double>(), x = s["x"].as<double>(), y = s["y"].as<double>(), yaw = s["yaw"].as<double>();
      t_states.emplace_back(t, State(x - Constants::MAP_SIZE_X / 2, y - Constants::MAP_SIZE_Y / 2, -yaw));
    }
    visualizer.addPlan(t_states);
    controllers.push_back(std::make_unique<LocalPlannerSim>(key, t_states));
    if (is_real && key == which_agent) controllers.push_back(std::make_unique<LocalPlannerReal>(key, t_states));
  }
  ROS_INFO("SETTING UP FINISHED");
  ros::Duration(0.5).sleep();

  // Publish path to RVIZ
  if (is_sim) {
    visualizer.publishOnce();
    ros::Duration(0.5).sleep();
  }

  // Start controlling
  while (!LocalPlannerBase::activateAll());
  auto rate = ros::Rate(50);
  while (ros::ok()) {
    ros::spinOnce();
    for (auto &ptr : controllers) {
      ptr->calculateVelocityAndPublish();
    }
    rate.sleep();
  }
}