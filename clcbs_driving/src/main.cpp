#include "CarModel.h"
#include "LocalPlanner.h"
#include <iostream>
#include "ctime"
#include <yaml-cpp/yaml.h>
#include "GlobalPlanner.h"
#include <boost/program_options.hpp>
#include "PlanVisualizer.h"
#include <xmlrpcpp/XmlRpcValue.h>
using namespace std;

double toNum(const XmlRpc::XmlRpcValue &v) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return int(v);
  else return double(v);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Tracking");
  ros::NodeHandle nh;
  bool is_sim, is_real;
  std::string which_agent;
  XmlRpc::XmlRpcValue schedule;
  nh.getParam("is_sim", is_sim);
  nh.getParam("is_real", is_real);
  nh.getParam("agent_binding", which_agent);
  nh.getParam("/config/schedule", schedule);
  Constants::loadFromRosParam();

  std::vector<std::unique_ptr<LocalPlannerBase>> controllers;
  PlanVisualizer visualizer;
  for (const auto &a : schedule) {
    std::string agent = a.first;
    std::vector<std::pair<double, State>> t_states;
    for (int i = 0; i < a.second.size(); i++) {
      const auto &mid_point = a.second[i];
      t_states.emplace_back(toNum(mid_point["t"]),
                            State(toNum(mid_point["x"]) - Constants::MAP_SIZE_X / 2,
                                  toNum(mid_point["y"]) - Constants::MAP_SIZE_Y / 2,
                                  -toNum(mid_point["yaw"])));
    }
    visualizer.addPlan(t_states);
    controllers.push_back(std::make_unique<LocalPlannerSim>(agent, t_states));
    if (is_real && agent == which_agent) controllers.push_back(std::make_unique<LocalPlannerReal>(agent, t_states));
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
    for (auto &ptr : controllers) ptr->calculateVelocityAndPublish();
    rate.sleep();
  }
}