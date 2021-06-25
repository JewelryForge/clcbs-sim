//
// Created by jewelry on 25/06/2021.
//
#include "PlanVisualizer.h"
#include <geometry_msgs/Point.h>
#include "Constants.h"

PlanVisualizer::PlanVisualizer(ros::NodeHandle &nh) : nh_(nh) {
  term_publisher_ = nh_.advertise<visualization_msgs::Marker>("/path_visualizer", 10);
//  path_publisher_ = nh_.advertise<nav_msgs::Path>("/path_visualizer", 10);
}

geometry_msgs::Point makePoint(double x, double y, double z = 0) {
  geometry_msgs::Point p;
  std::tie(p.x, p.y, p.z) = {x, y, z};
  return p;
}

void PlanVisualizer::addPlan(const std::vector<std::pair<double, State>> &states) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.header.frame_id = "map";
  marker.id = counter_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  const auto &term_state = states.back().second;
  marker.pose.position.x = term_state.x - Constants::MAP_SIZE_X / 2; // TODO: CANCEL ALIGNMENT
  marker.pose.position.y = term_state.y - Constants::MAP_SIZE_Y / 2;
  marker.pose.orientation.w = std::cos(term_state.yaw / 2);
  marker.pose.orientation.z = std::sin(term_state.yaw / 2);
  marker.scale.x = Constants::CAR_LENGTH;
  marker.scale.y = Constants::CAR_WIDTH;
  marker.scale.z = 0.1;
  markers_.push_back(marker);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.id = counter_++;
  marker.color.b = 1.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  for (const auto &t_state : states) {
    const auto &state = t_state.second;
    marker.points.push_back(makePoint(state.x - Constants::MAP_SIZE_X / 2, state.y - Constants::MAP_SIZE_Y / 2));
  }
  markers_.push_back(marker);

//  nav_msgs::Path path;
//  geometry_msgs::PoseStamped pose;
//  path.header.frame_id = "map";
//  path.header.seq = counter_;
//  int counter = 0;
//  pose.header.frame_id = "map";

//  path.header.stamp = ros::Time::now();
//  for (const auto &t_state : states) {
//    const auto &state = t_state.second;
//    pose.header.seq = counter++;
//    pose.pose.position.x = state.x;
//    pose.pose.position.y = state.y;
//    pose.pose.orientation.w = std::cos(state.yaw / 2);
//    pose.pose.orientation.z = std::sin(state.yaw / 2);
//    path.poses.push_back(pose);
//  }
//  paths_.push_back(path);
//  counter_++;
}

void PlanVisualizer::publishOnce() {
//  visualization_msgs::Marker del;
//  del.type = visualization_msgs::Marker::DELETEALL;
//  del.header.frame_id = "map";
//  term_publisher_.publish(del);
  for (const auto &marker: markers_) {
    term_publisher_.publish(marker);
  }
}