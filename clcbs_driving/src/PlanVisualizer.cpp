//
// Created by jewelry on 25/06/2021.
//
#include "PlanVisualizer.h"
#include <geometry_msgs/Point.h>
#include "Constants.h"

PlanVisualizer::PlanVisualizer(ros::NodeHandle &nh) : nh_(nh) {
  path_publisher_ = nh_.advertise<visualization_msgs::Marker>("/path_visualizer", 1);
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
  marker.id = counter++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  const auto &term_state = states.back().second - State(Constants::MAP_SIZE_X / 2, Constants::MAP_SIZE_Y / 2, 0);
  marker.pose.position.x = term_state.x;
  marker.pose.position.y = term_state.y;
  marker.scale.x = Constants::CAR_LENGTH;
  marker.scale.y = Constants::CAR_WIDTH;
  marker.scale.z = 0.1;
  marker.pose.orientation.w = std::cos(term_state.yaw / 2);
  marker.pose.orientation.z = std::sin(term_state.yaw / 2);
  markers_.push_back(marker);
}

void PlanVisualizer::publishOnce() {
//  visualization_msgs::Marker del;
//  del.type = visualization_msgs::Marker::DELETEALL;
//  del.header.frame_id = "map";
//  path_publisher_.publish(del);
  for (const auto &marker: markers_) {
    path_publisher_.publish(marker);
  }
}