//
// Created by jewelry on 25/06/2021.
//
#include "PlanVisualizer.h"
#include <geometry_msgs/Point.h>
#include "Constants.h"

PlanVisualizer::PlanVisualizer() {
  path_publisher_ = nh_.advertise<visualization_msgs::Marker>("/path_visualizer", 10);
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
  const auto &dest_state = states.back().second;
  marker.pose.position.x = dest_state.x;
  marker.pose.position.y = dest_state.y;
  marker.pose.position.x += std::cos(dest_state.yaw) * (Constants::CAR_LENGTH / 2 - Constants::WHEEL_RADIUS);
  marker.pose.position.y += std::sin(dest_state.yaw) * (Constants::CAR_LENGTH / 2 - Constants::WHEEL_RADIUS);
  marker.pose.orientation.w = std::cos(dest_state.yaw / 2);
  marker.pose.orientation.z = std::sin(dest_state.yaw / 2);
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
  marker.scale.x = 0.1;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  for (const auto &t_state : states) {
    const auto &state = t_state.second;
    marker.points.push_back(makePoint(state.x, state.y));
  }
  markers_.push_back(marker);
}

void PlanVisualizer::publishOnce() {
  for (const auto &marker: markers_) {
    path_publisher_.publish(marker);
  }
}