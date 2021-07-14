#ifndef CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_
#define CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "GlobalPlanner.h"

class PlanVisualizer {
 public:
  PlanVisualizer();
  void addPlan(const std::vector<std::pair<double, State>> &states);
  void publishOnce();
 private:
  ros::NodeHandle nh_;
  ros::Publisher path_publisher_;
  std::vector<visualization_msgs::Marker> markers_;
  std::vector<nav_msgs::Path> paths_;
  int counter_ = 0;
};

#endif //CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_
