#ifndef CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_
#define CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include "StateManager.h"

class PlanVisualizer {
 public:
  explicit PlanVisualizer(ros::NodeHandle &nh);
  void addPlan(const std::vector<std::pair<double, State>> &states);
  void publishOnce();
 private:
  ros::NodeHandle nh_;
  ros::Publisher path_publisher_;
  std::vector<visualization_msgs::Marker> markers_;
  int counter = 0;
};

#endif //CLCBS_DRIVING_INCLUDE_PLANVISUALIZER_H_
