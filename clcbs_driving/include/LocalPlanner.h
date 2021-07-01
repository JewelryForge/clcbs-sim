#ifndef CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_
#define CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "StateManager.h"
#include "PlanVisualizer.h"
#include "CarModel.h"
#include "PID.hpp"
#include "WheelOdometer.h"

class LocalPlanner {
 public:
  LocalPlanner(std::string name, const std::vector<std::pair<double, State>> &states);
  bool isActive() const;
  void calculateVelocityAndPublish();
  static bool activateAll();

 private:
  void stateUpdate(const geometry_msgs::Pose::ConstPtr &p);
  void publishOnce(const std::pair<double, double> &v);
  void calculateVelocityAndPublishBase(double dt);

  static std::vector<LocalPlanner *> all_controller_;
  ros::NodeHandle nh_;
  ros::Publisher left_pub_, right_pub_;
  ros::Subscriber state_sub_;
  tf::TransformBroadcaster tf_broadcaster_; // TODO: CHANGE TO tf2

//  PlanVisualizer visualizer_;
  WheelOdometer odom_;
  std::unique_ptr<State> curr_state_;
  std::string name_;
  bool is_finished_ = false;
  std::unique_ptr<StateManager> state_manager_;
  ros::Time t_start_{};
  CarModel model_;
};

#endif //CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_
