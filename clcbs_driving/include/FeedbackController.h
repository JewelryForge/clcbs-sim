#ifndef CLCBS_DRIVING_INCLUDE_FEEDBACKCONTROLLER_H_
#define CLCBS_DRIVING_INCLUDE_FEEDBACKCONTROLLER_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "StateManager.h"
#include "CarModel.h"
#include "PID.hpp"

class FeedbackController {
 public:
  FeedbackController(ros::NodeHandle &nh, std::string name, const std::vector<std::pair<double, State>>& states);
  void start();
  void stateUpdate(const geometry_msgs::Pose::ConstPtr& p);
  void spin();
  void spinOnce();
  void publishOnce(const std::pair<double, double> &v);
  void calculateVelocityAndPublish();

 private:
  ros::NodeHandle nh_;
  ros::Publisher left_pub_, right_pub_;
  ros::Subscriber state_sub_;
  tf::TransformBroadcaster tf_broadcaster_; // TODO: CHANGE TO tf2

  std::shared_ptr<State> curr_state_, prev_state_;
  double velocity_measured{0};
  double ROTATION_RADIUS{};
  std::string name_;
  bool is_started_{false};
  std::unique_ptr<StateManager> state_manager_;
  PID pid1_, pid2_;
  ros::Time t_start_{};
  CarModel model_;
};

#endif //CLCBS_DRIVING_INCLUDE_FEEDBACKCONTROLLER_H_
