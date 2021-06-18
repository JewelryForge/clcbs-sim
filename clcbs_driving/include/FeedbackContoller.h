#ifndef CLCBS_DRIVING_INCLUDE_FEEDBACKCONTOLLER_H_
#define CLCBS_DRIVING_INCLUDE_FEEDBACKCONTOLLER_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "StateManager.h"
#include "CarModel.h"

class FeedbackContoller {
 public:
  FeedbackContoller(std::string name, std::vector<std::pair<double, State>> states, int argc, char **argv);
  void start();
  void stateUpdate(geometry_msgs::Pose::ConstPtr p);
  void spin();
  void publishOnce();
  static void reset(int);

 private:
  ros::NodeHandle nh_;
  ros::Publisher left_pub_, right_pub_;
  ros::Subscriber state_sub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::unique_ptr<State> curr_state_;
  double ROTATION_RADIUS{};
  bool is_started_{false};
  StateManager state_manager_;
  ros::Time t_start_{};
  CarModel model_;
};

#endif //CLCBS_DRIVING_INCLUDE_FEEDBACKCONTOLLER_H_
