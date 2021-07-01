//
// Created by jewelry on 30/06/2021.
//

#ifndef CLCBS_DRIVING_INCLUDE_WHEELODOMETER_H_
#define CLCBS_DRIVING_INCLUDE_WHEELODOMETER_H_

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include "StateManager.h"
#include "tf2_ros/transform_broadcaster.h"

class WheelOdometer {
 public:
  explicit WheelOdometer(const std::string &agent_name, State init_state = State());
  void jointStateUpdate(const sensor_msgs::JointState::ConstPtr &p);
  void integral(const ros::TimerEvent &e);

 private:
  std::string name_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  Eigen::Vector2d vel_;
  State state_;
  tf2_ros::TransformBroadcaster br_;
};

#endif //CLCBS_DRIVING_INCLUDE_WHEELODOMETER_H_
