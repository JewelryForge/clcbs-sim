//
// Created by jewelry on 30/06/2021.
//

#include "WheelOdometer.h"
#include "Constants.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

WheelOdometer::WheelOdometer(const std::string &agent_name, State init_state) :
    name_(agent_name), state_(init_state) {
  sub_ = nh_.subscribe<hunter_msgs::HunterStatus>("/hunter_status", 1,
                                                  [=](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  pub_ = nh_.advertise<geometry_msgs::Pose>("/" + agent_name + "/odometry", 1);
  timer_ = nh_.createTimer(ros::Rate(20), &WheelOdometer::integral, this);
}

void WheelOdometer::stateUpdate(const hunter_msgs::HunterStatus::ConstPtr &p) {
  linear_velocity_ = p->linear_velocity;
  angular_velocity_ = linear_velocity_ / (sign(p->angular_velocity) * 0.579 / 2 + 0.650 / std::tan(p->angular_velocity));
  ROS_INFO("State Update: linear = %.1lf, angular = %.1lf", linear_velocity_, angular_velocity_);
  is_started_ = true;
}

void WheelOdometer::integral(const ros::TimerEvent &e) {
  if (is_started_) {
    double dt = (e.current_real - e.last_real).toSec();
    double x = linear_velocity_ * dt, r = angular_velocity_ * dt;
    state_.x += x * std::cos(state_.yaw);
    state_.y += x * std::sin(state_.yaw);
    state_.yaw += r;
    geometry_msgs::Pose p;
//    geometry_msgs::TransformStamped t;
    /* t.transform.translation.x = */ p.position.x = state_.x;
    /* t.transform.translation.y = */ p.position.y = state_.y;
    /* t.transform.rotation.w = */ p.orientation.w = std::cos(state_.yaw / 2);
    /* t.transform.rotation.z = */ p.orientation.z = std::sin(state_.yaw / 2);
    pub_.publish(p);
    /*
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "map";
    t.child_frame_id = name_ + "/odom";
    br_.sendTransform(t);
    */
  }
}
