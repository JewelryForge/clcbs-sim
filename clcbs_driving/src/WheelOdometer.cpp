//
// Created by jewelry on 30/06/2021.
//

#include "WheelOdometer.h"
#include "Constants.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

WheelOdometer::WheelOdometer(const std::string &agent_name, State init_state) :
    name_(agent_name), vel_(0.0, 0.0), state_(init_state) {
  sub_ = nh_.subscribe<sensor_msgs::JointState>("/" + agent_name + "/joint_states", 1,
                                                [=](auto &&PH1) { jointStateUpdate(std::forward<decltype(PH1)>(PH1)); });
  pub_ = nh_.advertise<geometry_msgs::Pose>("/" + agent_name + "/wheel_odometry", 1);
  timer_ = nh_.createTimer(ros::Rate(50), &WheelOdometer::integral, this);
}

void WheelOdometer::jointStateUpdate(const sensor_msgs::JointState_<std::allocator<void>>::ConstPtr &p) {
  if (p->name.size() == 2 and p->velocity.size() == 2) {
    for (int i = 0; i < 2; i++) {
      if (p->name[i] == "agent_leftwheel_joint")
        vel_(0) = p->velocity[i] * Constants::WHEEL_RADIUS;
      else if (p->name[i] == "agent_rightwheel_joint")
        vel_(1) = p->velocity[i] * Constants::WHEEL_RADIUS;
    }
//    ROS_INFO("JointStateUpdate");
  }
}

void WheelOdometer::integral(const ros::TimerEvent &e) {
  Eigen::Vector2d x = vel_ * (e.current_real - e.last_real).toSec();
  state_.x += (x(0) + x(1)) / 2 * std::cos(state_.yaw);
  state_.y += (x(0) + x(1)) / 2 * std::sin(state_.yaw);
  state_.yaw += (x(1) - x(0)) / Constants::CAR_WIDTH;
  geometry_msgs::Pose p;
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = p.position.x = state_.x;
  t.transform.translation.y = p.position.y = state_.y;
  t.transform.rotation.w = p.orientation.w = std::cos(state_.yaw / 2);
  t.transform.rotation.z = p.orientation.z = std::sin(state_.yaw / 2);
  pub_.publish(p);
  t.header.stamp = ros::Time::now();
  t.header.frame_id = "map";
  t.child_frame_id = name_ + "/wheel_odom";
  br_.sendTransform(t);
//  ROS_INFO("WheelOdometerPublish");
}
