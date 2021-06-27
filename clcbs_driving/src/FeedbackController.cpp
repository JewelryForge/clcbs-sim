#include <csignal>
#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <PID.hpp>

#include "FeedbackController.h"

std::vector<const FeedbackController *> FeedbackController::all_controller;

FeedbackController::FeedbackController(ros::NodeHandle &nh, std::string name,
                                       const std::vector<std::pair<double, State>> &states)
    : name_(std::move(name)), model_(), pid1_(2, 0.1, 1.0), pid2_(2, 0.1, 1.0) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/left_wheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/right_wheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/" + name_ + "/robot_base", 1,
                                                  [this](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  state_manager_ = std::make_unique<MinAccStateManager>(states);
  state_manager_->setAlignmentParam(-Constants::MAP_SIZE_X / 2, -Constants::MAP_SIZE_Y / 2);
  all_controller.push_back(this);
}
void FeedbackController::start() {
  is_started_ = true;
  t_start_ = ros::Time::now();
}
void FeedbackController::stateUpdate(const geometry_msgs::Pose::ConstPtr &p) {
  const auto &q = p->orientation;
  Angle yaw(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
  curr_state_ = std::make_unique<State>(p->position.x, p->position.y, yaw);
}

bool FeedbackController::isActive() const {
  return curr_state_ != nullptr;
}

void FeedbackController::spinOnce() {
  if (isActive()) {
    if (!is_started_) start();
    calculateVelocityAndPublish();
  }
}

void FeedbackController::publishOnce(const std::pair<double, double> &v) {
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
  left_wheel_velocity.data = v.first / Constants::WHEEL_RADIUS;
  right_wheel_velocity.data = v.second / Constants::WHEEL_RADIUS;
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
}

void FeedbackController::calculateVelocityAndPublish() {
  double dt = (ros::Time::now() - t_start_).toSec();
  const Instruction &des = (*state_manager_)(dt);
  const State &interp_state = des.interp_state;
//  std::tie(vx, vw) = state_manager_.getInstruction(dt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(interp_state.x, interp_state.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, interp_state.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name_ + "_desired_state"));

  if (is_finished_) {
    publishOnce({0, 0});
    return;
  }
  if (state_manager_->finished or (des.terminal - *curr_state_).norm() < 2.0) {
    State diff_state = des.terminal - *curr_state_;
    double heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - curr_state_->yaw;
    double delta_yaw = diff_state.yaw;
    if (diff_state.norm() < 0.2 && delta_yaw < M_PI / 24) {
      model_.reset();
      is_finished_ = true;
      ROS_INFO_STREAM(name_ << " FINISHED " << delta_yaw);
    } else {
      if (std::abs(heading_deviation) > M_PI_2) heading_deviation = Angle(heading_deviation + M_PI);
      model_.setVx(1.0 * sign(diff_state.asVector2().dot(curr_state_->oritUnit2())) * diff_state.diff());
      model_.setVw(4.0 * delta_yaw + 1.0 * heading_deviation * diff_state.norm());
      // TODO: DYNAMIC PATH PLANNING USING REEDS-SHEPP CURVES
      ROS_INFO_STREAM(name_ << " TUNING ");
    }
  } else {
    double vl, vr;
    std::tie(vl, vr) = des.des_velocity;
    State diff_state = interp_state - *curr_state_;
    double heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - curr_state_->yaw;
    if (std::abs(heading_deviation) > M_PI_2) {
      heading_deviation = Angle(heading_deviation + M_PI);
    }
    double vx = (vl + vr) / 2, vw = (vr - vl) / Constants::CAR_WIDTH;
    double delta_yaw = des.des_state.yaw - curr_state_->yaw;
    model_.setVx(vx + 1.0 * sign(vx) * diff_state.asVector2().dot(curr_state_->oritUnit2()));
    model_.setVw(vw + 6.0 * delta_yaw + 1.0 * heading_deviation * diff_state.norm());
    ROS_INFO_STREAM(name_ << '\t' << model_.getVelocity());
  }
  publishOnce(model_.getVelocity());
}

bool FeedbackController::allActive() {
  for (auto c: all_controller) {
    if (!c->isActive()) return false;
  }
  return true;
}
void FeedbackController::calculateVelocityAndPublish(const ros::TimerEvent) {

}
void FeedbackController::calculateVelocityAndPublishBase(double dt) {

}


