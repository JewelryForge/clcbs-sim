#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <utility>

#include "LocalPlanner.h"

std::vector<LocalPlanner *> LocalPlanner::all_controller_;

LocalPlanner::LocalPlanner(std::string name, const std::vector<std::pair<double, State>> &states)
    : name_(std::move(name)), model_(), odom_(name, states.front().second) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/left_wheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/right_wheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/" + name_ + "/robot_base", 1,
                                                  [=](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  state_manager_ = std::make_unique<MinAccStateManager>(states);
  all_controller_.push_back(this);
}

void LocalPlanner::stateUpdate(const geometry_msgs::Pose::ConstPtr &p) {
  const auto &q = p->orientation;
  Angle yaw(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
  curr_state_ = std::make_unique<State>(p->position.x, p->position.y, yaw);
}

bool LocalPlanner::activateAll() {
  for (auto &c: all_controller_) {
    if (c->isActive()) return false;
  }
  auto t_start = ros::Time::now() + ros::Duration(0.5);
  for (auto &c: all_controller_) {
    c->t_start_ = t_start;
  }
  return true;
}

bool LocalPlanner::isActive() const {
  return curr_state_ != nullptr;
}

void LocalPlanner::publishOnce(const std::pair<double, double> &v) {
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
  left_wheel_velocity.data = v.first / Constants::WHEEL_RADIUS;
  right_wheel_velocity.data = v.second / Constants::WHEEL_RADIUS;
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
}

void LocalPlanner::calculateVelocityAndPublish() {
  if (isActive()) calculateVelocityAndPublishBase((ros::Time::now() - t_start_).toSec());
}

void LocalPlanner::calculateVelocityAndPublishBase(double dt) {
  const Instruction &des = (*state_manager_)(dt);
  const State &interp_state = des.des_state;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(interp_state.x, interp_state.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, interp_state.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name_ + "/desired_state"));

  if (is_finished_) {
    publishOnce({0, 0});
    return;
  }
  State dest_diff = des.global_dest - *curr_state_, interp_diff = interp_state - *curr_state_;

  if (state_manager_->finished or dest_diff.norm() < 2.0) {
    double heading_deviation = Angle(std::atan2(dest_diff.y, dest_diff.x)) - curr_state_->yaw;
    double delta_yaw = dest_diff.yaw;
    if (std::abs(dest_diff.asVector2().dot(curr_state_->oritUnit2())) < 0.1 && std::abs(delta_yaw) < M_PI / 24) {
      model_.reset();
      is_finished_ = true;
      ROS_INFO_STREAM(name_ << " FINISHED " << dest_diff);
    } else {
      if (std::abs(heading_deviation) > M_PI_2) heading_deviation = Angle(heading_deviation + M_PI);
      model_.setVx(1.0 * sign(dest_diff.asVector2().dot(curr_state_->oritUnit2())) * dest_diff.diff());
      model_.setVw(4.0 * delta_yaw + 1.0 * heading_deviation * dest_diff.norm());
      // TODO: DYNAMIC PATH PLANNING USING REEDS-SHEPP CURVES
      ROS_INFO_STREAM(name_ << " TUNING ");
    }
  } else {
    double vl = des.des_velocity(0), vr = des.des_velocity(1);
    double heading_deviation = Angle(std::atan2(interp_diff.y, interp_diff.x)) - curr_state_->yaw;
    if (std::abs(heading_deviation) > M_PI_2) {
      heading_deviation = Angle(heading_deviation + M_PI);
    }
    double vx = (vl + vr) / 2, vw = (vr - vl) / Constants::CAR_WIDTH;
    double delta_yaw = des.des_state.yaw - curr_state_->yaw;
    model_.setVx(vx + 1.0 * sign(vx) * interp_diff.asVector2().dot(curr_state_->oritUnit2()));
    model_.setVw(vw + 6.0 * delta_yaw + 1.0 * heading_deviation * interp_diff.norm());
    ROS_INFO_STREAM(name_ << " TRACING " << model_.getVelocity());
  }
  publishOnce(model_.getVelocity());
}

