#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <utility>
#include "LocalPlanner.h"

std::vector<LocalPlannerBase *> LocalPlannerBase::all_controller_;

LocalPlannerBase::LocalPlannerBase(std::string name, const std::vector<std::pair<double, State>> &states)
    : name_(std::move(name)) {
  state_manager_ = std::make_unique<MiniAccGlobal>(states);
  all_controller_.push_back(this);
}

bool LocalPlannerBase::isActive() const {
  return curr_state_ != nullptr;
}

bool LocalPlannerBase::activateAll() {
  for (auto &c: all_controller_) {
    if (c->isActive()) return false;
  }
  auto t_start = ros::Time::now() + ros::Duration(0.5);
  for (auto &c: all_controller_) c->t_start_ = t_start;
  return true;
}

void LocalPlannerBase::calculateVelocityAndPublish() {
  if (isActive()) calculateVelocityAndPublishBase((ros::Time::now() - t_start_).toSec());
}

void LocalPlannerBase::tfPublishOnce(const State &s) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(s.x, s.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, s.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name_ + "/desired_state"));
}

void LocalPlannerBase::calculateVelocityAndPublishBase(double dt) {
  const Instruction &des = (*state_manager_)(dt);
  const State &des_state = des.des_state;
  tfPublishOnce(des_state);
  if (is_finished_) {
    cmdPublishOnce(0.0, 0.0);
    return;
  }
  State dest_diff = des.global_dest - *curr_state_, interp_diff = des_state - *curr_state_;

  if (state_manager_->finished or dest_diff.norm() < 0.5) { //TODO: GET A SMARTER STRATEGY
    double heading_deviation = Angle(std::atan2(dest_diff.y, dest_diff.x)) - curr_state_->yaw;
    double delta_yaw = dest_diff.yaw;
    if (std::abs(dest_diff.asVector2().dot(curr_state_->oritUnit2())) < 0.05 && std::abs(delta_yaw) < M_PI / 24) {
      cmdPublishOnce(0.0, 0.0);
      is_finished_ = true;
      ROS_INFO_STREAM(name_ << " FINISHED " << dest_diff);
    } else {
      if (std::abs(heading_deviation) > M_PI_2)
        heading_deviation = Angle(heading_deviation + M_PI);

      cmdPublishOnce(
          1.0 * sign(dest_diff.asVector2().dot(curr_state_->oritUnit2())) * dest_diff.diff(),
          4.0 * delta_yaw + 1.0 * heading_deviation * dest_diff.norm()
      );
      ROS_INFO_STREAM(name_ << " TUNING ");
    }
  } else {
    double heading_deviation = Angle(std::atan2(interp_diff.y, interp_diff.x)) - curr_state_->yaw;
    ROS_INFO_STREAM(
        heading_deviation << ' ' << interp_diff << ' ' << Angle(std::atan2(interp_diff.y, interp_diff.x)) << ' '
                          << curr_state_->yaw);
    double vx = des.des_velocity(0), vw = des.des_velocity(1);
    if (vx < 0)
      heading_deviation = Angle::normalize(heading_deviation + M_PI);
    cmdPublishOnce(
        vx + 1.0 * sign(vx) * interp_diff.asVector2().dot(curr_state_->oritUnit2()),
        vw + 6.0 * static_cast<double>(interp_diff.yaw) + 2.0 * heading_deviation * interp_diff.norm()
    );
    ROS_INFO_STREAM(name_ << " TRACING " << model_.getWheelVelocity());
  }
}

void LocalPlannerSim::cmdPublishOnce(double vx, double vw) {
  ROS_INFO_STREAM("vx: " << vx << " vw: " << vw);
  model_.setLinearVelocity(vx);
  model_.setAngularVelocity(vw);
  const auto &v = model_.getWheelVelocity();
  ROS_INFO_STREAM("vw: " << vw << ' ' << v);
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
  left_wheel_velocity.data = v.first / Constants::WHEEL_RADIUS;
  right_wheel_velocity.data = v.second / Constants::WHEEL_RADIUS;
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
}

LocalPlannerSim::LocalPlannerSim(std::string name, const std::vector<std::pair<double, State>> &states)
    : LocalPlannerBase(std::move(name), states) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/left_wheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/right_wheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/" + name_ + "/robot_base", 1,
                                                  [=](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
}

void LocalPlannerSim::stateUpdate(const geometry_msgs::Pose::ConstPtr &p) {
  const auto &q = p->orientation;
  Angle yaw(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
  curr_state_ = std::make_unique<State>(p->position.x, p->position.y, yaw);
}

LocalPlannerReal::LocalPlannerReal(const std::string &name, const std::vector<std::pair<double, State>> &states) :
    LocalPlannerBase(name, states), odom_(name, states.front().second) {
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  timer_ = nh_.createTimer(ros::Rate(20), &LocalPlannerReal::stateUpdate, this);
}

void LocalPlannerReal::stateUpdate(const ros::TimerEvent &) {
  curr_state_ = std::make_unique<State>(odom_.getState());
}

void LocalPlannerReal::cmdPublishOnce(double vx, double vw) {
  model_.setLinearVelocity(vx);
  model_.setAngularVelocity(vw);
  geometry_msgs::Twist t;
  t.linear.x = model_.getLinearVelocity();
  t.angular.z = model_.getTurningAngle();
  cmd_pub_.publish(t);
}