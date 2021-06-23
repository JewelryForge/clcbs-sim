#include <csignal>
#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <PID.hpp>

#include "FeedbackController.h"

FeedbackController::FeedbackController(ros::NodeHandle &nh, std::string name,
                                       std::vector<std::pair<double, State>> states)
    : name_(std::move(name)), state_manager_(std::move(states)), model_(), pid1_(2, 0.1, 1.0), pid2_(2, 0.1, 1.0) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/left_wheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/right_wheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/" + name_ + "/robot_base", 1,
                                                  [this](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  state_manager_.setAlignmentParam(-Constants::MAP_SIZE_X / 2, -Constants::Y_MAP_SIZE / 2);
}
void FeedbackController::start() {
  is_started_ = true;
  t_start_ = ros::Time::now();
}
void FeedbackController::stateUpdate(const geometry_msgs::Pose::ConstPtr &p) {
  prev_state_ = curr_state_;
  auto &o = p->orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  Angle yaw(atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.x() * q.x()) + q.y() * q.y()));
  curr_state_ = std::make_shared<State>(p->position.x, p->position.y, yaw);
  if (prev_state_ != nullptr) {
    velocity_measured = (*curr_state_ - *prev_state_).diff();
  }
}
void FeedbackController::spin() {
  int PUBLISHING_FREQUENCY;
  nh_.param("PUBLISHING_FREQUENCY", PUBLISHING_FREQUENCY, 50);
  auto rate = ros::Rate(PUBLISHING_FREQUENCY);
  while (ros::ok()) {
    spinOnce();
    rate.sleep();
  }
}
void FeedbackController::spinOnce() {
  ros::spinOnce();
  if (prev_state_ != nullptr) {
    if (!is_started_) start();
    calculateVelocityAndPublish();
  }
}

void FeedbackController::publishOnce(const std::pair<double, double> &v) {
  const double WHEEL_RADIUS = 0.5; // TODO: CHANGE TO ROS_PARAM
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
  left_wheel_velocity.data = v.first / WHEEL_RADIUS;
  right_wheel_velocity.data = v.second / WHEEL_RADIUS;
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
}

void FeedbackController::calculateVelocityAndPublish() {
  double dt = (ros::Time::now() - t_start_).toSec();
  State des_state = state_manager_(dt);
  double vx, vw;
  std::tie(vx, vw) = state_manager_.getInstruction(dt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(des_state.x, des_state.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, des_state.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name_ + "_desired_state"));
  if (state_manager_.finished) {
    model_.reset();
    ROS_INFO_STREAM("FINISHED");
  } else {
    State diff_state = des_state - *curr_state_;
    ROS_INFO_STREAM(vx << ' ' << vw);
//    model_.setVx(pid1_(vx - velocity_measured));
    model_.setVx(vx);
//    model_.setVw(vw + pid2_(des_state.yaw - curr_state_->yaw));
    model_.setVw(vw);
//    double dist = std::hypot(diff_state.x, diff_state.y);
//    State instant_state = (state_manager_(dt + 0.1) - des_state) / 0.1;
//    Angle heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - des_state.yaw;
//    Angle des_yaw_deviation = heading_deviation - curr_state_->yaw;
//
//    if (abs(heading_deviation) > M_PI / 2) {
//      dist *= -1;
//    }
//
//    if (model_.vx() < 0) {
//      heading_deviation += M_PI;
//      des_yaw_deviation += M_PI;
//    }
//
//    double rho = dist, beta = heading_deviation, alpha = des_yaw_deviation;
//    double k1 = 0.5, k2 = 1;
//    double kappa_1 = k2 * (alpha - atan(-k1 * beta));
//    double kappa_2 = (1 + k1 / (1 + pow(k1 * beta, 2))) * sin(alpha);
//    double kappa = (kappa_1 + kappa_2) / rho;
//    double mu = 1, lambda = 1;
//    model_.setThr(1 / (1 + mu * pow(abs(kappa), lambda)));
//    model_.setRad(kappa);
    // TODO: TRY ADVANCED FEEDBACK ALGORITHM OR CHANGE INTERPOLATION ALGORITHM
//    ROS_INFO_STREAM(name_ << ' ' << dt << ' ' << diff_state << '\t' << heading_deviation << '\t' << model_.vx() << '\t'
//                          << model_.vw());
  }
  publishOnce(model_.getVelocity());
}

