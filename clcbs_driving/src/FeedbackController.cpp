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
                                       const std::vector<std::pair<double, State>> &states)
    : name_(std::move(name)), model_(), pid1_(2, 0.1, 1.0), pid2_(2, 0.1, 1.0) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/left_wheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/" + name_ + "/right_wheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/" + name_ + "/robot_base", 1,
                                                  [this](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  state_manager_ = std::make_unique<MinAccStateManager>(states);
  state_manager_->setAlignmentParam(-Constants::MAP_SIZE_X / 2, -Constants::MAP_SIZE_Y / 2);
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
void FeedbackController::spin() {
  auto rate = ros::Rate(50);
  while (ros::ok()) {
    spinOnce();
    rate.sleep();
  }
}

void FeedbackController::spinOnce() {
  ros::spinOnce();
  if (curr_state_ != nullptr) {
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

  // TODO: CLOSE LOOP
  if (state_manager_->finished) {
    model_.reset();
    publishOnce({0.0, 0.0});
    ROS_INFO_STREAM("FINISHED");
  } else {
    double vl, vr;
    std::tie(vl, vr) = des.des_velocity;
    State diff_state = interp_state - *curr_state_;
    double heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - curr_state_->yaw;
//    Angle theta = curr_state_->yaw - des.des_state.yaw;
//    Angle des_yaw_deviation = heading_deviation - theta;
    double vx = (vl + vr) / 2, vw = (vr - vl) / Constants::CAR_WIDTH;
    double delta_yaw = des.des_state.yaw - curr_state_->yaw;
//    model_.setVx(vx + 0.2 * diff_state.norm());
//    model_.setVw(vw + 1.0 * heading_deviation + 1.0 * des_yaw_deviation);
    model_.setVx(vx + 1.0 * sign(vx) * diff_state.asVector2().dot(curr_state_->oritUnit2()));
    model_.setVw(vw + 4.0 * delta_yaw + 1.0 * heading_deviation * diff_state.norm());
//    vl -= 2.0 * delta_yaw * Constants::CAR_WIDTH / 2;
//    vr += 2.0 * delta_yaw * Constants::CAR_WIDTH / 2;
    std::tie(vl, vr) = model_.getVelocity();
    publishOnce(model_.getVelocity());
//    ROS_INFO_STREAM(state_manager_->start_state.yaw);
    ROS_INFO_STREAM(model_.ort_ << '\t' << vw << '\t' << delta_yaw << '\t' << heading_deviation);

//    State diff_state = interp_state - *curr_state_;
//    ROS_INFO_STREAM(vx << ' ' << vw);
//    model_.setVx(pid1_(vx - velocity_measured));
//    model_.setVx(vx);
//    model_.setVw(vw + pid2_(interp_state.yaw - curr_state_->yaw));
//    model_.setVw(vw);
//    double dist = std::hypot(diff_state.x, diff_state.y);
//    State instant_state = (state_manager_(dt + 0.1) - interp_state) / 0.1;
//    Angle heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - interp_state.yaw;
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

//    ROS_INFO_STREAM(name_ << ' ' << dt << ' ' << diff_state << '\t' << heading_deviation << '\t' << model_.vx() << '\t'
//                          << model_.vw());
  }
//  publishOnce(model_.getVelocity());
}

