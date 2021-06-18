#include <csignal>
#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <PID.hpp>

#include "FeedbackController.h"

FeedbackController::FeedbackController(ros::NodeHandle &nh,
                                       std::string name,
                                       std::vector<std::pair<double, State>> states)
    : state_manager_(std::move(name), std::move(states)), model_(1.5) {
  left_pub_ = nh_.advertise<std_msgs::Float64>("/agent_control/agent_leftwheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/agent_control/agent_rightwheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/agent1_robot_base", 1,
                                                  [this](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  state_manager_.setAlignmentParam(-25, -25);
//  nh_.param("ROTATION_RADIUS", ROTATION_RADIUS, 3.0);
//  double Kp, Ki, Kd;
//  nh_.param("Kp", Kp, 2.0);
//  nh_.param("Ki", Ki, 0.0);
//  nh_.param("Kd", Kd, 0.0);
//  state_manager_ = StateManager(std::move(name), std::move(states));
//  signal(SIGINT, reset);
}
void FeedbackController::start() {
  is_started_ = true;
  t_start_ = ros::Time::now();
}
void FeedbackController::stateUpdate(const geometry_msgs::Pose::ConstPtr &p) {
  auto &o = p->orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  double yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.x() * q.x()) + q.y() * q.y());
  curr_state_ = std::make_unique<State>(p->position.x, p->position.y, yaw);
}
void FeedbackController::spin() {
  int CONTROL_RATE;
  nh_.param("CONTROL_RATE", CONTROL_RATE, 50);
  auto rate = ros::Rate(CONTROL_RATE);
  while (ros::ok()) {
    ros::spinOnce();
    if (curr_state_ != nullptr) {
      if (!is_started_) start();
      publishOnce();
      rate.sleep();
    }
  }
}
void FeedbackController::publishOnce() {
  double dt = (ros::Time::now() - t_start_).toSec();
  State des_state = state_manager_(dt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(des_state.x, des_state.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, des_state.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "desired_state"));
  if (state_manager_.finished) {
    model_.reset();
    ROS_INFO_STREAM("FINISHED");
  } else {
    State diff_state = des_state - *curr_state_;
    double dist = std::hypot(diff_state.x, diff_state.y);
//    State instant_state = (state_manager_(dt + 0.1) - des_state) / 0.1;
    Angle heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - curr_state_->yaw;
    Angle des_yaw_deviation = des_state.yaw - curr_state_->yaw;
    if (abs(heading_deviation) > M_PI / 3) {
      dist *= -0.5;
//      ROS_WARN_STREAM("ABOUT TO TRANSCEND\t" << dist << '\t' << heading_deviation);
//      ROS_WARN_STREAM("TRANSCEND\t" << dist << '\t' << heading_deviation);
    }
    if (model_.vx() < 0) {
      heading_deviation += M_PI;
      des_yaw_deviation += M_PI;
    }
    static PID pid(0.6, 0.0, 0.0);
    model_.setThr(pid(dist));
//    if (dist > 1e-1) {
      model_.setOrt(0.3 * heading_deviation + 0.6 * des_yaw_deviation);
    // TODO: TRY ADVANCED FEEDBACK ALGORITHM OR CHANGE INTERPOLATION ALGORITHM
//    } else {
//      model_.set_vw(2.0 * des_yaw_deviation);
//    }
    ROS_INFO_STREAM(dt << diff_state << '\t' << heading_deviation << '\t' << model_.vx() << '\t' << model_.vw());
  }
  auto v = model_.getVelocity();
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
  left_wheel_velocity.data = v.first;
  right_wheel_velocity.data = v.second;
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
}
void FeedbackController::reset(int) {
  system("/home/jewelry/catkin_ws/CLCBS/src/clcbs_driving/scripts/reset.py");
  ros::shutdown();
}
