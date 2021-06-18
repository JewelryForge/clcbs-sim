#include <csignal>
#include <functional>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "FeedbackContoller.h"

FeedbackContoller::FeedbackContoller(std::string name,
                                     std::vector<std::pair<double, State>> states,
                                     int argc,
                                     char **argv) :
    state_manager_(name, states) {
  ros::init(argc, argv, "FeedbackContoller");
  left_pub_ = nh_.advertise<std_msgs::Float64>("/agent_control/agent_leftwheel_controller/command", 1);
  right_pub_ = nh_.advertise<std_msgs::Float64>("/agent_control/agent_rightwheel_controller/command", 1);
  state_sub_ = nh_.subscribe<geometry_msgs::Pose>("/agent_states/agent1_robot_base", 1,
                                                  [this](auto &&PH1) { stateUpdate(std::forward<decltype(PH1)>(PH1)); });
  nh_.param("ROTATION_RADIUS", ROTATION_RADIUS, 3.0);
  double Kp, Ki, Kd;
  nh_.param("Kp", Kp, 2.0);
  nh_.param("Ki", Ki, 0.0);
  nh_.param("Kd", Kd, 0.0);
  state_manager_ = StateManager(std::move(name), std::move(states));
  signal(SIGINT, reset);
}
void FeedbackContoller::start() {
  is_started_ = true;
  t_start_ = ros::Time::now();
}
void FeedbackContoller::stateUpdate(const geometry_msgs::Pose::ConstPtr p) {
  auto &o = p->orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  Eigen::Vector3d ea = q.matrix().eulerAngles(2, 1, 0);
  curr_state_ = std::make_unique<State>(p->position.x, p->position.y, ea[0]);
}
void FeedbackContoller::spin() {
  int CONTROL_RATE;
  nh_.param("CONTROL_RATE", CONTROL_RATE, 20);
  auto rate = ros::Rate(CONTROL_RATE);
  while (ros::ok()) {
    if (curr_state_ != nullptr) {
      if (!is_started_) start();
      publishOnce();
      rate.sleep();
    }
  }
}
void FeedbackContoller::publishOnce() {
  double dt = (ros::Time::now() - t_start_).toSec();
  State des_state = state_manager_(dt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(des_state.x, des_state.y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, des_state.yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "desired_state"));
  State diff_state = des_state - *curr_state_;
//    static State instant_state = (state_manager_(dt + 0.1) - des_state) / 0.1;
  double dist = std::hypot(diff_state.x, diff_state.y);
  Angle heading_deviation = Angle(std::atan2(diff_state.y, diff_state.x)) - curr_state_->yaw;
  double des_yaw_deviation = des_state.yaw - curr_state_->yaw;
  model_.set_vx(5 * dist);
  model_.set_vw(8.0 * heading_deviation);
  std_msgs::Float64 left_wheel_velocity, right_wheel_velocity;
//    std::tie(left_wheel_velocity, right_wheel_velocity) = model_.getVelocity();
  left_pub_.publish(left_wheel_velocity);
  right_pub_.publish(right_wheel_velocity);
//    static PID<d>
}
void FeedbackContoller::reset(int) {
  system("/home/jewelry/catkin_ws/CLCBS/src/clcbs_driving/scripts/reset.py");
  ros::shutdown();
}
