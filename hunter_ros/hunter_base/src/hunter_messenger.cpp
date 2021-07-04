/* 
 * hunter_messenger.cpp
 * 
 * Created on: Apr 26, 2019 22:14
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "hunter_base/hunter_messenger.hpp"

#include "hunter_msgs/HunterStatus.h"

namespace wescore {
HunterROSMessenger::HunterROSMessenger(ros::NodeHandle nh) : hunter_(nullptr), nh_(nh) {}

HunterROSMessenger::HunterROSMessenger(HunterBase *hunter, ros::NodeHandle nh) : hunter_(hunter), nh_(nh) {}

void HunterROSMessenger::SetupSubscription() {
  // odometry publisher
  // odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_frame_, 50);
  status_publisher_ = nh_.advertise<hunter_msgs::HunterStatus>("/hunter_status", 10);

  // cmd subscriber
  motion_cmd_subscriber_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel",
                                                               5,
                                                               &HunterROSMessenger::TwistCmdCallback,
                                                               this); //不启用平滑包则订阅“cmd_vel”
  // light_cmd_subscriber_ = nh_.subscribe<hunter_msgs::HunterLightCmd>("/hunter_light_control", 5, &HunterROSMessenger::LightCmdCallback, this);
}

void HunterROSMessenger::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  if (!simulated_robot_) {
    hunter_->SetMotionCommand(msg->linear.x, msg->angular.z);
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
}

void HunterROSMessenger::GetCurrentMotionCmdForSim(double &linear, double &angular) {
  std::lock_guard<std::mutex> guard(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}
/*
void HunterROSMessenger::LightCmdCallback(const hunter_msgs::HunterLightCmd::ConstPtr &msg)
{
    if (!simulated_robot_)
    {
        if (msg->enable_cmd_light_control)
        {
            HunterLightCmd cmd;

            switch (msg->front_mode)
            {
            case hunter_msgs::HunterLightCmd::LIGHT_CONST_OFF:
            {
                cmd.front_mode = HunterLightCmd::LightMode::CONST_OFF;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_CONST_ON:
            {
                cmd.front_mode = HunterLightCmd::LightMode::CONST_ON;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_BREATH:
            {
                cmd.front_mode = HunterLightCmd::LightMode::BREATH;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_CUSTOM:
            {
                cmd.front_mode = HunterLightCmd::LightMode::CUSTOM;
                cmd.front_custom_value = msg->front_custom_value;
                break;
            }
            }

            switch (msg->rear_mode)
            {
            case hunter_msgs::HunterLightCmd::LIGHT_CONST_OFF:
            {
                cmd.rear_mode = HunterLightCmd::LightMode::CONST_OFF;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_CONST_ON:
            {
                cmd.rear_mode = HunterLightCmd::LightMode::CONST_ON;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_BREATH:
            {
                cmd.rear_mode = HunterLightCmd::LightMode::BREATH;
                break;
            }
            case hunter_msgs::HunterLightCmd::LIGHT_CUSTOM:
            {
                cmd.rear_mode = HunterLightCmd::LightMode::CUSTOM;
                cmd.rear_custom_value = msg->rear_custom_value;
                break;
            }
            }

            hunter_->SetLightCommand(cmd);
        }
        else
        {
            hunter_->DisableLightCmdControl();
        }
    }
    else
    {
        std::cout << "simulated robot received light control cmd" << std::endl;
    }
}
*/
void HunterROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = hunter_->GetHunterState();

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;
  status_msg.linear_velocity = state.linear_velocity;
  status_msg.angular_velocity = state.angular_velocity;

  status_msg.base_state = state.base_state;
  status_msg.control_mode = state.control_mode;
  status_msg.fault_code = state.fault_code;
  status_msg.battery_voltage = state.battery_voltage;

  for (int i = 1; i < 4; ++i)//只算三个电机,跟scout的是对应的
  {
    status_msg.motor_states[i].current = state.motor_states[i].current;
    status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
  }

  //   status_msg.light_control_enabled = state.light_control_enabled;
  //   status_msg.front_light_state.mode = state.front_light_state.mode;
  //   status_msg.front_light_state.custom_value = state.front_light_state.custom_value;
  //   status_msg.rear_light_state.mode = state.rear_light_state.mode;
  //   status_msg.rear_light_state.custom_value = state.front_light_state.custom_value;

  status_publisher_.publish(status_msg);

  // publish odometry and tf
  //   PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);

  // record time for next integration
  last_time_ = current_time_;
}
/* 
void HunterROSMessenger::PublishSimStateToROS(double linear, double angular)
{
    current_time_ = ros::Time::now();
    double dt = 1.0 / sim_control_rate_;

    // publish hunter state message
    hunter_msgs::HunterStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;

    // for (int i = 0; i < 4; ++i)
    // {
    //     status_msg.motor_states[i].current = state.motor_states[i].current;
    //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    //     status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
    // }

   // status_msg.light_control_enabled = false;
    // status_msg.front_light_state.mode = state.front_light_state.mode;
    // status_msg.front_light_state.custom_value = state.front_light_state.custom_value;
    // status_msg.rear_light_state.mode = state.rear_light_state.mode;
    // status_msg.rear_light_state.custom_value = state.front_light_state.custom_value;

    status_publisher_.publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(linear, angular, dt);
}*/
/*
void HunterROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{
    // perform numerical integration to get an estimation of pose
    linear_speed_ =
     linear;
    angular_speed_ = angular;

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_.publish(odom_msg);
}
*/
} // namespace wescore
