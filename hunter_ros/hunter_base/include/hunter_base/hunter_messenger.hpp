/* 
 * hunter_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//#include "hunter_msgs/HunterLightCmd.h"
#include "hunter_base/hunter_base.hpp"

namespace wescore
{
class HunterROSMessenger
{
public:
    explicit HunterROSMessenger(ros::NodeHandle nh);
    HunterROSMessenger(HunterBase *hunter, ros::NodeHandle nh);

    std::string odom_frame_;
    std::string base_frame_;

    bool simulated_robot_;
    int sim_control_rate_;

    void SetupSubscription();
    void PublishStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);

private:
    HunterBase *hunter_;
    ros::NodeHandle nh_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Publisher status_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    //ros::Subscriber light_cmd_subscriber_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    //void LightCmdCallback(const hunter_msgs::HunterLightCmd::ConstPtr &msg);
    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace wescore

#endif /* HUNTER_MESSENGER_HPP */
