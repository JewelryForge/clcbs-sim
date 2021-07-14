#ifndef CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_
#define CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "GlobalPlanner.h"
#include "PlanVisualizer.h"
#include "CarModel.h"
#include "WheelOdometer.h"

class LocalPlannerBase {
 public:
  LocalPlannerBase(std::string name, const std::vector<std::pair<double, State>> &states);;
  bool isActive() const;
  void calculateVelocityAndPublish();
  static bool activateAll();
  virtual void cmdPublishOnce(double vx, double vw) = 0;
  void tfPublishOnce(const State &s);

 protected:
  void calculateVelocityAndPublishBase(double dt);

  static std::vector<LocalPlannerBase *> all_controller_;
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_; // TODO: CHANGE TO tf2

  std::unique_ptr<State> curr_state_;
  std::string name_;
  bool is_finished_ = false;
  std::unique_ptr<GlobalPlanner> state_manager_;
  ros::Time t_start_;
  CarModel model_;
};

class LocalPlannerSim : public LocalPlannerBase {
 public:
  LocalPlannerSim(std::string name, const std::vector<std::pair<double, State>> &states);

 private:
  void stateUpdate(const geometry_msgs::Pose::ConstPtr &p);
  void cmdPublishOnce(double vx, double vw) override;

  ros::Publisher left_pub_, right_pub_;
  ros::Subscriber state_sub_;
};

class LocalPlannerReal : public LocalPlannerBase {
 public:
  LocalPlannerReal(const std::string &name, const std::vector<std::pair<double, State>> &states);;
 private:
  void stateUpdate(const ros::TimerEvent &);
  void cmdPublishOnce(double vx, double vw) override;

  WheelOdometer odom_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;
};

#endif //CLCBS_DRIVING_INCLUDE_LOCALPLANNER_H_
