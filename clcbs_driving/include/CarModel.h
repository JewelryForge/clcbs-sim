#ifndef CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#define CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#include <tuple>
#include "Constants.h"

// An Ackerman chassis expressed by linear velocity and front Wheel inside turning angle
class CarModel {
 public:
  CarModel();
  CarModel(double car_width, double wheel_base, double max_linear_velocity, double max_turning_angle);
  void setTurningAngle(double turning);
  void setLinearVelocity(double vx);
  void setAngularVelocity(double vw);
  void reset();
  double getLinearVelocity() const;
  double getTurningAngle() const;
  double getAngularVelocity() const;
  std::pair<double, double> getWheelVelocity() const;
 private:
  double linear_, turning_;
  double car_width_, wheel_base_;
  double max_linear_, max_turning_, min_rotation_radius_;
};

#endif //CLCBS_DRIVING_INCLUDE_CARMODEL_H_
