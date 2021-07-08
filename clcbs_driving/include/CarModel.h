#ifndef CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#define CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#include <tuple>
#include "Constants.h"
class CarModel {
 public:
  friend class FeedbackController;
  CarModel();
  explicit CarModel(double rotation_radius);
  void setThr(double thr);
  void setOrt(double ort);
  void setRad(double radius);
  void setVx(double vx);
  void setVw(double vw);
  void reset();
  double vx() const;
  double vw() const;
  const std::pair<double, double> getVelocity() const;
 private:
  double thr_, ort_;
  double vx_max_, rot_radius_, width_;
};


#endif //CLCBS_DRIVING_INCLUDE_CARMODEL_H_
