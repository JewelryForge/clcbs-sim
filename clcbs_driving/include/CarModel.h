#ifndef CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#define CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#include <tuple>

class CarModel {
 public:
  explicit CarModel(double min_rot_radius = 3.0);
  void setThr(double thr);
  void setOrt(double ort);
  void setRad(double radius);
  void setVx(double vx);
  void setVw(double vw);
  void reset();
  double vx() const;
  double vw() const;
  std::pair<double, double> getVelocity() const;
 private:
  double thr_{0.}, ort_{0.};
  double vx_max_{3.}, step_, rot_radius_, width_;
};


#endif //CLCBS_DRIVING_INCLUDE_CARMODEL_H_
