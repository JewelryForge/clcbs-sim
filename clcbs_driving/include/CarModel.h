#ifndef CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#define CLCBS_DRIVING_INCLUDE_CARMODEL_H_
#include <tuple>

class CarModel {
 public:
  CarModel();
  void acc();
  void dec();
  void ltn();
  void rtn();
  void set_vx(double vx);
  void set_vw(double vw);
  void reset();
  std::pair<double, double> getVelocity() const;
 private:
  void updateVWMax();
  void restrain();
  double vx_{0.0}, vw_{0.0};
  double step_, rot_radius_;
  double vx_max_, vw_max_{0.0};
};


#endif //CLCBS_DRIVING_INCLUDE_CARMODEL_H_
