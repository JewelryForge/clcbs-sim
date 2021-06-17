#ifndef CLCBS_DRIVING_INCLUDE_VELOCITYCONTROLLER_H_
#define CLCBS_DRIVING_INCLUDE_VELOCITYCONTROLLER_H_
#include <tuple>

class VelocityController {
 public:
  VelocityController();
  void acc();
  void dec();
  void ltn();
  void rtn();
  void set_vx(double vx);
  void set_vw(double vw);
  void reset();
 private:
  void updateVWMax();
  void restrain();
  double vx_{0.0}, vw_{0.0};
  double step_, rot_radius_;
  double vx_max_, vw_max_{0.0};
};

class PID {
 public:
  PID(const double &variable, const double &set_point) : variable_(variable), set_point_(set_point) {}
  void set_param(double Kp, double Ki = 0, double Kd = 0);
  double operator()() const;
 private:
  double Kp_{1.0}, Ki_{0.0}, Kd_{0.0};
  const double &set_point_, &variable_;
};
#endif //CLCBS_DRIVING_INCLUDE_VELOCITYCONTROLLER_H_
