#ifndef CLCBS_DRIVING_INCLUDE_PID_HPP_
#define CLCBS_DRIVING_INCLUDE_PID_HPP_

//#include <tuple>

//template<typename T>
//class PIDFixed {
// public:
//  PIDFixed(const T &variable, const T &set_point) : variable_(variable), set_point_(set_point) {}
//  void set_param(double Kp, double Ki = 0, double Kd = 0);
//  T operator()() const;
// private:
//  double Kp_{1.0}, Ki_{0.0}, Kd_{0.0};
//  const T &set_point_, &variable_;
//};
//
//template<typename T>
//void PIDFixed<T>::set_param(double Kp, double Ki, double Kd) {
//  std::tie(Kp_, Ki_, Kd_) = std::tie(Kp_, Ki, Kd);
//}
//
//template<typename T>
//T PIDFixed<T>::operator()() const {
//  static double u_p1 = 0.0, e_p1 = 0.0, e_p2 = 0.0;
//  double e = set_point_ - variable_;
//  double du = Kp_ * (e - e_p1) + Ki_ * e + Kd_ * (e - 2 * e_p1 + e_p2);
//  double u = u_p1 + du;
//  std::tie(u_p1, e_p1, e_p2) = std::make_tuple(u, e, e_p1);
//  return u;
//}
//
class PID {
 public:
  explicit PID(double Kp, double Ki = 0, double Kd = 0) : Kp_(Kp), Ki_(Ki), Kd_(Kd) {}
  double operator()(double e) const;
 private:
  double Kp_, Ki_, Kd_;
};


double PID::operator()(double e) const {
  static double u_p1 = 0.0, e_p1 = 0.0, e_p2 = 0.0;
  double du = Kp_ * (e - e_p1) + Ki_ * e + Kd_ * (e - 2 * e_p1 + e_p2);
  double u = u_p1 + du;
  std::tie(u_p1, e_p1, e_p2) = std::make_tuple(u, e, e_p1);
  return u;
}

#endif //CLCBS_DRIVING_INCLUDE_PID_HPP_
