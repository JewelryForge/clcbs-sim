#ifndef CLCBS_DRIVING_INCLUDE_ANGLE_H_
#define CLCBS_DRIVING_INCLUDE_ANGLE_H_

#include <iostream>
#include <cmath>

class Angle {
 public:
  Angle() = default;
  explicit Angle(double angle) : angle_(angle) { normalize(); }
  static Angle Degree(double degree) { return Angle(degree * M_PI / 180); }
  static double normalize(double a);

  friend Angle operator+(const Angle &a, const Angle &b) { return Angle(a.angle_ + b.angle_); }
  friend Angle operator+(const Angle &a, double b) { return a + Angle(b); }
  friend Angle operator+(double b, const Angle &a) { return a + b; }

  friend Angle operator-(const Angle &a) { return Angle(-a.angle_); }
  friend Angle operator-(const Angle &a, const Angle &b) { return a + (-b); }
  friend Angle operator-(const Angle &a, double b) { return a - Angle(b); }

  template<typename T>
  friend Angle operator*(const Angle &a, T p) { return Angle(a.angle_ * p); }
  template<typename T>
  friend Angle operator*(T p, const Angle &a) { return a * p; }
  template<typename T>
  friend Angle operator/(const Angle &a, T p) { return a * (1.0 / p); }

  friend bool operator==(Angle a, Angle b) { return a.angle_ == b.angle_; }
  friend bool operator==(Angle a, double b) { return a.angle_ == b; }
  friend bool operator==(double a, Angle b) { return b == a; }

  Angle operator+=(Angle b);
  Angle operator+=(double b) { return operator+=(Angle(b)); }
  template<typename T>
  Angle operator-=(T b) { return operator+=(-b); }
  template<typename T>
  Angle operator*=(T p);
  template<typename T>
  Angle operator/=(T p) { return operator*=(1.0 / p); }

  friend std::ostream &operator<<(std::ostream &os, const Angle &a);
  operator double() const { return angle_; }

  void normalize() { angle_ = normalize(angle_); }
  double toDeg() const { return angle_ / M_PI * 180; }
 private:
  double angle_;
};

template<typename T>
Angle Angle::operator*=(T p) {
  angle_ *= p;
  normalize();
  return *this;
}


#endif //CLCBS_DRIVING_INCLUDE_ANGLE_H_
