//
// Created by jewelry on 17/06/2021.
//

#ifndef CLCBS_DRIVING_INCLUDE_ANGLE_H_
#define CLCBS_DRIVING_INCLUDE_ANGLE_H_

#include <cmath>
#include <iostream>

class Angle {
 public:
  Angle(double angle);
  static Angle Degree(double degree);

  friend Angle operator+(const Angle &a, const Angle &b);
  friend Angle operator-(const Angle &a, const Angle &b);
  friend Angle operator*(const Angle &a, double p);
  friend Angle operator*(double p, const Angle &a);
  friend Angle operator/(const Angle &a, double p);
  Angle operator+=(Angle b);
  Angle operator-=(Angle b);
  Angle operator*=(double p);
  Angle operator/=(double p);
  friend std::ostream &operator<<(std::ostream &os, const Angle &a);
  operator double() const;

  void normalize();
  double toDeg() const;
 private:
  double angle_;
};

#endif //CLCBS_DRIVING_INCLUDE_ANGLE_H_
