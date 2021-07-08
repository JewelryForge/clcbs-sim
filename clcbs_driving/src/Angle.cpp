#include <iomanip>
#include <cmath>
#include "Angle.h"

Angle::Angle(double angle) : angle_(angle) { normalize(); }

Angle Angle::Degree(double degree) { return Angle(degree * M_PI / 180); }

Angle Angle::operator+=(Angle b) {
  angle_ += b.angle_;
  normalize();
  return *this;
}

Angle Angle::operator-=(Angle b) {
  angle_ -= b.angle_;
  normalize();
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Angle &a) {
  return os << std::setprecision(3) << a.angle_;
}

Angle::operator double() const { return angle_; }

void Angle::normalize() { angle_ = normalize(angle_); }

double Angle::toDeg() const { return angle_ / M_PI * 180; }

double Angle::normalize(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a <= -M_PI) a += 2 * M_PI;
  return a;
}

