#include <iomanip>
#include <cmath>
#include "Angle.h"

Angle::Angle(double angle) : angle_(angle) {
  normalize();
}
Angle Angle::Degree(double degree) {
  return {degree * M_PI / 180};
}
Angle operator+(const Angle &a, const Angle &b) {
  Angle c(a.angle_ + b.angle_);
  c.normalize();
  return c;
}
Angle Angle::operator+=(Angle b) {
  angle_ += b.angle_;
  normalize();
  return *this;
}
Angle operator-(const Angle &a, const Angle &b) {
  Angle c(a.angle_ - b.angle_);
  c.normalize();
  return c;
}
Angle Angle::operator-=(Angle b) {
  angle_ -= b.angle_;
  normalize();
  return *this;
}
Angle operator*(const Angle &a, double p) {
  Angle c(a.angle_ * p);
  c.normalize();
  return c;
}
Angle operator*(double p, const Angle &a) {
  return a * p;
}
Angle Angle::operator*=(double p) {
  angle_ *= p;
  return *this;
}
Angle operator/(const Angle &a, double p) {
  Angle c(a.angle_ / p);
  c.normalize();
  return c;
}
Angle Angle::operator/=(double p) {
  angle_ /= p;
  return *this;
}
std::ostream &operator<<(std::ostream &os, const Angle &a) {
  return os << std::setprecision(3) << a.angle_;
}
Angle::operator double() const {
  return angle_;
}
void Angle::normalize() {
  while (angle_ > M_PI) angle_ -= 2 * M_PI;
  while (angle_ <= -M_PI) angle_ += 2 * M_PI;
}
double Angle::toDeg() const {
  return angle_ / M_PI * 180;
}

