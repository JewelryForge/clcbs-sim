#include "Angle.hpp"
#include <iomanip>

double Angle::normalize(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a <= -M_PI) a += 2 * M_PI;
  return a;
}

Angle Angle::operator+=(Angle b) {
  angle_ += b.angle_;
  normalize();
  return *this;
}

std::ostream &operator<<(std::ostream &os, const Angle &a) {
  return os << std::setprecision(3) << a.angle_;
}