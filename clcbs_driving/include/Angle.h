#ifndef CLCBS_DRIVING_INCLUDE_ANGLE_H_
#define CLCBS_DRIVING_INCLUDE_ANGLE_H_

#include <iostream>

class Angle {
 public:
  Angle() = default;
  explicit Angle(double angle);
  static Angle Degree(double degree);

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
  Angle operator+=(double b) { return operator+=(Angle(b)); };

  Angle operator-=(Angle b);
  Angle operator-=(double b) { return operator-=(Angle(b)); };
  template<typename T>
  Angle operator*=(T p) {
    angle_ *= p;
    normalize();
    return *this;
  };
  template<typename T>
  Angle operator/=(T p){
    angle_ /= p;
    normalize();
    return *this;
  };
  friend std::ostream &operator<<(std::ostream &os, const Angle &a);
  operator double() const;

  void normalize();
  double toDeg() const;
 private:
  double angle_;
};

#endif //CLCBS_DRIVING_INCLUDE_ANGLE_H_
