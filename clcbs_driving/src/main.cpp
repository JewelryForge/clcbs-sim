#include "CarModel.h"
#include <iostream>
#include "PID.hpp"
#include "Angle.h"
using namespace std;

int main() {
  auto a = Angle(0), b = Angle(1);

  auto pid = PID<Angle>(a, b);
  pid.set_param(1.0, 0.5);
  cout << pid() << endl;
  a = 0.1;
  cout << pid() << endl;
  a = 0.2;
  cout << pid() << endl;
  a = 0.3;
  cout << pid() << endl;
  a = 1.1;
  cout << pid() << endl;
}