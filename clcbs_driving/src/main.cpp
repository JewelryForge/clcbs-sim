#include "VelocityController.h"
#include <iostream>
using namespace std;

int main() {
  double a = 0, b = 1;
  auto pid = PID(a, b);
  pid.set_param(2.0, 1.0);
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