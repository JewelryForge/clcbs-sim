#ifndef CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
#define CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_

#include <utility>
#include <vector>
#include <tuple>
#include <functional>
#include "Angle.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

class State {
 public:
  State() : x(0), y(0), yaw(0) {};
  State(double x, double y, Angle yaw);
  State(double x, double y, double yaw);
  friend State operator+(const State &s1, const State &s2);
  friend State operator-(const State &s1, const State &s2);
  friend State operator*(const State &s1, double p);
  friend State operator*(double p, const State &s1);
  friend State operator/(const State &s1, double p);
  friend std::ostream &operator<<(std::ostream &os, const State &s);
  std::tuple<double, double, Angle> asTuple() const;
  Eigen::Vector2d asVector2() const;
  Eigen::Vector3d asVector3() const;
  Eigen::Vector2d oritUnit2() const;
  Eigen::Vector3d oritUnit3() const;
  static State interp(const State &s1, const State &s2, double ratio);
  double norm() const;
  double diff() const;
  double x, y;
  Angle yaw;
};

namespace Move {
using MoveType = unsigned char;
static const MoveType STOP = 0x00;
static const MoveType FORWARD = 0x01;
static const MoveType BACK = 0x02;
static const MoveType LEFT_TURN = 0x10;
static const MoveType RIGHT_TURN = 0x20;

std::string move2str(MoveType m);
};

struct Instruction {
  Move::MoveType operation = Move::STOP;
  State des_state;
  std::pair<double, double> des_velocity;
};

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  if (v.empty()) return os << "[]";
  os << '[' << v.front();
  for (auto iter = v.begin() + 1; iter != v.end(); ++iter) {
    os << '\t' << *iter;
  }
  return os << ']';
}
template<typename T1, typename T2>
std::ostream &operator<<(std::ostream &os, const std::pair<T1, T2> &p) {
  return os << '<' << p.first << ", " << p.second << '>';
}

struct Transition {
  Transition() = default;
  Transition(State state, std::pair<double, double> velocity, Move::MoveType move) :
      state(state), v(std::move(velocity)), move(move) {};
  friend std::ostream &operator<<(std::ostream &os, const Transition &t) {
    return os << "T<" << t.state << "," << t.v << "," << Move::move2str(t.move) << ">" << std::endl;
  };
  State state;
  std::pair<double, double> x, v;
  Move::MoveType move = Move::STOP; // next move
};

class StateManager {
 public:
  explicit StateManager(const std::vector<std::pair<double, State>> &states);
  void setAlignmentParam(double x, double y);
  const Instruction &operator()(double t);
  bool finished = false;
 protected:
  virtual void interpolateVelocity(int idx, double dt, const std::pair<double, Transition> &s_p,
                                   const std::pair<double, Transition> &s_n) {
    instruction_.des_velocity = s_n.second.v;
  }
  Instruction instruction_;
  std::vector<std::pair<double, Transition>> transitions_;
  std::function<State(const State &)> align;
};

class Poly3StateManager : public StateManager {
  void interpolateVelocity(int /* idx */, double dt, const std::pair<double, Transition> &s_p,
                           const std::pair<double, Transition> &s_n) override {
    double vl0, vr0, vlf, vrf, xl, xr, vl, vr;
    std::tie(vl0, vr0) = s_p.second.v;
    std::tie(vlf, vrf) = s_n.second.v;
    std::tie(xl, xr) = s_n.second.x;
    double period = s_n.first - s_p.first;
    vl = vl0 + 2 / pow(period, 2) * dt * (3 * xl - 2 * vl0 * period - vlf * period) +
        3 / pow(period, 3) * pow(dt, 2) * ((vl0 + vlf) * period - 2 * xl);
    vr = vr0 + 2 / pow(period, 2) * dt * (3 * xr - 2 * vr0 * period - vrf * period) +
        3 / pow(period, 3) * pow(dt, 2) * ((vr0 + vrf) * period - 2 * xr);
    instruction_.des_velocity = {vl, vr};
  }
};

class MinAccStateManager : public StateManager {
 public:
  explicit MinAccStateManager(const std::vector<std::pair<double, State>> &states) :
      StateManager(states) {
    Eigen::SparseMatrix<double> hessian, constrains;
    Eigen::VectorXd l_values, r_values;
    Eigen::VectorXd gradient;
    unsigned long k = transitions_.size() - 1;
    int number_of_variables = 6 * k;
    int number_of_constrains = 4 * k + 2;
    hessian.resize(number_of_variables, number_of_variables);
    constrains.resize(number_of_constrains, number_of_variables);
    l_values.resize(number_of_constrains);
    r_values.resize(number_of_constrains);
    gradient.resize(number_of_variables);
    l_values.setZero();
    r_values.setZero();
    gradient.setZero();

    int idx = 0, constrain_count = 0;
    for (auto curr = transitions_.begin(), next = curr + 1; next != transitions_.end(); curr = next, ++next, ++idx) {
      double dt = next->first - curr->first;
      double dt_pow[8]{1};
      for (int i = 1; i < 8; i++) dt_pow[i] = dt_pow[i - 1] * dt;
      hessian.insert(2 + 6 * idx, 2 + 6 * idx) = 4 * dt_pow[1];
      hessian.insert(3 + 6 * idx, 2 + 6 * idx) = 6 * dt_pow[2];
      hessian.insert(4 + 6 * idx, 2 + 6 * idx) = 8 * dt_pow[3];
      hessian.insert(5 + 6 * idx, 2 + 6 * idx) = 10 * dt_pow[4];
      hessian.insert(2 + 6 * idx, 3 + 6 * idx) = 6 * dt_pow[2];
      hessian.insert(3 + 6 * idx, 3 + 6 * idx) = 12 * dt_pow[3];
      hessian.insert(4 + 6 * idx, 3 + 6 * idx) = 18 * dt_pow[4];
      hessian.insert(5 + 6 * idx, 3 + 6 * idx) = 24 * dt_pow[5];
      hessian.insert(2 + 6 * idx, 4 + 6 * idx) = 8 * dt_pow[3];
      hessian.insert(3 + 6 * idx, 4 + 6 * idx) = 18 * dt_pow[4];
      hessian.insert(4 + 6 * idx, 4 + 6 * idx) = 144. / 5 * dt_pow[5];
      hessian.insert(5 + 6 * idx, 4 + 6 * idx) = 40 * dt_pow[6];
      hessian.insert(2 + 6 * idx, 5 + 6 * idx) = 10 * dt_pow[4];
      hessian.insert(3 + 6 * idx, 5 + 6 * idx) = 24 * dt_pow[5];
      hessian.insert(4 + 6 * idx, 5 + 6 * idx) = 40 * dt_pow[6];
      hessian.insert(5 + 6 * idx, 5 + 6 * idx) = 400. / 7 * dt_pow[7];
      constrains.insert(constrain_count++, 6 * idx) = 1; // x0
      if (idx == 0) {
        constrains.insert(constrain_count++, 1 + 6 * idx) = 1;
        constrains.insert(constrain_count++, 2 + 6 * idx) = 2;
      }
      for (int i = 0; i < 6; i++) constrains.insert(constrain_count, i + 6 * idx) = dt_pow[i]; // xt
      l_values(constrain_count) = curr->second.x.first;
      r_values(constrain_count) = curr->second.x.second;
      constrain_count++;
      if (idx < k - 1) {
        for (int i = 1; i < 6; i++) constrains.insert(constrain_count, i + 6 * idx) = i * dt_pow[i - 1]; // v continuity
        constrains.insert(constrain_count, 7 + 6 * idx) = -1;
        constrain_count++;
        for (int i = 2; i < 6; i++)
          constrains.insert(constrain_count, i + 6 * idx) = i * (i - 1) * dt_pow[i - 2]; // a continuity
        constrains.insert(constrain_count, 8 + 6 * idx) = -2;
        constrain_count++;
      } else {
        for (int i = 1; i < 6; i++) constrains.insert(constrain_count, i + 6 * idx) = i * dt_pow[i - 1]; // vf
        constrain_count++;
        for (int i = 2; i < 6; i++)
          constrains.insert(constrain_count, i + 6 * idx) = i * (i - 1) * dt_pow[i - 2]; // af
        constrain_count++;
      }
    }

    std::cout << hessian << std::endl;
    std::cout << constrains << std::endl;
    std::cout << l_values.transpose() << std::endl;
    std::cout << r_values.transpose() << std::endl;
    std::cout << k << std::endl;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(number_of_variables);
    solver.data()->setNumberOfConstraints(number_of_constrains);
    assert(solver.data()->setHessianMatrix(hessian));
    assert(solver.data()->setGradient(gradient));
    assert(solver.data()->setLinearConstraintsMatrix(constrains));
    assert(solver.data()->setLowerBound(l_values));
    assert(solver.data()->setUpperBound(l_values));
    assert(solver.initSolver());
    assert(solver.solve());
    left_params = solver.getSolution();
    std::cout << "SOLUTION: \n" << left_params.transpose() << std::endl;
    solver.clearSolver();
    assert(solver.data()->setLowerBound(r_values));
    assert(solver.data()->setUpperBound(r_values));
    assert(solver.initSolver());
    assert(solver.solve());
    right_params = solver.getSolution();
    std::cout << "SOLUTION: \n" << right_params.transpose() << std::endl;
  };
 private:
  void interpolateVelocity(int idx, double dt, const std::pair<double, Transition> &s_p,
                           const std::pair<double, Transition> &s_n) override {
    Eigen::Matrix<double, 1, 6> dt_pow, coeff;
    dt_pow.setOnes();
    coeff.setZero();
    for (int i = 1; i < 5; i++) dt_pow[i] = dt_pow[i - 1] * dt;
    for (int i = 1; i < 6; i++) coeff[i] = dt_pow[i - 1] * i;
//    std::cout << idx
    double vl = coeff * left_params.block<6, 1>(idx * 6, 0);
    double vr = coeff * right_params.block<6, 1>(idx * 6, 0);
    instruction_.des_velocity = {vl, vr};
  }
  Eigen::VectorXd left_params, right_params;
};
#endif //CLCBS_DRIVING_INCLUDE_STATEMANAGER_H_
