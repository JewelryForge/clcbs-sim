#include "StateManager.h"
#include <cassert>
#include <iomanip>
#include <utility>
#include <cmath>
#include "Constants.h"

State::State(double x, double y, Angle yaw) : x(x), y(y), yaw(yaw) {}
State::State(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
State operator+(const State &s1, const State &s2) {
  return {s1.x + s2.x, s1.y + s2.y, s1.yaw + s2.yaw};
}
State operator-(const State &s1, const State &s2) {
  return {s1.x - s2.x, s1.y - s2.y, s1.yaw - s2.yaw};
}
State operator*(const State &s1, double p) {
  return {s1.x * p, s1.y * p, s1.yaw * p};
}
State operator*(double p, const State &s1) {
  return s1 * p;
}
State operator/(const State &s1, double p) {
  return s1 * (1 / p);
}
std::ostream &operator<<(std::ostream &os, const State &s) {
  return os << std::setprecision(2) << "ST<" << s.x << ' ' << s.y << ' ' << s.yaw << '>';
}
State State::interp(const State &s1, const State &s2, double ratio) {
  return {s1.x + (s2.x - s1.x) * ratio, s1.y + (s2.y - s1.y) * ratio, s1.yaw + (s2.yaw - s1.yaw) * ratio};
}
double State::norm() const {
  return std::hypot(x, y);
}
double State::diff() const {
  return yaw == 0. ? norm() : (norm() / (2 * sin(yaw / 2)) * yaw);
}
std::tuple<double, double, Angle> State::asTuple() const {
  return {x, y, yaw};
}
Eigen::Vector2d State::asVector2() const {
  return {x, y};
}
Eigen::Vector3d State::asVector3() const {
  return {x, y, 0};
}
Eigen::Vector2d State::oritUnit2() const {
  return {cos(yaw), sin(yaw)};
}
Eigen::Vector3d State::oritUnit3() const {
  return {cos(yaw), sin(yaw), 0};
}
StateManager::StateManager(const std::vector<std::pair<double, State>> &states)
    : start_state(states.front().second),
      terminal_state(states.back().second),
      align([](const State &s) { return s; }) {
  assert(states.size() >= 2 and states.front().first == 0);
  logs_.reserve(states.size());
  for (auto curr = states.begin(), next = curr + 1;; curr = next, ++next) {
    if (next == states.end()) {
      logs_.emplace_back(curr->first, Transition(curr->second, {0.0, 0.0}, Move::STOP));
      break;
    }
    Transition t;
    t.state = curr->second;
    double dt = next->first - curr->first;
    auto diff_state = next->second - curr->second;
    if (diff_state.norm() == 0.0) {
      t.move = Move::STOP;
      t.x.setZero();
    } else if (diff_state.asVector2().dot(curr->second.oritUnit2()) > 0) {
      t.move = Move::FORWARD;
      t.x.setConstant(diff_state.diff());
    } else {
      t.move = Move::BACK;
      t.x.setConstant(diff_state.diff() * -1);
    }
    if (diff_state.yaw != 0) {
      double diff_x = diff_state.yaw * (Constants::CAR_WIDTH / 2);
      if (diff_state.yaw > 0) {
        t.move |= Move::LEFT_TURN;
        t.x += Eigen::Vector2d(-diff_x, diff_x);
      } else {
        t.move |= Move::RIGHT_TURN;
        t.x += Eigen::Vector2d(-diff_x, diff_x);
      }
    }

    if (curr == states.begin()) t.v = {0.0, 0.0};
    else t.v = t.x / dt;

    logs_.emplace_back(curr->first, t);
  }
//  std::cout << logs << std::endl;
}
void StateManager::interpolateVelocity(int idx, double dt,
                                       const std::pair<double, Transition> &s_p,
                                       const std::pair<double, Transition> &s_n) {
  instruction_.des_velocity = s_n.second.v;
}

void StateManager::setAlignmentParam(double x, double y) {
  align = [=](const State &s) { return State(s.x + x, s.y + y, s.yaw); };
}

const Instruction &StateManager::operator()(double t) {
  if (t <= 0) {
    instruction_.operation = Move::STOP;
    instruction_.des_state = instruction_.local_dest = logs_.front().second.state; // TODO: REMOVE INTERP_STATE
    instruction_.des_velocity = {0.0, 0.0};
  } else if (t <= logs_.back().first) {
    int idx = -1;
    for (auto iter = logs_.begin(); iter != logs_.end(); ++iter, ++idx) {
      if (iter->first < t) continue;
      auto s_p = iter - 1, s_n = iter;
      double period = s_n->first - s_p->first, dt = t - s_p->first;
      instruction_.local_dest = s_n->second.state;
      instruction_.des_state = State::interp(s_p->second.state, s_n->second.state, dt / period);
      interpolateVelocity(idx, dt, *s_p, *s_n);
      break;
    }
  } else {
    finished = true;
    instruction_.operation = Move::STOP;
    instruction_.des_state = instruction_.local_dest = logs_.back().second.state;
    instruction_.des_velocity = {0.0, 0.0};
  }
  instruction_.des_state = align(instruction_.des_state);
  instruction_.local_dest = align(instruction_.local_dest);
  instruction_.global_dest = align(logs_.back().second.state);
  return instruction_;
}

std::string Move::move2str(Move::MoveType m) {
  switch (m) {
    case STOP: return "SP";
    case FORWARD: return "FW";
    case FORWARD | LEFT_TURN: return "LF";
    case FORWARD | RIGHT_TURN: return "RF";
    case BACK: return "BK";
    case BACK | LEFT_TURN: return "LB";
    case BACK | RIGHT_TURN: return "RB";
    default: throw std::invalid_argument("Invalid Move");
  }
}

std::ostream &operator<<(std::ostream &os, const Transition &t) {
  return os << "T<" << t.state << "," << t.v << "," << Move::move2str(t.move) << ">" << std::endl;
}

void Poly3StateManager::interpolateVelocity(int,
                                            double dt,
                                            const std::pair<double, Transition> &s_p,
                                            const std::pair<double, Transition> &s_n) {
  const auto &v0 = s_p.second.v, vf = s_n.second.v, x = s_n.second.x;
  double period = s_n.first - s_p.first;
  for (int i = 0; i < 2; i++) {
    instruction_.des_velocity(i) = v0(i) + 2 / pow(period, 2) * dt * (3 * x(i) - 2 * v0(i) * period - vf(i) * period) +
        3 / pow(period, 3) * pow(dt, 2) * ((v0(i) + vf(i)) * period - 2 * x(i));
  }
}

MinAccStateManager::MinAccStateManager(const std::vector<std::pair<double, State>> &states) :
    StateManager(states), init_yaw(states.front().second.yaw) {
  Eigen::SparseMatrix<double> hessian, constrains;
  Eigen::Matrix<double, -1, 2> constants;
  Eigen::VectorXd gradient;
  unsigned long k = logs_.size() - 1;
  int number_of_variables = 6 * k;
  int number_of_constrains = 4 * k + 2;
  hessian.resize(number_of_variables, number_of_variables);
  constrains.resize(number_of_constrains, number_of_variables);
  constants.resize(number_of_constrains, 2);
  gradient.resize(number_of_variables);
  constants.setZero();
  gradient.setZero();

  const static double coeff[4][4]{
      {4, 6, 8, 10},
      {6, 12, 18, 24},
      {8, 18, 144. / 5, 40},
      {10, 24, 40, 400. / 7}
  };
  int idx = 0, constrain_counter = 0;
  Eigen::Vector2d accumulated_x = Eigen::Vector2d::Zero();
  for (auto curr = logs_.begin(), next = curr + 1; next != logs_.end(); curr = next, ++next, ++idx) {
    double dt = next->first - curr->first;
    double dt_pow[8]{1};
    for (int i = 1; i < 8; i++) dt_pow[i] = dt_pow[i - 1] * dt;
    int shifting = 6 * idx;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++)
        hessian.insert(2 + j + shifting, 2 + i + shifting) = coeff[i][j] * dt_pow[i + j + 1];
    }
    constrains.insert(constrain_counter, shifting) = 1; // x0
    constants.block<1, 2>(constrain_counter, 0) = accumulated_x.transpose();
    constrain_counter++;
    if (idx == 0) {
      constrains.insert(constrain_counter++, 1 + shifting) = 1;
      constrains.insert(constrain_counter++, 2 + shifting) = 2;
    }
    for (int i = 0; i < 6; i++) constrains.insert(constrain_counter, i + shifting) = dt_pow[i]; // xt
    accumulated_x += curr->second.x;
    constants.block<1, 2>(constrain_counter++, 0) = accumulated_x.transpose();
    if (idx < k - 1) {
      for (int i = 1; i < 6; i++) constrains.insert(constrain_counter, i + shifting) = i * dt_pow[i - 1]; // v continuity
      constrains.insert(constrain_counter++, 7 + shifting) = -1;
      for (int i = 2; i < 6; i++)
        constrains.insert(constrain_counter, i + shifting) = i * (i - 1) * dt_pow[i - 2]; // a continuity
      constrains.insert(constrain_counter++, 8 + shifting) = -2;
    } else {
      for (int i = 1; i < 6; i++) constrains.insert(constrain_counter, i + shifting) = i * dt_pow[i - 1]; // vf
      constrain_counter++;
      for (int i = 2; i < 6; i++)
        constrains.insert(constrain_counter, i + shifting) = i * (i - 1) * dt_pow[i - 2]; // af
      constrain_counter++;
    }
  }

//  std::cout << hessian << std::endl;
//  std::cout << constrains << std::endl;
//  std::cout << l_values.transpose() << std::endl;
//  std::cout << r_values.transpose() << std::endl;
//  std::cout << k << std::endl;

  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfVariables(number_of_variables);
  solver.data()->setNumberOfConstraints(number_of_constrains);
  assert(solver.data()->setHessianMatrix(hessian));
  assert(solver.data()->setGradient(gradient));
  assert(solver.data()->setLinearConstraintsMatrix(constrains));
  assert(solver.data()->setLowerBound(constants.col(0)));
  assert(solver.data()->setUpperBound(constants.col(0)));
  assert(solver.initSolver());
  assert(solver.solve());
  left_params = solver.getSolution();
//  std::cout << "SOLUTION: \n" << left_params.transpose() << std::endl;
  solver.clearSolver();
  assert(solver.data()->setLowerBound(constants.col(1)));
  assert(solver.data()->setUpperBound(constants.col(1)));
  assert(solver.initSolver());
  assert(solver.solve());
  right_params = solver.getSolution();
//  std::cout << "SOLUTION: \n" << right_params.transpose() << std::endl;
}

void MinAccStateManager::interpolateVelocity(int idx, double dt,
                                             const std::pair<double, Transition> &s_p,
                                             const std::pair<double, Transition> &s_n) {
  Eigen::Matrix<double, 1, 6> dt_pow, coeff;
  dt_pow.setOnes();
  coeff.setZero();
  for (int i = 1; i < 5; i++) dt_pow[i] = dt_pow[i - 1] * dt;
  for (int i = 1; i < 6; i++) coeff[i] = dt_pow[i - 1] * i;
  int shifting = idx * 6;
  double vl = coeff * left_params.block<6, 1>(shifting, 0);
  double xl = dt_pow * left_params.block<6, 1>(shifting, 0);
  double vr = coeff * right_params.block<6, 1>(shifting, 0);
  double xr = dt_pow * right_params.block<6, 1>(shifting, 0);
  instruction_.des_velocity = {vl, vr};
  instruction_.des_state.yaw = (xr - xl) / Constants::CAR_WIDTH + init_yaw;
}
