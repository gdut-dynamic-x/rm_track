//
// Created by qiayuan on 2022/4/17.
//

// Reference: https://en.wikipedia.org/wiki/Extended_Kalman_filter#Discrete-time_predict_and_update_equations
//            https://github.com/schoelst/casadi-ekf

#pragma once

#include <vector>

#include <casadi/casadi.hpp>

namespace rm_track
{
using namespace std;
using namespace casadi;

class EkfBase
{
public:
  EkfBase();
  void predict(const vector<double>& y, const std::vector<double>& u, double dt);
  void update(const vector<double>& y, const std::vector<double>& u, double dt);

  virtual Function getF() = 0;
  virtual Function getG() = 0;

protected:
  const size_t n_x_;  // number of states
  const size_t n_u_;  // number of controls
  const size_t n_y_;  // number of measurements

  Function f_;      // discrete process model of the system
  Function g_;      // discrete measurement model of the system
  Function jac_f_;  // jacobian of the system's ode with respect to x
  Function jac_g_;  // jacobian of measurement model

  DM x_;  // current estimate of the state vector
  DM P_;  // current estimate of the covariance matrix

  DM R_;
  DM Q_;

  // temporary variables
  DM A_, C_, P12_, K_, y_pred_;
};

}  // namespace rm_track
