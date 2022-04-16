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
  EkfBase(const Function& f, const Function& g);

  void predict(const DM& u, double dt);
  void update(const DM& y, const DM& u, double dt);

  void setInitialGuess(const DM& x0, const DM& p0);
  void setNoise(const DM& Q, const DM& R);
  DM& getState() const;
  DM& getCovariance() const;

  virtual Function getF() = 0;
  virtual Function getG() = 0;

protected:
  Function f_;      // discrete process model of the system
  Function g_;      // discrete measurement model of the system
  Function jac_f_;  // jacobian of the system's ode with respect to x
  Function jac_g_;  // jacobian of measurement model

  DM x_;  // current estimate of the state vector
  DM P_;  // current estimate of the covariance matrix

  DM R_;
  DM Q_;
};

}  // namespace rm_track
