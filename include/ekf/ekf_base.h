//
// Created by yezi on 2022/4/16.
//

#pragma once

#include <ros/ros.h>
#include <vector>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

class EkfBase
{
public:
  EkfBase() : Nx_(0), Nu_(0), Ny_(0)
  {
  }
  void init(std::vector<double> x0, std::vector<std::vector<double>> P0)
  {
  }

  void step(const std::vector<double>& y, const std::vector<double>& u, const ros::Time time)
  {
    // Prediction Step
    double dt = (time - last_update_time_).toSec();
    last_update_time_ = time;
    x_ = F_(DMDict({ { "x", x_ }, { "u", u }, { "dt", dt } })).at("xf");
    A_ = jac_f_(DMDict({ { "x", x_ }, { "u", u }, { "dt", dt } })).at("jac");
    P_ = mtimes(A_, mtimes(P_, A_.T())) + dt * R_;

    // (Innovation) Update Step / correction step
    C_ = jac_g_(DMDict({ { "x", x_ }, { "u", u } })).at("jac");
    y_pred_ = g_(DMDict({ { "x", x_ }, { "u", u } })).at("y");
    P12_ = mtimes(P_, C_.T());

    K_ = mtimes(P12_, inv(mtimes(C_, P12_) + Q_ / dt));
    x_ += mtimes(K_, (y - y_pred_));
    P_ -= mtimes(mtimes(K_, C_), P_);
  }
  void getState(std::vector<double>& x) const
  {
    x = vector<double>(this->x_);
  }
  void getCovariance(std::vector<std::vector<double>>& cov) const;

  virtual Function getF() = 0;
  virtual Function getG() = 0;

protected:
  ros::Time last_update_time_;

  const size_t Nx_;  // number of states
  const size_t Nu_;  // number of controls
  const size_t Ny_;  // number of measurements

  DM x_;  // current estimate of the state vector
  DM P_;  // current estimate of the covariance matrix

  DM R_;
  DM Q_;

  Function g_;
  Function jac_f_;  // jacobian of the system's ode with respect to x
  Function jac_g_;  // jacobian of measurement function

  Function F_;  // discrete time model of you the system

  // temporary variables
  DM A_, C_, P12_, K_, y_pred_;
};