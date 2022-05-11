//
// Created by qiayuan on 2022/4/17.
//

// Reference: https://en.wikipedia.org/wiki/Extended_Kalman_filter#Discrete-time_predict_and_update_equations
//            https://github.com/schoelst/casadi-ekf

#pragma once

#include <vector>
#include <ros/ros.h>
#include <casadi/casadi.hpp>

namespace rm_track
{
using namespace std;
using namespace casadi;

class EkfBase
{
public:
  virtual void init(ros::NodeHandle& nh) = 0;
  void predict(const DM& u, double dt);
  void update(const DM& y, const DM& u, double dt);
  void setInitialGuess(const DM& x0, const DM& p0);
  void getQR(const ros::NodeHandle& nh);
  void setNoise(const DM& Q, const DM& R);
  DM getState() const;
  DM getCovariance() const;
  bool inited_ = false;

protected:
  void setup(const Function& f, const Function& g);
  DM getMatrix(const ros::NodeHandle& nh, const std::string& name, int dimension);

  Function f_;      // discrete process model of the system
  Function g_;      // discrete measurement model of the system
  Function jac_f_;  // jacobian of the system's ode with respect to x
  Function jac_g_;  // jacobian of measurement model

  DM x_;  // current estimate of the state vector
  DM p_;  // current estimate of the covariance matrix
  DM r_;
  DM q_;
};

}  // namespace rm_track
