//
// Created by qiayuan on 2022/4/17.
//
#include "rm_track/ekf/ekf_base.h"

namespace rm_track
{
EkfBase::EkfBase(const Function& f, const Function& g) : f_(f), g_(g)
{
  SX x = SX::sym("x", f_.size1_in(0));
  SX u = SX::sym("u", f_.size1_in(1));
  SX dt = SX::sym("dt", 1);
  SX x_next = f_(SXDict({ { "x", x }, { "u", u }, { "dt", dt } })).at("x_next");
  SX z = g_(SXDict({ { "x", x }, { "u", u } })).at("z");
  SX jac_f = SX::jacobian(x_next, x);
  SX jac_g = SX::jacobian(z, x);

  jac_f_ = Function("jac_f", { { "x", x }, { "u", u }, { "dt", dt }, { "jac", jac_f } }, { "x", "u", "dt" }, { "jac" });
  jac_g_ = Function("jac_g", { { "x", x }, { "u", u }, { "jac", jac_g } }, { "x", "u" }, { "jac" });
}

void EkfBase::predict(const DM& u, double dt)
{
  // Get state transition matrices F
  DM f = jac_f_(DMDict({ { "x", x_ }, { "u", u }, { "dt", dt } })).at("jac");
  // Predicted state estimate
  x_ = f_(DMDict({ { "x", x_ }, { "u", u }, { "dt", dt } })).at("x_next");
  // Predicted covariance estimate
  P_ = mtimes(f, mtimes(P_, f.T())) + dt * Q_;
}

void EkfBase::update(const DM& z, const DM& u, double dt)
{
  // Get observation matrices H
  DM h = jac_g_(DMDict({ { "x", x_ }, { "u", u } })).at("jac");
  // Innovation or measurement residual
  DM y = z - g_(DMDict({ { "x", x_ }, { "u", u } })).at("z");
  // Innovation(or residual) covariance
  DM s = mtimes(h, mtimes(P_, h.T())) + R_ / dt;
  // Near-optimal Kalman gain TODO(qiayuan): pseudo inverse?
  DM k = mtimes(P_, mtimes(transpose(h), inv(s)));
  x_ += mtimes(k, y);
  P_ -= mtimes(mtimes(k, h), P_);
}

void EkfBase::setInitialGuess(const DM& x0, const DM& p0)
{
  x_ = x0;
  P_ = p0;
}

void EkfBase::setNoise(const DM& Q, const DM& R)
{
  Q_ = Q;
  R_ = R;
}

}  // namespace rm_track
