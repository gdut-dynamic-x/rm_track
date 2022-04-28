//
// Created by qiayuan on 2022/4/17.
//

#pragma once

#include "ekf_base.h"

namespace rm_track
{
class LinearKf : public EkfBase
{
public:
  LinearKf()
  {
    SX x = SX::sym("x", 6);
    SX u = SX::sym("u", 0);  // Error if equal to zero?
    SX dt = SX::sym("dt", 1);

    SX x_next = SX::vertcat({ x(0) + x(1) * dt, x(1), x(2) + x(3) * dt, x(3), x(4) + x(5) * dt, x(5) });
    SX z = SX::vertcat({ x(0), x(2), x(4) });
    Function f =
        Function("f", { { "x", x }, { "u", u }, { "dt", dt }, { "x_next", x_next } }, { "x", "u", "dt" }, { "x_next" });
    Function g = Function("g", { { "x", x }, { "u", u }, { "z", z } }, { "x", "u" }, { "z" });
    setup(f, g);
  }
  void reset(double x0[6])
  {
    DM x = DM::vertcat({ x0[0], x0[1], x0[2], x0[3], x0[4], x0[5] });
    DM p = DM::zeros(6, 6);
    setInitialGuess(x, p);
  }
  void predict(double dt)
  {
    DM u(0);
    EkfBase::predict(u, dt);
  }
  void update(double z[3], double dt)
  {
    DM z_m = DM::vertcat({ z[0], z[1], z[2] });
    DM u(0);
    EkfBase::update(z_m, u, dt);
  }
  void getState(double x[6]) const
  {
    DM x_m = EkfBase::getState();
    x[0] = static_cast<double>(x_m(0, 0));
    x[1] = static_cast<double>(x_m(1, 0));
    x[2] = static_cast<double>(x_m(2, 0));
    x[3] = static_cast<double>(x_m(3, 0));
    x[4] = static_cast<double>(x_m(4, 0));
    x[5] = static_cast<double>(x_m(5, 0));
  }
};

}  // namespace rm_track
