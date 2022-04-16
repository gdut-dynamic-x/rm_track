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

    SX x_next = SX::vertcat({ x(0) + x(1) * dt, x(1), x(2) + x(3) * dt, x(3), x(4) + x(5) * dt });
    SX z = SX::vertcat({ x(0), x(2), x(4) });
    Function f =
        Function("f", { { "x", x }, { "u", u }, { "dt", dt }, { "x_next", x_next } }, { "x", "u", "dt" }, { "x_next" });
    Function g = Function("g", { { "x", x }, { "u", u }, { "z", z } }, { "x", "u" }, { "z" });
    setup(f, g);
  }
};

}  // namespace rm_track