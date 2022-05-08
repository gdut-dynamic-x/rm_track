//
// Created by qiayuan on 2022/4/27.
//

#pragma once

#include "ekf_base.h"

namespace rm_track
{
class ArmorEkf : public EkfBase
{
public:
  ArmorEkf()
  {
    SX u = SX::sym("u", 0);
    SX dt = SX::sym("dt", 1);

    // State X ( 6 + 24 = 30) consistent of two parts shown below and their derivative
    // Chassis (5):
    //    Plane Position and velocity of the center $x_c, y_c, \dot{x}_c, \dot_{y}_c$;
    //    2D Orientation $\theta$ and its velocity $\omega$ ;
    // Armor (4 x 6 = 24):
    //    Plane Position and velocity of center $x_{0 \ldots 3}, y_{0 \ldots 3}, \dot{x}_{0 \ldots 3}, \dot{y}_{0 \ldots
    //    3}$;
    //    The height of armor $z_{0 \ldots 3}$;
    //    Distance between armor and chassis along the horizontal direction with the 0, pi/2, pi, 3pi/2 angle
    //    offset in chassis frame $l_{0 \ldots 3}$;
    SX x = SX::sym("x", 30);

    // Measure Z, (4 x 4 = 16):
    //    Plane Position of armor center $x,y$;
    //    Height of armor $z$;
    //    Orientation of armor (obtain from armor orientation) $\theta$;
    SX z;
    for (int i = 0; i < 4; ++i)
      z = SX::vertcat({ z, x(6 + 4 * i + 0), x(6 + 4 * i + 1), x(6 + 4 * i + 4), x(4) });

    // We assume the center of chassis moving in constance linear velocity and angular velocity
    SX x_next = SX::vertcat({ x(0) + x(2) * dt, x(1) + x(3) * dt, x(2), x(3), x(4) + x(5) * dt, x(5) });
    // The next state of armor is determine by the next state of chassis
    for (int i = 0; i < 4; ++i)
      x_next = SX::vertcat({ x_next, x(6 + 4 * i + 5) * SX::cos(x_next(4) + i * M_PI_2),
                             x(6 + 4 * i + 5) * SX::sin(x_next(4) + i * M_PI_2),
                             x(6 + 4 * i + 5) * x(5) * SX::cos(x_next(4) + (i + 1) * M_PI_2) + x(2),
                             x(6 + 4 * i + 5) * x(5) * SX::sin(x_next(4) + (i + 1) * M_PI_2) + x(3), x(6 + 4 * i + 4),
                             x(6 + 4 * i + 5) });
    Function f =
        Function("f", { { "x", x }, { "u", u }, { "dt", dt }, { "x_next", x_next } }, { "x", "u", "dt" }, { "x_next" });
    Function g = Function("g", { { "x", x }, { "u", u }, { "z", z } }, { "x", "u" }, { "z" });
    setup(f, g);
  }
};
}  // namespace rm_track