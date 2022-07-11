//
// Created by yezi on 2022/5/10.
//

#include <geometry_msgs/Point.h>
#include "rm_track/ekf/linear_kf.h"

namespace rm_track
{
LinearKf::LinearKf()
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

void LinearKf::init(ros::NodeHandle& nh)
{
  getQR(nh);
  nh.param("debug", is_debug_, false);

  if (is_debug_)
    measure_pub_ = nh.advertise<geometry_msgs::Point>("measure", 1);
  for (int i = 0; i < 3; i++)
  {
    q_dynamic_[i] = double(q_(2 * i, 2 * i));
  }
  for (int i = 0; i < 3; i++)
  {
    r_dynamic_[i] = double(r_(i, i));
  }
  // Init dynamic reconfigure
  reconf_server_ = new dynamic_reconfigure::Server<rm_track::EKfConfig>(ros::NodeHandle("~/linear_kf"));
  dynamic_reconfigure::Server<rm_track::EKfConfig>::CallbackType cb = boost::bind(&LinearKf::reconfigCB, this, _1, _2);
  reconf_server_->setCallback(cb);
}

void LinearKf::reset(double* x0)
{
  DM x = DM::vertcat({ x0[0], x0[1], x0[2], x0[3], x0[4], x0[5] });
  DM p = DM::zeros(6, 6);
  setInitialGuess(x, p);
}

void LinearKf::predict(double dt)
{
  DM u(0);
  DM gamma_k = DM::horzcat({ DM::vertcat({ 0.5 * dt * dt, 0, 0, 0, 0, 0 }), DM::vertcat({ 0, dt, 0, 0, 0, 0 }),
                             DM::vertcat({ 0, 0, 0.5 * dt * dt, 0, 0, 0 }), DM::vertcat({ 0, 0, 0, dt, 0, 0 }),
                             DM::vertcat({ 0, 0, 0, 0, 0.5 * dt * dt, 0 }), DM::vertcat({ 0, 0, 0, 0, 0, dt }) });
  DM w = DM::horzcat({ DM::vertcat({ q_dynamic_[0], 0, 0, 0, 0, 0 }), DM::vertcat({ 0, q_dynamic_[0], 0, 0, 0, 0 }),
                       DM::vertcat({ 0, 0, q_dynamic_[1], 0, 0, 0 }), DM::vertcat({ 0, 0, 0, q_dynamic_[1], 0, 0 }),
                       DM::vertcat({ 0, 0, 0, 0, q_dynamic_[2], 0 }), DM::vertcat({ 0, 0, 0, 0, 0, q_dynamic_[2] }) });
  q_ = mtimes(gamma_k, w);
  EkfBase::predict(u, dt);
}

void LinearKf::predict(double* x, double dt)
{
  DM u(0);
  DM x_predict = f_(DMDict({ { "x", x_ }, { "u", u }, { "dt", dt } })).at("x_next");
  x[0] = static_cast<double>(x_predict(0, 0));
  x[1] = static_cast<double>(x_predict(1, 0));
  x[2] = static_cast<double>(x_predict(2, 0));
  x[3] = static_cast<double>(x_predict(3, 0));
  x[4] = static_cast<double>(x_predict(4, 0));
  x[5] = static_cast<double>(x_predict(5, 0));
}

void LinearKf::update(double* z)
{
  DM z_m = DM::vertcat({ z[0], z[1], z[2] });
  DM u(0);
  EkfBase::update(z_m, u);
  if (is_debug_)
  {
    geometry_msgs::Point measure_data;
    measure_data.x = z[0];
    measure_data.y = z[1];
    measure_data.z = z[2];
    measure_pub_.publish(measure_data);
  }
}

void LinearKf::getState(double* x) const
{
  DM x_m = EkfBase::getState();
  x[0] = static_cast<double>(x_m(0, 0));
  x[1] = static_cast<double>(x_m(1, 0));
  x[2] = static_cast<double>(x_m(2, 0));
  x[3] = static_cast<double>(x_m(3, 0));
  x[4] = static_cast<double>(x_m(4, 0));
  x[5] = static_cast<double>(x_m(5, 0));
}

double LinearKf::q_dynamic_[3];
double LinearKf::r_dynamic_[3];
dynamic_reconfigure::Server<rm_track::EKfConfig>* LinearKf::reconf_server_;
}  // namespace rm_track
