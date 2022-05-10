//
// Created by qiayuan on 2022/4/17.
//
#include "rm_track/ekf/ekf_base.h"

namespace rm_track
{
void EkfBase::setup(const Function& f, const Function& g)
{
  f_ = f;
  g_ = g;
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
  p_ = mtimes(f, mtimes(p_, f.T())) + dt * q_;
}

void EkfBase::update(const DM& z, const DM& u, double dt)
{
  // Get observation matrices H
  DM h = jac_g_(DMDict({ { "x", x_ }, { "u", u } })).at("jac");
  // Innovation or measurement residual
  DM y = z - g_(DMDict({ { "x", x_ }, { "u", u } })).at("z");
  // Innovation(or residual) covariance
  DM s = mtimes(h, mtimes(p_, h.T())) + r_ / dt;
  // Near-optimal Kalman gain TODO(qiayuan): pseudo inverse?
  DM k = mtimes(p_, mtimes(transpose(h), inv(s)));
  // Updated state estimate
  x_ += mtimes(k, y);
  // Updated covariance estimate
  p_ -= mtimes(mtimes(k, h), p_);
}

void EkfBase::setInitialGuess(const DM& x0, const DM& p0)
{
  x_ = x0;
  p_ = p0;
  inited_ = true;
}

void EkfBase::setNoise(const DM& Q, const DM& R)
{
  q_ = Q;
  r_ = R;
}
DM EkfBase::getState() const
{
  return x_;
}
DM EkfBase::getCovariance() const
{
  return q_;
}
void EkfBase::getQR(const ros::NodeHandle& nh)
{
  int state_dimension = f_.size1_in(0);
  int measure_dimension = g_.size1_out(0);
  setNoise(getMatrix(nh, "Q", state_dimension), getMatrix(nh, "R", measure_dimension));
}
DM EkfBase::getMatrix(const ros::NodeHandle& nh, const std::string& name, int dimension)
{
  XmlRpc::XmlRpcValue matrix;
  DM m = DM::zeros(dimension, dimension);
  nh.getParam(name, matrix);
  ROS_ASSERT(matrix.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(matrix.size() == dimension);
  for (int i = 0; i < dimension; ++i)
  {
    ROS_ASSERT(matrix[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
               matrix[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (matrix[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      m(i, i) = static_cast<double>(matrix[i]);
    if (matrix[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      m(i, i) = static_cast<int>(matrix[i]);
  }
  return m;
}

}  // namespace rm_track
