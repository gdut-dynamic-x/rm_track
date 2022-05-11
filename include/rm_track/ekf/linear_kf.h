//
// Created by qiayuan on 2022/4/17.
//

#pragma once

#include "ekf_base.h"
#include "rm_track/EKfConfig.h"
#include <dynamic_reconfigure/server.h>

namespace rm_track
{
class LinearKf : public EkfBase
{
public:
  LinearKf();
  void init(ros::NodeHandle& nh) override;
  void reset(double x0[6]);
  void updateQR();
  void predict(double dt);
  void update(double z[3], double dt);
  void getState(double x[6]) const;

private:
  void reconfigCB(rm_track::EKfConfig& config, uint32_t level)
  {
    q_dynamic_[config.q_element] = config.q_value;
    r_dynamic_[config.q_element] = config.q_value;
  }
  static double q_dynamic_[], r_dynamic_[];
  static dynamic_reconfigure::Server<rm_track::EKfConfig>* reconf_server_;
};

}  // namespace rm_track
