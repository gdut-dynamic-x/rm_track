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
  void predict(double dt);
  void update(double z[3]);
  void getState(double x[6]) const;

private:
  void reconfigCB(rm_track::EKfConfig& config, uint32_t level)
  {
    if (!dynamic_reconfig_initialized_)
    {
      config.q_element = 0;
      config.q_value = q_dynamic_[0];
      config.r_element = 0;
      config.r_value = r_dynamic_[0];
      dynamic_reconfig_initialized_ = true;
    }
    q_dynamic_[config.q_element] = config.q_value;
    r_dynamic_[config.r_element] = config.r_value;
  }
  bool dynamic_reconfig_initialized_ = false;
  static double q_dynamic_[], r_dynamic_[];
  static dynamic_reconfigure::Server<rm_track::EKfConfig>* reconf_server_;
};

}  // namespace rm_track
