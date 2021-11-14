//
// Created by qiayuan on 2021/9/25.
//

#pragma once

#include "buffer.h"
#include <ros/ros.h>

namespace rm_track
{
class LogicFilterBase
{
public:
  LogicFilterBase() = default;

  virtual void input(Buffer& buffer) = 0;
};

class HeightFilter : public LogicFilterBase
{
public:
  HeightFilter(ros::NodeHandle& nh, XmlRpc::XmlRpcValue rpc_value);
  void input(Buffer& buffer) override;

private:
  double basic_range_[2];
  double double_check_range_[2];
};

}  // namespace rm_track
