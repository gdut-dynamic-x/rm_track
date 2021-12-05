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
  explicit LogicFilterBase(XmlRpc::XmlRpcValue rpc_value);
  virtual void input(Buffer& buffer){};

protected:
  double basic_range_[2];
  double double_check_range_[2];
};

class HeightFilter : public LogicFilterBase
{
public:
  explicit HeightFilter(XmlRpc::XmlRpcValue rpc_value);
  void input(Buffer& buffer) override;
};

class DistanceFilter : public LogicFilterBase
{
public:
  explicit DistanceFilter(XmlRpc::XmlRpcValue rpc_value);
  void input(Buffer& buffer) override;
};

class ConfidenceFilter : public LogicFilterBase
{
public:
  explicit ConfidenceFilter(XmlRpc::XmlRpcValue rpc_value);
  void input(Buffer& buffer) override;
};

}  // namespace rm_track
