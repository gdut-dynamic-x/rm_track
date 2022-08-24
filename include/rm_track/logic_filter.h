//
// Created by qiayuan on 2021/9/25.
//

#pragma once

#include "buffer.h"
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace rm_track
{
class LogicFilterBase
{
public:
  explicit LogicFilterBase(XmlRpc::XmlRpcValue rpc_value);
  virtual void input(std::shared_ptr<Buffer> buffer){};

protected:
  double basic_range_[2];
  double double_check_range_[2];
};

class HeightFilter : public LogicFilterBase
{
public:
  explicit HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer);
  void input(std::shared_ptr<Buffer> buffer) override;

private:
  tf2_ros::Buffer* tf_buffer_;
};

class DistanceFilter : public LogicFilterBase
{
public:
  explicit DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer);
  void input(std::shared_ptr<Buffer> buffer) override;

private:
  tf2_ros::Buffer* tf_buffer_;
};

class ConfidenceFilter : public LogicFilterBase
{
public:
  explicit ConfidenceFilter(const XmlRpc::XmlRpcValue& rpc_value);
  void input(std::shared_ptr<Buffer> buffer) override;
};

}  // namespace rm_track
