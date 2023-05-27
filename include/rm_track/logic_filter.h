//
// Created by qiayuan on 2021/9/25.
//

#pragma once

#include "tracker.h"
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace rm_track
{
class LogicFilterBase
{
public:
  explicit LogicFilterBase(XmlRpc::XmlRpcValue rpc_value);
  LogicFilterBase(){};
  virtual void input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers_){};

protected:
  double basic_range_[2];
  double double_check_range_[2];
};

class HeightFilter : public LogicFilterBase
{
public:
  explicit HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer);
  void input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;

private:
  tf2_ros::Buffer* tf_buffer_;
};

class DistanceFilter : public LogicFilterBase
{
public:
  explicit DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer);
  void input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;

private:
  tf2_ros::Buffer* tf_buffer_;
};

class ConfidenceFilter : public LogicFilterBase
{
public:
  explicit ConfidenceFilter(const XmlRpc::XmlRpcValue& rpc_value);
  void input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;
};

class IdFilter : public LogicFilterBase
{
public:
  explicit IdFilter(const XmlRpc::XmlRpcValue& rpc_value);
  void input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;

private:
  int id_;
};

}  // namespace rm_track
