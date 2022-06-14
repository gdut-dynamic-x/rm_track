//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/logic_filter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_track
{
LogicFilterBase::LogicFilterBase(XmlRpc::XmlRpcValue rpc_value)
{
  if (rpc_value.hasMember("basic_range") && rpc_value.hasMember("double_check_range"))
  {
    basic_range_[0] = (double)rpc_value["basic_range"][0];
    basic_range_[1] = (double)rpc_value["basic_range"][1];
    double_check_range_[0] = (double)rpc_value["double_check_range"][0];
    double_check_range_[1] = (double)rpc_value["double_check_range"][1];
  }
  else
    ROS_ERROR("Some filter params doesn't given");
}

HeightFilter::HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
  ROS_INFO("Height filter add.");
}
void HeightFilter::input(Buffer& buffer)
{
  for (auto& cache : buffer.id2caches_)
  {
    for (auto& storage : cache.second.storage_que_)
    {
      auto& targets = storage.targets_;
      auto target_it = targets.begin();
      while (target_it != targets.end())
      {
        tf2::Transform odom2target = target_it->transform;
        auto odom2base = tf_buffer_->lookupTransform("base_link", "odom", storage.stamp_);
        if (std::abs(odom2target.getOrigin().z() - odom2base.transform.translation.z) < basic_range_[0] ||
            std::abs(odom2target.getOrigin().z() - odom2base.transform.translation.z) > basic_range_[1])
          target_it = targets.erase(target_it);
        else
          target_it++;
      }
    }
  }
}

DistanceFilter::DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
  ROS_INFO("Distance filter add.");
}
void DistanceFilter::input(Buffer& buffer)
{
  for (auto& cache : buffer.id2caches_)
  {
    for (auto& storage : cache.second.storage_que_)
    {
      auto& targets = storage.targets_;
      auto target_it = targets.begin();
      while (target_it != targets.end())
      {
        tf2::Transform transform = target_it->transform;
        geometry_msgs::Pose pose;
        tf2::toMsg(transform, pose);
        tf2::doTransform(pose, pose, tf_buffer_->lookupTransform("base_link", "odom", storage.stamp_));
        tf2::fromMsg(pose, transform);
        if (transform.getOrigin().length() < basic_range_[0] || transform.getOrigin().length() > basic_range_[1])
          target_it = targets.erase(target_it);
        else
          target_it++;
      }
    }
  }
}

ConfidenceFilter::ConfidenceFilter(const XmlRpc::XmlRpcValue& rpc_value) : LogicFilterBase(rpc_value)
{
}
void ConfidenceFilter::input(Buffer& buffer)
{
}

}  // namespace rm_track
