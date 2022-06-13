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
}
void HeightFilter::input(Buffer& buffer)
{
  for (auto& cache : buffer.id2caches_)
  {
    auto targets = cache.second.storage_que_.front().targets_;
    for (auto it = targets.begin(); it != targets.end(); it++)
    {
      tf2::Transform transform = it->transform;
      geometry_msgs::Pose pose;
      tf2::toMsg(transform, pose);
      tf2::doTransform(pose, pose, tf_buffer_->lookupTransform("base_link", "odom", ros::Time(0)));
      if (pose.position.z < basic_range_[0] || pose.position.z > basic_range_[1])
        targets.erase(it);
    }
  }
}

DistanceFilter::DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
}
void DistanceFilter::input(Buffer& buffer)
{
  for (auto& cache : buffer.id2caches_)
  {
    auto targets = cache.second.storage_que_.front().targets_;
    for (auto it = targets.begin(); it != targets.end(); it++)
    {
      tf2::Transform transform = it->transform;
      geometry_msgs::Pose pose;
      tf2::toMsg(transform, pose);
      tf2::doTransform(pose, pose, tf_buffer_->lookupTransform("base_link", "odom", ros::Time(0)));
      tf2::fromMsg(pose, transform);
      ROS_INFO_THROTTLE(3, "Distance: %f,lower_range: %f, upper_range: %f", transform.getOrigin().length(),
                        basic_range_[0], basic_range_[1]);
      if (transform.getOrigin().length() < basic_range_[0] || transform.getOrigin().length() > basic_range_[1])
        targets.erase(it);
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
