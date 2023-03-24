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
void HeightFilter::input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  for (auto& trackers : id2trackers)
  {
    trackers.second->imprecise_exist_trackers_ = trackers.second->getExistTracker();
    for (auto& tracker : trackers.second->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR)
      {
        tf2::Transform odom2target = tracker.target_cache_.back().target.transform;
        geometry_msgs::TransformStamped odom2base;
        try
        {
          odom2base = tf_buffer_->lookupTransform("base_link", "odom", tracker.target_cache_.back().stamp);
        }
        catch (tf2::TransformException& ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }
        if (std::abs(odom2target.getOrigin().z() - odom2base.transform.translation.z) < basic_range_[0] ||
            std::abs(odom2target.getOrigin().z() - odom2base.transform.translation.z) > basic_range_[1])
        {
          ROS_ERROR("Height filter ERROR");
          tracker.state_ = Tracker::NOT_SELECTABLE;
        }
      }
    }
  }
}

DistanceFilter::DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, tf2_ros::Buffer* tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
  ROS_INFO("Distance filter add.");
}
void DistanceFilter::input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  for (auto& trackers : id2trackers)
  {
    trackers.second->imprecise_exist_trackers_ = trackers.second->getExistTracker();
    for (auto& tracker : trackers.second->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR)
      {
        tf2::Transform transform = tracker.target_cache_.back().target.transform;
        geometry_msgs::Pose pose;
        tf2::toMsg(transform, pose);
        geometry_msgs::TransformStamped odom2base;
        try
        {
          odom2base = tf_buffer_->lookupTransform("base_link", "odom", tracker.target_cache_.back().stamp);
        }
        catch (tf2::TransformException& ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }
        tf2::doTransform(pose, pose, odom2base);
        tf2::fromMsg(pose, transform);
        if (transform.getOrigin().length() < basic_range_[0] || transform.getOrigin().length() > basic_range_[1])
        {
          ROS_ERROR("Distance filter ERROR");
          tracker.state_ = Tracker::NOT_SELECTABLE;
        }
      }
    }
  }
}

ConfidenceFilter::ConfidenceFilter(const XmlRpc::XmlRpcValue& rpc_value) : LogicFilterBase(rpc_value)
{
}
void ConfidenceFilter::input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
}

PitchFilter::PitchFilter(const XmlRpc::XmlRpcValue& rpc_value) : LogicFilterBase(rpc_value)
{
  ROS_INFO("Pitch filter add.");
}

void PitchFilter::input(std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  for (auto& trackers : id2trackers)
  {
    trackers.second->imprecise_exist_trackers_ = trackers.second->getExistTracker();
    if (trackers.second->state_ == Trackers::IMPRECISE_AUTO_AIM)
    {
      for (auto& tracker : trackers.second->trackers_)
        if (tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR)
        {
          if (abs(tracker.target_cache_.back().target.target2camera_rpy[1]) > basic_range_[1])
            tracker.state_ = Tracker::NOT_SELECTABLE;
        }
    }
  }
}

}  // namespace rm_track
