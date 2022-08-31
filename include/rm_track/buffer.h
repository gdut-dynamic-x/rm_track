//
// Created by qiayuan on 2021/9/26.
//
#pragma once

#include "tracker.h"
#include <unordered_map>
#include <ros/ros.h>

namespace rm_track
{
class Buffer
{
public:
  Buffer(ros::NodeHandle& nh)
  {
    nh.param("max_match_distance", max_match_distance_, 0.2);
    nh.param("max_storage_time", max_storage_time_, 5.0);
    nh.param("max_lost_time", max_lost_time_, 0.1);
    LinearKf predictor;
    ros::NodeHandle linear_kf_nh = ros::NodeHandle(nh, "linear_kf");
    predictor.initStaticConfig(linear_kf_nh);
  }
  void addTracker(ros::Time stamp, Target& target)
  {
    (id2trackers_.find(target.id) == id2trackers_.end() ? allocateTrackers(target.id) : id2trackers_[target.id])
        ->addTracker(stamp, target);
  }
  void updateBuffer(TargetsStamp targets_stamp)
  {
    for (auto it = id2trackers_.begin(); it != id2trackers_.end();)
    {
      if (it->second->trackers_.empty())
        it = id2trackers_.erase(it);
      else
      {
        it->second->updateTracker(targets_stamp);
        it++;
      }
    }
    if (!targets_stamp.targets.empty())
      for (auto& target : targets_stamp.targets)
        addTracker(targets_stamp.stamp, target);
  }
  void updateState()
  {
    for (auto& trackers : id2trackers_)
    {
      for (auto it = trackers.second->trackers_.begin(); it != trackers.second->trackers_.end();)
      {
        it->updateTrackerState();
        if (it->target_cache_.empty())
          it = trackers.second->trackers_.erase(it);
        else
          it++;
      }
    }
  }

  std::unordered_map<int, std::shared_ptr<Trackers>> id2trackers_;

private:
  std::shared_ptr<Trackers>& allocateTrackers(int id)
  {
    id2trackers_.insert(
        std::make_pair(id, std::make_shared<Trackers>(id, max_match_distance_, max_lost_time_, max_storage_time_)));
    return id2trackers_[id];
  }
  double max_match_distance_;
  double max_storage_time_, max_lost_time_;
};
}  // namespace rm_track
