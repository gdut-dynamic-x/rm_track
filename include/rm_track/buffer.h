//
// Created by qiayuan on 2021/9/26.
//
#pragma once

#include "time_cache.h"
#include <unordered_map>
#include <ros/ros.h>

namespace rm_track
{
class Buffer
{
public:
  Buffer(ros::NodeHandle& nh)
  {
    nh.param("max_storage_time", max_storage_time_, 5.0);
    nh.param("max_lost_time", max_lost_time_, 0.1);
  }
  void insertData(int id, DetectionStorage data)
  {
    (id2caches_.find(id) == id2caches_.end() ? allocateCache(id) : id2caches_.at(id)).insertData(data);
  }
  TimeCache& getTimeCache(int id)
  {
    return id2caches_.at(id);
  }
  void updateState(ros::Time latest_time)
  {
    for (auto& cache : id2caches_)
      cache.second.updateState(latest_time);
  }
  void eraseUselessData()
  {
    auto cache_it = id2caches_.begin();
    while (cache_it != id2caches_.end())
    {
      cache_it->second.eraseUselessData();
      if (cache_it->second.storage_que_.empty())
        id2caches_.erase(cache_it++);
      else
        cache_it++;
    }
  }

  std::unordered_map<int, TimeCache> id2caches_;

private:
  double max_storage_time_, max_lost_time_;
  TimeCache& allocateCache(int id)
  {
    id2caches_.insert(std::make_pair(id, TimeCache(ros::Duration(max_storage_time_), ros::Duration(max_lost_time_))));
    return id2caches_.at(id);
  }
};
}  // namespace rm_track
