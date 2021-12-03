//
// Created by qiayuan on 2021/9/26.
//
#pragma once

#include "time_cache.h"
#include <unordered_map>

namespace rm_track
{
class Buffer
{
public:
  Buffer() = default;
  void insertData(int id, DetectionStorage data)
  {
    (id2caches_.find(id) == id2caches_.end() ? allocateCache(id) : id2caches_[id]).insertData(data);
  }
  TimeCache& getTimeCache(int id)
  {
    return id2caches_[id];
  }
  void updateState(ros::Time latest_time)
  {
    for (auto cache : id2caches_)
      cache.second.updateState(latest_time);
  }

private:
  TimeCache& allocateCache(int id)
  {
    id2caches_.insert(std::make_pair(id, TimeCache()));
    return id2caches_[id];
  }
  std::unordered_map<int, TimeCache> id2caches_;
};
}  // namespace rm_track