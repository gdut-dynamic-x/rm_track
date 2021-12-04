//
// Created by qiayuan on 2021/9/26.
//
#pragma once

#include "time_cache.h"
#include <unordered_map>

namespace rm_track
{
struct Armor
{
  int id;
  tf2::Transform transform;
};
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
  std::vector<Armor> eraseUselessData()
  {
    std::vector<Armor> output_armors;
    auto cache = id2caches_.begin();
    while (cache != id2caches_.end())
    {
      std::vector<Target> input_targets = cache->second.eraseUselessData();
      if (input_targets.empty())
        id2caches_.erase(cache);
      else
        for (auto target : input_targets)
          output_armors.push_back(Armor{ .id = cache->first, .transform = target.transform });
      cache++;
    }
    return output_armors;
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