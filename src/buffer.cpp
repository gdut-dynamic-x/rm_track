//
// Created by qiayuan on 2021/9/26.
//

#pragma once

#include "rm_track/buffer.h"

namespace rm_track
{
void Buffer::insertData(int id, const DetectionStorage& data)
{
  TimeCachePtr time_cache = getTimeCache(id);
  if (time_cache == nullptr)
    time_cache = allocateFrame(id);
  time_cache->insertData(data);
}

TimeCachePtr Buffer::getTimeCache(int id)
{
  if (id >= caches_.size())
    return TimeCachePtr();
  else
    return caches_[id];
}

TimeCachePtr Buffer::allocateFrame(int id)
{
  caches_.push_back(TimeCachePtr());
  TimeCachePtr frame_ptr = caches_[id];
  caches_[id] = TimeCachePtr(new TimeCache());
  return caches_[id];
}

}  // namespace rm_track
