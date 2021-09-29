//
// Created by qiayuan on 2021/9/26.
//
#include "time_cache.h"

namespace rm_track
{
typedef boost::shared_ptr<TimeCache> TimeCachePtr;

class Buffer
{
public:
  Buffer() = default;
  void insertData(int id, const DetectionStorage& data);
  TimeCachePtr getTimeCache(int id);

private:
  TimeCachePtr allocateFrame(int id);
  std::vector<TimeCachePtr> caches_;
};
}  // namespace rm_track