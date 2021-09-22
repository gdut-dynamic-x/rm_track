//
// Created by qiayuan on 2021/9/22.
//

#pragma once

#include "detection_storage.h"
#include <ctime>
#include <deque>

namespace rm_track
{
class Cache
{
public:
  explicit Cache(ros::Duration max_storage_time = ros::Duration(5.)) : max_storage_time_(max_storage_time){};

  bool getData(ros::Time time, DetectionStorage& data);
  void insertData(const DetectionStorage& data);
  double findClosestInPast(const DetectionStorage& in, DetectionStorage* out);

  ros::Time getLatestTimestamp();
  ros::Time getOldestTimestamp();

private:
  inline void interpolate(const DetectionStorage& one, const DetectionStorage& two, ros::Time time,
                          DetectionStorage& output)
  {
    // TODO: Add interpolate
  }

  void pruneList();

  ros::Duration max_storage_time_;
  std::deque<DetectionStorage> detections_;
};
}  // namespace rm_track