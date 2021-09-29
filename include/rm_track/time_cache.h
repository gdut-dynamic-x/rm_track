//
// Created by qiayuan on 2021/9/22.
//

#pragma once

#include "detection_storage.h"
#include <ctime>
#include <deque>

namespace rm_track
{
class TimeCache
{
public:
  explicit TimeCache(ros::Duration max_storage_time = ros::Duration(5.)) : max_storage_time_(max_storage_time){};

  bool getData(ros::Time time, DetectionStorage& data);
  bool insertData(const DetectionStorage& data);
  double findClosestInPast(const ros::Time& time_in, ros::Time& time_out, const Target& in, Target* out);

  ros::Time getLatestTimestamp();
  ros::Time getOldestTimestamp();

private:
  // TODO: Interpolate implementation
  //  inline void interpolate(const DetectionStorage& one, const DetectionStorage& two, ros::Time time,
  //                          DetectionStorage& output){}

  void pruneList();

  ros::Duration max_storage_time_;
  std::deque<DetectionStorage> storage_;
};
}  // namespace rm_track