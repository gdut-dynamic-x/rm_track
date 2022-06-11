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
  explicit TimeCache(ros::Duration max_storage_time = ros::Duration(5.),
                     ros::Duration max_lost_time = ros::Duration(0.5))
    : max_storage_time_(max_storage_time), max_lost_time_(max_lost_time){};

  bool getData(ros::Time time, DetectionStorage& data);
  bool insertData(DetectionStorage& data);
  double findClosestInPast(const ros::Time& time_in, ros::Time& time_out, const Target& in, Target* out);
  void updateState(ros::Time latest_time);
  void eraseUselessData();

  ros::Time getLatestTimestamp();
  ros::Time getOldestTimestamp();

  std::deque<DetectionStorage> storage_que_;

private:
  // TODO: Interpolate implementation
  //  inline void interpolate(const DetectionStorage& one, const DetectionStorage& two, ros::Time time,
  //                          DetectionStorage& output){}

  void pruneList();

  ros::Duration max_storage_time_;
  ros::Duration max_lost_time_;
};
}  // namespace rm_track
