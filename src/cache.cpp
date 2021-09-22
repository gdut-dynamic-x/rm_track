//
// Created by qiayuan on 2021/9/22.
//

#pragma once

#include "rm_track/cache.h"
namespace rm_track
{
bool Cache::getData(ros::Time time, DetectionStorage& data)
{
  return false;
}

void insertData(const DetectionStorage& data)
{
}

double Cache::findClosestInPast(const DetectionStorage& in, DetectionStorage* out)
{
  return 0;
}
ros::Time Cache::getLatestTimestamp()
{
  return ros::Time();
}
ros::Time Cache::getOldestTimestamp()
{
  return ros::Time();
}
}  // namespace rm_track