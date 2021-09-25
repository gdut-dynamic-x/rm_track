//
// Created by qiayuan on 2021/9/22.
//

#pragma once

#include "rm_track/cache.h"
#include <ros/ros.h>

namespace rm_track
{
bool Cache::getData(ros::Time time, DetectionStorage& data)
{
  if (time == ros::Time() && storage_.empty())
  {
    data = storage_.front();
    return true;
  }
  else  // TODO other time with Interpolate;
    return false;
}

bool Cache::insertData(const DetectionStorage& data)
{
  auto storage_it = storage_.begin();
  if (storage_it != storage_.end())
  {
    if (storage_it->stamp_ > data.stamp_ + max_storage_time_)
    {
      ROS_WARN("Ignoring data from the past");
      return false;
    }
  }
  while (storage_it != storage_.end())
  {
    if (storage_it->stamp_ <= data.stamp_)
      break;
    storage_it++;
  }
  storage_.insert(storage_it, data);
  pruneList();
  return true;
}

double Cache::findClosestInPast(const ros::Time& time_in, ros::Time& time_out, const Target& in, Target* out)
{
  auto storage_it = storage_.begin();
  while (storage_it != storage_.end())
  {
    if (storage_it->stamp_ >= time_in || ros::Time(0) == time_in)
      break;
    storage_it++;
  }
  time_out = storage_it->stamp_;
  double distance = 1e10;
  for (auto storage : storage_it->targets_)
  {
    double dis = (storage.transform.getOrigin() - in.transform.getOrigin()).length();
    if (distance > dis)
    {
      out = &storage;
      distance = dis;
    }
  }
  return distance;
}

ros::Time Cache::getLatestTimestamp()
{
  if (storage_.empty())  // empty list case
    return ros::Time();
  return storage_.front().stamp_;
}

ros::Time Cache::getOldestTimestamp()
{
  if (storage_.empty())  // empty list case
    return ros::Time();
  return storage_.back().stamp_;
}

void Cache::pruneList()
{
  while (!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < storage_.begin()->stamp_)
    storage_.pop_back();
}

}  // namespace rm_track