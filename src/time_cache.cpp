//
// Created by qiayuan on 2021/9/22.
//

#include "rm_track/time_cache.h"
#include <ros/ros.h>

namespace rm_track
{
bool TimeCache::getData(ros::Time time, DetectionStorage& data)
{
  if (time == ros::Time() && storage_.empty())
  {
    data = storage_.front();
    return true;
  }
  else  // TODO other time with Interpolate;
    return false;
}

bool TimeCache::insertData(DetectionStorage& data)
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

void TimeCache::updateState(ros::Time latest_time)
{
  auto storage_it = storage_.begin();

  if (storage_it == storage_.end())
    return;

  if (storage_it->stamp_ == latest_time)
  {
    if ((storage_it + 1) == storage_.end() || storage_it->stamp_ - (storage_it + 1)->stamp_ > max_lost_time_)
      for (auto target : storage_it->targets_)
        target.state = Target::APPEAR;
    else
    {
      // The number of armor with the same ID not changed
      if ((storage_it + 1)->targets_.size() == storage_it->targets_.size())
      {
        for (int i = 0; i < storage_it->targets_.size(); ++i)
        {
          storage_it->targets_[i].state = Target::EXIST;
          (storage_it + 1)->targets_[i].state = Target::EXPIRED;
        }
      }

      // The number of armor with the same ID changed from 1 to 2
      else if ((storage_it + 1)->targets_.size() == 1 && storage_it->targets_.size() == 2)
      {
        double first_distance = (storage_it->targets_.begin()->transform.getOrigin() -
                                 (storage_it + 1)->targets_.begin()->transform.getOrigin())
                                    .length();
        double second_distance = (storage_it->targets_.end()->transform.getOrigin() -
                                  (storage_it + 1)->targets_.begin()->transform.getOrigin())
                                     .length();
        first_distance < second_distance ? storage_it->targets_.begin()->state :
                                           storage_it->targets_.end()->state = Target::EXIST;
        for (auto storage : storage_)
        {
          if (storage.targets_.size() == 2 && ((storage.targets_.begin()->state == Target::DISAPPEAR &&
                                                storage.targets_.end()->state == Target::EXPIRED) ||
                                               (storage.targets_.begin()->state == Target::EXPIRED &&
                                                storage.targets_.end()->state == Target::DISAPPEAR)))
          {
            first_distance >= second_distance ? storage_it->targets_.begin()->state :
                                                storage_it->targets_.end()->state = Target::EXIST;
            storage.targets_.begin()->state = Target::EXPIRED;
            storage.targets_.end()->state = Target::EXPIRED;
            break;
          }
          if (latest_time - storage.stamp_ > max_lost_time_)
          {
            first_distance >= second_distance ? storage_it->targets_.begin()->state :
                                                storage_it->targets_.end()->state = Target::APPEAR;
            break;
          }
        }
      }

      // The number of armor with the same ID changed from 2 to 1
      else if ((storage_it + 1)->targets_.size() == 2 && storage_it->targets_.size() == 1)
      {
        double first_distance = (storage_it->targets_.begin()->transform.getOrigin() -
                                 (storage_it + 1)->targets_.begin()->transform.getOrigin())
                                    .length();
        double second_distance = (storage_it->targets_.begin()->transform.getOrigin() -
                                  (storage_it + 1)->targets_.end()->transform.getOrigin())
                                     .length();
        if (first_distance >= second_distance)
        {
          (storage_it + 1)->targets_.begin()->state =
              (storage_it + 1)->targets_.begin()->state == Target::APPEAR ? Target::NOT_EXIST : Target::DISAPPEAR;
          (storage_it + 1)->targets_.end()->state = Target::EXPIRED;
        }
        else
        {
          (storage_it + 1)->targets_.begin()->state = Target::EXPIRED;
          (storage_it + 1)->targets_.end()->state =
              (storage_it + 1)->targets_.end()->state == Target::APPEAR ? Target::NOT_EXIST : Target::DISAPPEAR;
        }
        storage_it->targets_.begin()->state = Target::EXIST;
      }
      else
        ROS_ERROR("Multiple (> 2) armor appears");
    }
  }
  else if (storage_it->stamp_ < latest_time)
  {
    if (latest_time - storage_it->stamp_ <= max_lost_time_)
      for (auto target : storage_it->targets_)
        target.state = target.state == Target::APPEAR ? Target::NOT_EXIST : Target::DISAPPEAR;
    else
      for (auto target : storage_it->targets_)
        target.state = Target::NOT_EXIST;
  }
  else
    ROS_ERROR("Future timestamp data appears");
}

double TimeCache::findClosestInPast(const ros::Time& time_in, ros::Time& time_out, const Target& in, Target* out)
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

ros::Time TimeCache::getLatestTimestamp()
{
  if (storage_.empty())  // empty list case
    return ros::Time();
  return storage_.front().stamp_;
}

ros::Time TimeCache::getOldestTimestamp()
{
  if (storage_.empty())  // empty list case
    return ros::Time();
  return storage_.back().stamp_;
}

void TimeCache::pruneList()
{
  while (!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < storage_.begin()->stamp_)
    storage_.pop_back();
}

}  // namespace rm_track