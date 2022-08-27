//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool RandomArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  if (!buffer->id2trackers_.empty())
  {
    for (auto& trackers : buffer->id2trackers_)
    {
      for (auto& tracker : trackers.second->trackers_)
        if (tracker.state_ == Tracker::EXIST)
        {
          selected_tracker_ = &tracker;
          double x[6];
          tracker.getState(x);
          last_armor_ = Armor{ .id = tracker.target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
          has_last_armor_ = true;
          return true;
        }
    }
    return false;
  }
  else
    return false;
}

bool LastArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  if (!has_last_armor_)
    return false;
  target_matcher_.setTargetPosition(last_armor_.position);
  for (auto& trackers : buffer->id2trackers_)
    for (auto& tracker : trackers.second->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST)
      {
        double x[6];
        tracker.getState(x);
        if (target_matcher_.input(tf2::Vector3{ x[0], x[2], x[4] }))
        {
          selected_tracker_ = &tracker;
          last_armor_ = Armor{ .id = tracker.target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
        }
      }
    }
  has_last_armor_ = target_matcher_.matchSuccessful();
  return target_matcher_.matchSuccessful();
}

bool SameIDArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  if (!has_last_armor_)
    return false;
  if (!buffer->id2trackers_.count(last_armor_.id))
    return false;
  else
  {
    for (auto& tracker : buffer->id2trackers_.at(last_armor_.id)->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST)
      {
        selected_tracker_ = &tracker;
        double x[6];
        selected_tracker_->getState(x);
        last_armor_ = Armor{ .id = selected_tracker_->target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
        has_last_armor_ = true;
        return true;
      }
    }
    return false;
  }
}

bool StaticArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  return true;
}

bool ClosestArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  return true;
}

bool HeroArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  return false;
}

bool StandardArmorSelector::input(const std::shared_ptr<Buffer>& buffer)
{
  return false;
}

Tracker* LogicSelectorBase::selected_tracker_;
Armor LogicSelectorBase::last_armor_;
bool LogicSelectorBase::has_last_armor_ = false;
}  // namespace rm_track
