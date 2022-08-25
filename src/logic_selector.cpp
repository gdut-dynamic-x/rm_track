//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool RandomArmorSelector::input(const std::shared_ptr<Buffer> buffer)
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
          return true;
        }
    }
    return false;
  }
  else
    return false;
}

bool LastArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return true;
}

bool SameIDArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return false;
}

bool StaticArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return true;
}

bool ClosestArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return true;
}

bool HeroArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return false;
}

bool StandardArmorSelector::input(const std::shared_ptr<Buffer> buffer)
{
  return false;
}

Tracker* LogicSelectorBase::selected_tracker_;
Armor LogicSelectorBase::last_armor_;
}  // namespace rm_track
