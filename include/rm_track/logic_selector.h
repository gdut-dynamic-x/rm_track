//
// Created by chenzheng on 2021/11/16.
//

#pragma once

#include "buffer.h"
#include <ros/ros.h>

namespace rm_track
{
struct Armor
{
  int id;
  tf2::Vector3 position;
};
class LogicSelectorBase
{
public:
  LogicSelectorBase() = default;
  virtual bool input(const std::shared_ptr<Buffer>& buffer)
  {
    return true;
  }
  Tracker* output() const
  {
    return selected_tracker_;
  }

protected:
  static Tracker* selected_tracker_;
  static Armor last_armor_;
  static bool has_last_armor_;
};

class RandomArmorSelector : public LogicSelectorBase
{
public:
  RandomArmorSelector()
  {
    ROS_INFO("Add random_armor_selector.");
  }
  bool input(const std::shared_ptr<Buffer>& buffer) override;
};

class LastArmorSelector : public LogicSelectorBase
{
public:
  LastArmorSelector()
  {
    ROS_INFO("Add last_armor_selector.");
    target_matcher_.setMaxMatchDistance(0.1);
  }
  bool input(const std::shared_ptr<Buffer>& buffer) override;

private:
  TargetMatcher target_matcher_;
};

class SameIDArmorSelector : public LogicSelectorBase
{
public:
  SameIDArmorSelector()
  {
    ROS_INFO("Add same_id_armor_selector");
  }
  bool input(const std::shared_ptr<Buffer>& buffer) override;
};
}  // namespace rm_track
