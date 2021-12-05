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
  ros::Time stamp;
  int id;
  tf2::Transform transform;
};
class LogicSelectorBase
{
public:
  LogicSelectorBase() = default;
  virtual bool input(const Buffer& buffer)
  {
    return true;
  }
  Armor output() const
  {
    return target_armor_;
  }

protected:
  Armor last_armor_, target_armor_;
};

class SameIDArmorSelector : public LogicSelectorBase
{
public:
  SameIDArmorSelector() = default;
  bool input(const Buffer& buffer) override;
};

class StaticArmorSelector : public LogicSelectorBase
{
public:
  StaticArmorSelector() = default;
  bool input(const Buffer& buffer) override;
};

class ClosestArmorSelector : public LogicSelectorBase
{
public:
  ClosestArmorSelector() = default;
  bool input(const Buffer& buffer) override;
};

}  // namespace rm_track
