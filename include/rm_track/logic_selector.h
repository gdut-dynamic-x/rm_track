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
  virtual bool input(const std::shared_ptr<Buffer> buffer)
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
};

class RandomArmorSelector : public LogicSelectorBase
{
public:
  RandomArmorSelector()
  {
    ROS_INFO("random_armor_selector add");
  }
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class LastArmorSelector : public LogicSelectorBase
{
public:
  LastArmorSelector() = default;
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class SameIDArmorSelector : public LogicSelectorBase
{
public:
  SameIDArmorSelector() = default;
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class StaticArmorSelector : public LogicSelectorBase
{
public:
  StaticArmorSelector() = default;
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class ClosestArmorSelector : public LogicSelectorBase
{
public:
  ClosestArmorSelector() = default;
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class HeroArmorSelector : public LogicSelectorBase
{
public:
  HeroArmorSelector() = default;
  bool input(const std::shared_ptr<Buffer> buffer) override;
};

class StandardArmorSelector : public LogicSelectorBase
{
public:
  StandardArmorSelector()
  {
    ROS_INFO("standard_armor_selector add");
  }
  bool input(const std::shared_ptr<Buffer> buffer) override;
};
}  // namespace rm_track
