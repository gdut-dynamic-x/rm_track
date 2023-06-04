//
// Created by chenzheng on 2021/11/16.
//

#pragma once

#include "tracker.h"
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
  virtual bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
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

class ClosestToImageCenterSelector : public LogicSelectorBase
{
public:
  ClosestToImageCenterSelector()
  {
    ROS_INFO("Add closest_to_image_center_selector.");
  }
  bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;
};

class RandomArmorSelector : public LogicSelectorBase
{
public:
  RandomArmorSelector()
  {
    ROS_INFO("Add random_armor_selector.");
  }
  bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;
};

class LastArmorSelector : public LogicSelectorBase
{
public:
  LastArmorSelector(double max_match_distance)
  {
    ROS_INFO("Add last_armor_selector.");
    target_matcher_.setMaxMatchDistance(max_match_distance);
  }
  bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;

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
  bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;
};

class IdSelector : public LogicSelectorBase
{
public:
  IdSelector(const XmlRpc::XmlRpcValue& rpc_value)
  {
    if (rpc_value.hasMember("id"))
    {
      id_ = (int)rpc_value["id"];
      ROS_INFO("Add id_selector");
    }
    else
      ROS_ERROR("id_selector: id is not set.");
  }
  bool input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers) override;

private:
  int id_;
};
}  // namespace rm_track
