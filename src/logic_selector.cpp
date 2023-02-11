//
// Created by chenzheng on 2021/11/16.
//

#include "rm_track/logic_selector.h"

namespace rm_track
{
bool ClosestToImageCenterSelector::input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  double min_distance_from_light_center = DBL_MAX;
  bool has_target = false;
  for (auto& trackers : id2trackers)
  {
    for (auto& tracker : trackers.second->trackers_)
    {
      if ((tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR) &&
          tracker.target_cache_.back().target.distance_to_image_center < min_distance_from_light_center)
      {
        selected_tracker_ = &tracker;
        double x[6];
        selected_tracker_->getTargetState(x);
        last_armor_ = Armor{ .id = tracker.target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
        has_last_armor_ = true;
        min_distance_from_light_center = tracker.target_cache_.back().target.distance_to_image_center;
        has_target = true;
      }
    }
  }
  return has_target;
}

bool RandomArmorSelector::input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  if (!id2trackers.empty())
  {
    for (auto& trackers : id2trackers)
    {
      for (auto& tracker : trackers.second->trackers_)
        if (tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR)
        {
          selected_tracker_ = &tracker;
          double x[6];
          tracker.getTargetState(x);
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

bool LastArmorSelector::input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  if (!has_last_armor_)
    return false;
  target_matcher_.setTargetPosition(last_armor_.position);
  for (auto& trackers : id2trackers)
    for (auto& tracker : trackers.second->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST)
      {
        double x[6];
        tracker.getTargetState(x);
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

bool SameIDArmorSelector::input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  if (!has_last_armor_)
    return false;
  if (!id2trackers.count(last_armor_.id))
    return false;
  else
  {
    for (auto& tracker : id2trackers.at(last_armor_.id)->trackers_)
    {
      if (tracker.state_ == Tracker::EXIST)
      {
        selected_tracker_ = &tracker;
        double x[6];
        selected_tracker_->getTargetState(x);
        last_armor_ = Armor{ .id = selected_tracker_->target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
        has_last_armor_ = true;
        return true;
      }
    }
    return false;
  }
}

bool NewArmorSelector::input(const std::unordered_map<int, std::shared_ptr<Trackers>>& id2trackers)
{
  if (!has_last_armor_)
    return false;
  if (!id2trackers.count(last_armor_.id))
    return false;
  else
  {
    std::vector<Tracker> new_armor_trackers{};
    std::vector<Tracker> exist_armor{};
    for (auto tracker : id2trackers.at(last_armor_.id)->trackers_)
    {
      if (tracker.state_ == Tracker::NEW_ARMOR)
        new_armor_trackers.push_back(tracker);
      else if (tracker.state_ == Tracker::EXIST)
        exist_armor.push_back(tracker);
    }
    if (exist_armor.empty())
      return false;

    //    ROS_INFO("total size = %d\n", id2trackers.at(last_armor_.id)->trackers_.size());
    //    ROS_INFO("new_armor size = %d\n", new_armor_trackers.size());
    //    ROS_INFO("exist_armor size = %d\n", exist_armor.size());

    if (!new_armor_trackers.empty())
      for (auto& new_armor : new_armor_trackers)
      {
        double x_new[6];
        bool is_erase = false;
        new_armor.getTargetState(x_new);
        target_matcher_.setTargetPosition(tf2::Vector3(x_new[0], x_new[2], x_new[4]));
        for (auto& tracker : exist_armor)
        {
          double x[6];
          tracker.getTargetState(x);
          //          ROS_INFO("distance is %lf\n",
          //                   tf2::Vector3(x_new[0], x_new[2], x_new[4]).distance(tf2::Vector3(x[0], x[2], x[4])));
          //          ROS_INFO("x_new is %lf %lf %lf\n", x_new[0], x_new[2], x_new[4]);
          if (target_matcher_.input(tf2::Vector3(x[0], x[2], x[4])))
          {
            //            ROS_INFO("erasing");
            is_erase = true;
            break;
          }
        }
        if (!is_erase)
        {
          selected_tracker_ = &new_armor;
          double x[6];
          selected_tracker_->getTargetState(x);
          last_armor_ = Armor{ .id = selected_tracker_->target_id_, .position = tf2::Vector3(x[0], x[2], x[4]) };
          has_last_armor_ = true;
          ROS_INFO("USE NEW");
          return true;
        }
      }
    return false;
  }
}

Tracker* LogicSelectorBase::selected_tracker_;
Armor LogicSelectorBase::last_armor_;
bool LogicSelectorBase::has_last_armor_ = false;
}  // namespace rm_track
