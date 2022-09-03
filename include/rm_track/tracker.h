//
// Created by yezi on 2022/8/4.
//

#pragma once
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ekf/linear_kf.h"

namespace rm_track
{
struct Target
{
  int id;
  tf2::Transform transform;
  double confidence;
};

struct TargetStamp
{
  ros::Time stamp;
  Target target;
};

struct TargetsStamp
{
  ros::Time stamp;
  std::vector<Target> targets;
};

class TargetMatcher
{
public:
  void setTargetPosition(tf2::Vector3 target_position)
  {
    target_position_ = target_position;
    min_position_diff_ = DBL_MAX;
    match_successful_ = false;
  }
  void setMaxMatchDistance(double max_match_distance)
  {
    max_match_distance_ = max_match_distance;
  }
  bool input(tf2::Vector3 position)
  {
    double position_diff = position.distance(target_position_);
    if (position_diff < min_position_diff_)
    {
      match_successful_ = position_diff < max_match_distance_;
      min_position_diff_ = position_diff;
      return position_diff < max_match_distance_;
    }
    else
      return false;
  }
  const bool matchSuccessful()
  {
    return match_successful_;
  }

private:
  tf2::Vector3 target_position_;
  double max_match_distance_;
  double min_position_diff_ = DBL_MAX;
  bool match_successful_ = false;
};

class Tracker
{
public:
  enum STATE
  {
    APPEAR,
    EXIST,
    NOT_SELECTABLE,
    LOST
  } state_;

  Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time, TargetStamp& target_stamp,
          double* initial_velocity)
    : target_id_(id)
    , max_match_distance_(max_match_distance)
    , max_lost_time_(max_lost_time)
    , max_storage_time_(max_storage_time)
  {
    target_cache_.push_back(target_stamp);
    double x[6];
    x[0] = target_stamp.target.transform.getOrigin().x();
    x[1] = initial_velocity[0];
    x[2] = target_stamp.target.transform.getOrigin().y();
    x[3] = initial_velocity[1];
    x[4] = target_stamp.target.transform.getOrigin().z();
    x[5] = initial_velocity[2];
    predictor_.reset(x);
    last_predict_time_ = target_stamp.stamp;
    state_ = APPEAR;
    target_matcher_.setMaxMatchDistance(max_match_distance_);
  }

  void updateTracker(TargetsStamp& targets_stamp)
  {
    if (state_ == LOST)
      return;
    predictor_.predict((targets_stamp.stamp - last_predict_time_).toSec());
    last_predict_time_ = targets_stamp.stamp;
    if (targets_stamp.targets.empty())
      return;
    double x[6];
    predictor_.getState(x);
    auto match_target_it = targets_stamp.targets.begin();
    target_matcher_.setTargetPosition(tf2::Vector3(x[0], x[2], x[4]));
    for (auto it = targets_stamp.targets.begin(); it != targets_stamp.targets.end(); it++)
    {
      if (target_matcher_.input(it->transform.getOrigin()))
        match_target_it = it;
    }
    if (target_matcher_.matchSuccessful())
    {
      target_cache_.push_back(TargetStamp{
          .stamp = targets_stamp.stamp,
          .target = *match_target_it,
      });
      double z[3];
      z[0] = match_target_it->transform.getOrigin().x();
      z[1] = match_target_it->transform.getOrigin().y();
      z[2] = match_target_it->transform.getOrigin().z();
      predictor_.update(z);
      state_ = EXIST;
      targets_stamp.targets.erase(match_target_it);
    }
  }

  void updateTrackerState()
  {
    if ((ros::Time::now() - target_cache_.back().stamp).toSec() > max_lost_time_)
      state_ = LOST;
    for (auto it = target_cache_.begin(); it != target_cache_.end();)
    {
      if ((ros::Time::now() - it->stamp).toSec() > max_storage_time_)
      {
        it = target_cache_.erase(it);
      }
      else
        break;
    }
  }

  void getTargetState(double* x)
  {
    predictor_.getState(x);
  }

  void predict(double* x, ros::Time time)
  {
    predictor_.predict(x, (time - last_predict_time_).toSec());
  }

  int target_id_;
  std::deque<TargetStamp> target_cache_;

private:
  LinearKf predictor_;
  ros::Time last_predict_time_;
  TargetMatcher target_matcher_;
  double max_match_distance_ = 0.2;
  double max_lost_time_;
  double max_storage_time_;
};

class Trackers
{
public:
  Trackers(int id, double max_match_distance, double max_lost_time, double max_storage_time)
    : id_(id)
    , max_match_distance_(max_match_distance)
    , max_lost_time_(max_lost_time)
    , max_storage_time_(max_storage_time)
  {
  }
  void updateTracker(TargetsStamp& target_stamps)
  {
    for (auto& tracker : trackers_)
      tracker.updateTracker(target_stamps);
  }
  void addTracker(ros::Time stamp, Target& target)
  {
    TargetStamp target_stamp{ .stamp = stamp, .target = target };
    double v0[3] = { 0, 0, 0 };
    if (getExistTrackerNumber() == 1)
    {
      double x[6];
      getExistTracker().front().getTargetState(x);
      v0[0] = x[1];
      v0[1] = x[3];
      v0[2] = x[5];
    }
    trackers_.push_back(Tracker(id_, max_match_distance_, max_lost_time_, max_storage_time_, target_stamp, v0));
  }
  int getExistTrackerNumber()
  {
    int num = 0;
    for (auto& tracker : trackers_)
    {
      if (tracker.state_ == Tracker::EXIST)
        num++;
    }
    return num;
  }
  std::vector<Tracker> getExistTracker()
  {
    std::vector<Tracker> exist_tracker;
    for (auto& tracker : trackers_)
      if (tracker.state_ == Tracker::EXIST)
        exist_tracker.push_back(tracker);
    return exist_tracker;
  }
  std::vector<Tracker> trackers_;

private:
  int id_;
  double max_match_distance_;
  double max_lost_time_;
  double max_storage_time_;
};

}  // namespace rm_track
