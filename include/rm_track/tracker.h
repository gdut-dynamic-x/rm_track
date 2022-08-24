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
  Target target;
  ros::Time stamp;
};

class TargetMatcher
{
public:
  void setTargetPosition(tf2::Vector3 target_position)
  {
    target_position_ = target_position;
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

  bool match_successful_ = false;

private:
  tf2::Vector3 target_position_;
  double max_match_distance_;
  double min_position_diff_ = DBL_MAX;
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

  Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time, TargetStamp& target_stamp)
    : target_id_(id)
    , max_match_distance_(max_match_distance)
    , max_lost_time_(max_lost_time)
    , max_storage_time_(max_storage_time)
  {
    target_cache_.push_back(target_stamp);
    double x[6];
    x[0] = target_stamp.target.transform.getOrigin().x();
    x[1] = 0;
    x[2] = target_stamp.target.transform.getOrigin().y();
    x[3] = 0;
    x[4] = target_stamp.target.transform.getOrigin().z();
    x[5] = 0;
    predictor_.reset(x);
    last_predict_time_ = target_stamp.stamp;
    state_ = APPEAR;
    target_matcher_.setMaxMatchDistance(max_match_distance_);
  }

  void updateTracker(std::vector<TargetStamp>& target_stamps)
  {
    if (state_ == LOST)
      return;
    predictor_.predict((target_stamps.front().stamp - last_predict_time_).toSec());
    last_predict_time_ = target_stamps.front().stamp;
    double x[6];
    predictor_.getState(x);
    auto match_target_it = target_stamps.begin();
    target_matcher_.setTargetPosition(tf2::Vector3(x[0], x[2], x[4]));
    for (auto it = target_stamps.begin(); it != target_stamps.end(); it++)
    {
      if (target_matcher_.input(it->target.transform.getOrigin()))
        match_target_it = it;
    }
    if (target_matcher_.match_successful_)
    {
      target_cache_.push_back(*match_target_it);
      double z[3];
      z[0] = match_target_it->target.transform.getOrigin().x();
      z[1] = match_target_it->target.transform.getOrigin().y();
      z[2] = match_target_it->target.transform.getOrigin().z();
      predictor_.update(z);
      state_ = EXIST;
      target_stamps.erase(match_target_it);
    }
  }

  void updateState()
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
        it++;
    }
  }

  void getState(double* x)
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
  void updateTracker(std::vector<TargetStamp>& target_stamps)
  {
    for (auto it = trackers_.begin(); it != trackers_.end();)
      it->updateTracker(target_stamps);
  }
  void addTracker(TargetStamp& target_stamp)
  {
    trackers_.push_back(Tracker(id_, max_match_distance_, max_lost_time_, max_storage_time_, target_stamp));
  }
  std::vector<Tracker> trackers_;

private:
  int id_;
  double max_match_distance_;
  double max_lost_time_;
  double max_storage_time_;
};

}  // namespace rm_track
