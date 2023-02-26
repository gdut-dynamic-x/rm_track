//
// Created by yezi on 22-9-10.
//

#include "rm_track/tracker.h"

namespace rm_track
{
Tracker::Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time,
                 double max_new_armor_time, rm_track::TargetStamp& target_stamp, double* initial_velocity)
  : target_id_(id)
  , max_match_distance_(max_match_distance)
  , max_lost_time_(max_lost_time)
  , max_storage_time_(max_storage_time)
  , max_new_armor_time_(max_new_armor_time)
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

  tracker_yaw_.current_yaw_diff = 0.;
  tracker_yaw_.last_yaw = 0.;
}

void Tracker::updateTracker(rm_track::TargetsStamp& targets_stamp)
{
  if (state_ == LOST)
    return;
  double x[6];
  predictor_.getState(x);
  tracker_yaw_.last_yaw = atan2(x[2], x[0]);

  predictor_.predict((targets_stamp.stamp - last_predict_time_).toSec());
  last_predict_time_ = targets_stamp.stamp;
  if (targets_stamp.targets.empty())
    return;
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

    predictor_.getState(x);
    double current_yaw = atan2(x[2], x[0]);
    tracker_yaw_.current_yaw_diff = current_yaw - tracker_yaw_.last_yaw;

    if ((state_ == APPEAR || state_ == NEW_ARMOR) &&
        (ros::Time::now() - target_cache_.front().stamp).toSec() < max_new_armor_time_)
      state_ = NEW_ARMOR;
    else
      state_ = EXIST;
    targets_stamp.targets.erase(match_target_it);
  }
  else
  {
    predictor_.getState(x);
    double current_yaw = atan2(x[2], x[0]);
    tracker_yaw_.current_yaw_diff = current_yaw - tracker_yaw_.last_yaw;
  }
}
void Tracker::updateTrackerState()
{
  if (target_cache_.empty())
    return;
  if ((ros::Time::now() - target_cache_.back().stamp).toSec() > max_lost_time_)
    state_ = LOST;
  if (state_ == NOT_SELECTABLE)
    state_ = EXIST;
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
void Trackers::addTracker(ros::Time stamp, rm_track::Target& target)
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
  trackers_.push_back(
      Tracker(id_, max_match_distance_, max_lost_time_, max_storage_time_, max_new_armor_time_, target_stamp, v0));
}
void Trackers::updateTrackersState()
{
  ros::Time current_time = ros::Time::now();
  if (current_time - last_satisfied_time_ > max_judge_period_)
  {
    state_ = Trackers::PRECISE_AUTO_AIM;
    reconfirmation_ = true;
    is_satisfied_ = false;
  }
}
bool Trackers::attackModeDiscriminator()
{
  current_average_yaw_diff_ = 0.;
  for (auto& tracker : imprecise_trackers_)
  {
    current_average_yaw_diff_ +=
        tracker.tracker_yaw_.current_yaw_diff / static_cast<double>(imprecise_trackers_.size());
  }
  if (abs(current_average_yaw_diff_) > max_follow_angle_)
  {
    last_satisfied_time_ = ros::Time::now();
    if (!reconfirmation_ && abs(last_average_yaw_diff_) > max_follow_angle_ &&
        (std::signbit(current_average_yaw_diff_) == std::signbit(last_average_yaw_diff_)))
    {
      state_ = IMPRECISE_AUTO_AIM;
      is_satisfied_ = true;
    }
    else
    {
      state_ = PRECISE_AUTO_AIM;
      reconfirmation_ = false;
      is_satisfied_ = false;
    }
    last_average_yaw_diff_ = current_average_yaw_diff_;
  }
  else
    is_satisfied_ = false;
  return is_satisfied_;
}
int Trackers::getExistTrackerNumber()
{
  int num = 0;
  for (auto& tracker : trackers_)
  {
    if (tracker.state_ == Tracker::EXIST || tracker.state_ == Tracker::NEW_ARMOR)
      num++;
  }
  return num;
}
std::vector<Tracker> Trackers::getExistTracker()
{
  std::vector<Tracker> exist_tracker;
  for (auto& tracker : trackers_)
    if (tracker.state_ == Tracker::NEW_ARMOR || tracker.state_ == Tracker::EXIST)
      exist_tracker.push_back(tracker);
  return exist_tracker;
}
}  // namespace rm_track
