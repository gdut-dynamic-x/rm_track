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
  std::vector<double> target2camera_rpy;
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
    NEW_ARMOR,
    EXIST,
    NOT_SELECTABLE,
    LOST
  } state_;

  Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time, double max_new_armor_time,
          rm_track::TargetStamp& target_stamp, double* initial_velocity);
  void updateTracker(TargetsStamp& targets_stamp);
  void updateTrackerState();
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
  struct TrackerXYDiff
  {
    double current_xy_diff;
    tf2::Vector3 last_xy;
    double z_distance;
  } tracker_xy_diff;

private:
  LinearKf predictor_;
  ros::Time last_predict_time_;
  TargetMatcher target_matcher_;
  double max_match_distance_ = 0.2;
  double max_lost_time_;
  double max_new_armor_time_;
  double max_storage_time_;
};

class Trackers
{
public:
  Trackers(int id, double max_match_distance, double max_lost_time, double max_storage_time, double max_new_armor_time,
           double max_judge_period, double max_follow_area)
    : id_(id)
    , max_match_distance_(max_match_distance)
    , max_lost_time_(max_lost_time)
    , max_storage_time_(max_storage_time)
    , max_new_armor_time_(max_new_armor_time)
    , state_(Trackers::PRECISE_AUTO_AIM)
    , last_satisfied_time_(ros::Time::now())
    , max_follow_area_(max_follow_area)
    , max_judge_period_(max_judge_period)
  {
  }
  enum STATE
  {
    PRECISE_AUTO_AIM,
    IMPRECISE_AUTO_AIM,
  } state_;

  void updateTracker(TargetsStamp& target_stamps)
  {
    for (auto& tracker : trackers_)
      tracker.updateTracker(target_stamps);
  }
  void addTracker(ros::Time stamp, Target& target);
  int getExistTrackerNumber();
  void updateTrackersState();
  bool attackModeDiscriminator();
  void computeCircleCenter(Tracker* selected_tracker);
  void getCircleCenter(std::vector<double>& circle_center);
  std::vector<Tracker> getExistTracker();
  std::deque<double> height_;
  std::vector<Tracker> trackers_;
  std::vector<Tracker> imprecise_exist_trackers_;

private:
  int id_;
  double max_match_distance_;
  double max_lost_time_;
  double max_storage_time_;
  double max_new_armor_time_;

  ros::Time last_satisfied_time_;
  double max_follow_area_;
  ros::Duration max_judge_period_;
  double last_average_area_ = 0.;
  double current_average_area_ = 0.;
  bool reconfirmation_ = true;
  bool is_satisfied_;
  std::vector<std::vector<double>> points_of_2D_plant_;
  std::vector<double> current_circle_center_;
};

}  // namespace rm_track
