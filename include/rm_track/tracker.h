//
// Created by yezi on 2022/8/4.
//

#pragma once
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <rm_common/filters/filters.h>
#include "ekf/linear_kf.h"

namespace rm_track
{
struct Target
{
  int id;
  tf2::Transform transform;
  double confidence;
  double distance_to_image_center;
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

  Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time, int num_data,
          double max_new_armor_time, double max_new_armor_match_distance, rm_track::TargetStamp& target_stamp,
          double* initial_velocity);
  void updateTracker(TargetsStamp& targets_stamp);
  void updateTrackerState();
  void updateMarker(visualization_msgs::MarkerArray& marker_array, int marker_id);
  void matchNewArmor(std::vector<Tracker>& exist_trackers);
  void getTargetState(double* x)
  {
    predictor_.getState(x);
  }
  void predict(double* x, ros::Time time)
  {
    predictor_.predict(x, (time - last_predict_time_).toSec());
  }
  double targetAccelLength()
  {
    return std::sqrt(pow(accel_->x(), 2) + pow(accel_->y(), 2) + pow(accel_->z(), 2));
  }

  int target_id_;
  std::deque<TargetStamp> target_cache_;
  struct TrackerDistance
  {
    double angle;
  } tracker_distance_;

private:
  LinearKf predictor_;
  double last_target_vel_[3];
  std::shared_ptr<Vector3WithFilter<double>> accel_;
  ros::Time last_predict_time_;
  TargetMatcher target_matcher_;
  TargetMatcher new_armor_matcher_;

  visualization_msgs::Marker marker_pos_;
  visualization_msgs::Marker marker_vel_;
  double max_match_distance_ = 0.2;
  double max_lost_time_;
  double max_new_armor_time_;
  double max_new_armor_match_distance_;
  double max_storage_time_;
};

class Trackers
{
public:
  Trackers(int id, double max_match_distance, double max_lost_time, double max_storage_time, int num_data,
           double max_new_armor_time, double max_new_armor_match_distance, double max_spinning_time,
           double max_judge_period, double max_filter_deque_storage_time, int points_num)
    : id_(id)
    , max_match_distance_(max_match_distance)
    , max_lost_time_(max_lost_time)
    , max_storage_time_(max_storage_time)
    , num_data_(num_data)
    , max_new_armor_time_(max_new_armor_time)
    , max_new_armor_match_distance_(max_new_armor_match_distance)
    , max_spinning_time_(max_spinning_time)
    , max_judge_period_(max_judge_period)
    , max_filter_deque_storage_time_(max_filter_deque_storage_time)
    , points_num_(points_num)
  {
    state_ = Trackers::PRECISE_AUTO_AIM;
    last_satisfied_time_ = ros::Time::now();
    points_buffer_ = std::make_shared<std::vector<std::vector<double>>>(
        std::vector<std::vector<double>>(points_num_, std::vector<double>(3, 0.)));
    average_filter_ = std::make_shared<Vector3WithFilter<double>>(num_data_);
    idx_ = 0;
    last_spinning_time_ = ros::Time::now();
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
  bool attackModeDiscriminator(Tracker* selected_tracker);
  bool computeAttackPosition(Tracker* selected_tracker);
  void getAttackState(double* attack_state);
  std::vector<Tracker> getExistTracker();
  std::vector<Tracker> trackers_;
  std::vector<Tracker> imprecise_exist_trackers_;

private:
  int id_;
  double max_match_distance_;
  double max_lost_time_;
  double max_storage_time_;
  int num_data_;
  double max_new_armor_time_;
  double max_spinning_time_;
  double max_new_armor_match_distance_;

  ros::Time last_satisfied_time_;
  ros::Duration max_judge_period_;
  double current_angle_ = 0.;
  double last_angle_ = 0.;
  bool reconfirmation_ = true;
  bool is_satisfied_ = false;
  bool spinning_ = false;
  ros::Time last_spinning_time_;

  std::shared_ptr<std::vector<std::vector<double>>> points_buffer_;
  int idx_;
  int points_num_;
  std::shared_ptr<Vector3WithFilter<double>> average_filter_;
  double max_filter_deque_storage_time_;
};
}  // namespace rm_track
