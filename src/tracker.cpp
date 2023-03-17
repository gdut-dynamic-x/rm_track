//
// Created by yezi on 22-9-10.
//

#include "rm_track/tracker.h"

namespace rm_track
{
Tracker::Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time,
                 double max_new_armor_time, rm_track::TargetStamp& target_stamp, double* initial_velocity, int num_data)
  : target_id_(id)
  , max_match_distance_(max_match_distance)
  , max_lost_time_(max_lost_time)
  , max_storage_time_(max_storage_time)
  , max_new_armor_time_(max_new_armor_time)
{
  accel_ = std::make_shared<Vector3WithFilter<double>>(num_data);

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
  tracker_distance_.distance = 0.;
  marker_pos_.header.frame_id = "odom";
  marker_pos_.ns = "position";
  marker_pos_.action = visualization_msgs::Marker::ADD;
  marker_pos_.type = visualization_msgs::Marker::SPHERE;
  marker_pos_.scale.x = 0.1;
  marker_pos_.scale.y = 0.1;
  marker_pos_.scale.z = 0.1;
  marker_pos_.color.a = 1.0;
  marker_vel_ = marker_pos_;
  marker_vel_.ns = "velocity";
  marker_vel_.type = visualization_msgs::Marker::ARROW;
  marker_vel_.scale.x = 0.03;
  marker_vel_.scale.y = 0.05;
  marker_vel_.color.r = 1.0;
  marker_vel_.color.b = marker_vel_.color.g = 0.0;
}

void Tracker::updateTracker(rm_track::TargetsStamp& targets_stamp)
{
  if (state_ == LOST)
    return;
  double x[6];
  predictor_.getState(x);

  ros::Duration dt = targets_stamp.stamp - last_predict_time_;
  predictor_.predict(dt.toSec());
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
        .stamp = targets_stamp.stamp, .target = *match_target_it, .last_target_pt = &target_cache_.back().target });
    // compute the distance between current target and last target
    tracker_distance_.distance = tf2::Vector3(target_cache_.back().last_target_pt->current_target_position)
                                     .distance(match_target_it->current_target_position);
    double z[3];
    z[0] = match_target_it->transform.getOrigin().x();
    z[1] = match_target_it->transform.getOrigin().y();
    z[2] = match_target_it->transform.getOrigin().z();
    predictor_.update(z);
    predictor_.getState(x);
    if ((state_ == APPEAR || state_ == NEW_ARMOR) &&
        (ros::Time::now() - target_cache_.front().stamp).toSec() < max_new_armor_time_)
      state_ = NEW_ARMOR;
    else
      state_ = EXIST;
    targets_stamp.targets.erase(match_target_it);
  }
  predictor_.getState(x);
  double target_vel[3]{ x[1], x[3], x[5] };
  double target_accel[3];
  for (int i = 0; i < 3; i++)
  {
    target_accel[i] = (target_vel[i] - last_target_vel_[i]) / dt.toSec();
    last_target_vel_[i] = target_vel[i];
  }
  accel_->input(target_accel);
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

void Tracker::updateMarker(visualization_msgs::MarkerArray& marker_array, int marker_id)
{
  geometry_msgs::Point target_pos;
  geometry_msgs::Quaternion quaternion;
  double x[6];
  getTargetState(x);
  target_pos.x = x[0];
  target_pos.y = x[2];
  target_pos.z = x[4];
  quaternion.w = 1.0;
  marker_pos_.pose.position = target_pos;
  marker_pos_.pose.orientation = quaternion;
  marker_pos_.id = marker_id;
  if (state_ == APPEAR)
  {
    marker_pos_.color.r = marker_pos_.color.g = 0.0;
    marker_pos_.color.b = 1.0;
  }
  else if (state_ == EXIST)
  {
    marker_pos_.color.r = marker_pos_.color.b = 0.0;
    marker_pos_.color.g = 1.0;
  }
  else if (state_ == NOT_SELECTABLE)
  {
    marker_pos_.color.g = marker_pos_.color.b = 0.0;
    marker_pos_.color.r = 1.0;
  }
  else
    return;
  marker_vel_.points.clear();
  marker_vel_.id = marker_id;
  marker_vel_.pose.orientation = quaternion;
  marker_vel_.points.push_back(target_pos);
  geometry_msgs::Point arrow_end = target_pos;
  arrow_end.x += x[1] * 0.5;
  arrow_end.y += x[3] * 0.5;
  arrow_end.z += x[5] * 0.5;
  marker_vel_.points.push_back(arrow_end);
  marker_array.markers.push_back(marker_pos_);
  marker_array.markers.push_back(marker_vel_);
}

void Trackers::addTracker(ros::Time stamp, rm_track::Target& target)
{
  TargetStamp target_stamp{ .stamp = stamp, .target = target, .last_target_pt = nullptr };
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
    current_circle_center_.clear();
    imprecise_exist_trackers_.clear();
    height_.clear();
  }
}
bool Trackers::attackModeDiscriminator()
{
  current_average_distance_diff_ = 0.;
  for (auto& tracker : imprecise_exist_trackers_)
  {
    current_average_distance_diff_ +=
        tracker.tracker_distance_.distance / static_cast<double>(imprecise_exist_trackers_.size());
  }
  //  ROS_INFO("current_average_distance_diff_ = %lf", current_average_distance_diff_);
  if (abs(current_average_distance_diff_) > max_follow_distance_)
  {
    last_satisfied_time_ = ros::Time::now();
    // TODO: judgement of positive or negative digit here may not be effective
    if (!reconfirmation_ && abs(last_average_distance_diff_) > max_follow_distance_ &&
        (std::signbit(current_average_distance_diff_) == std::signbit(last_average_distance_diff_)))
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
    last_average_distance_diff_ = current_average_distance_diff_;
  }
  else
    is_satisfied_ = false;
  return is_satisfied_;
}

void Trackers::computeCircleCenter(Tracker* selected_tracker)
{
  if (imprecise_exist_trackers_.empty())
  {
    double x_not_predict[6];
    selected_tracker->getTargetState(x_not_predict);
    current_circle_center_ = { x_not_predict[0], x_not_predict[2] };
    return;
  }
  int rand_index = 0;
  if (imprecise_exist_trackers_.size() - 1 != 0)
  {
    rand_index = rand() % (imprecise_exist_trackers_.size() - 1);
  }
  Tracker& tracker = imprecise_exist_trackers_[rand_index];
  double x[6];
  tracker.getTargetState(x);
  if (points_of_2D_plant_.size() < 3)
  {
    double x_not_predict[6];
    selected_tracker->getTargetState(x_not_predict);
    points_of_2D_plant_.push_back(std::vector<double>{ x_not_predict[0], x_not_predict[2] });
    current_circle_center_ = { x_not_predict[0], x_not_predict[2] };
    return;
  }
  /// update points
  points_of_2D_plant_.erase(points_of_2D_plant_.begin());
  points_of_2D_plant_.push_back(std::vector<double>{ x[0], x[2] });
  /// compute circle center
  double a = points_of_2D_plant_[0][0] - points_of_2D_plant_[1][0];  // x1 - x2
  double b = points_of_2D_plant_[0][1] - points_of_2D_plant_[1][1];  // y1 - y2
  double c = points_of_2D_plant_[0][0] - points_of_2D_plant_[2][0];  // x1 - x3
  double d = points_of_2D_plant_[0][1] - points_of_2D_plant_[2][1];  // y1 - y3
  double det = b * c - a * d;
  if (abs(det) < 1e-5)  // If three points are at the same line, then return.
  {
    double x_not_predict[6];
    selected_tracker->getTargetState(x_not_predict);
    current_circle_center_ = { x_not_predict[0], x_not_predict[2] };
    return;
  }
  double e = ((pow(points_of_2D_plant_[0][0], 2) - pow(points_of_2D_plant_[1][0], 2)) -
              (pow(points_of_2D_plant_[0][1], 2) - pow(points_of_2D_plant_[1][1], 2))) /
             2.;  // ((x1^2-x2^2)-(y1^2-y2^2)) / 2
  double f = ((pow(points_of_2D_plant_[0][0], 2) - pow(points_of_2D_plant_[2][0], 2)) -
              (pow(points_of_2D_plant_[0][1], 2) - pow(points_of_2D_plant_[2][1], 2))) /
             2.;  // ((x1^2-x3^2)-(y1^2-y3^2)) / 2
  double circle_center_x = -(d * e - b * f) / det;
  double circle_center_y = -(a * f - c * e) / det;
  current_circle_center_ = { circle_center_x, circle_center_y };
  ROS_ERROR("circle_center = (%lf, %lf) ", circle_center_x, circle_center_y);
  ROS_INFO("r = %lf", sqrt(pow(circle_center_x, 2) + pow(circle_center_y, 2)));
}

void Trackers::getCircleCenter(std::vector<double>& circle_center)
{
  circle_center = current_circle_center_;
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
