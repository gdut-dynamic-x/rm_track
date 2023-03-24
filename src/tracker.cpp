//
// Created by yezi on 22-9-10.
//

#include "rm_track/tracker.h"

namespace rm_track
{
Tracker::Tracker(int id, double max_match_distance, double max_lost_time, double max_storage_time, int num_data,
                 double max_new_armor_time, double max_new_armor_match_distance, rm_track::TargetStamp& target_stamp,
                 double* initial_velocity)
  : target_id_(id)
  , max_match_distance_(max_match_distance)
  , max_lost_time_(max_lost_time)
  , max_storage_time_(max_storage_time)
  , max_new_armor_time_(max_new_armor_time)
  , max_new_armor_match_distance_(max_new_armor_match_distance)
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
  new_armor_matcher_.setMaxMatchDistance(max_new_armor_match_distance_);
  tracker_distance_.angle = 0.;
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
  double last_time_state[6];
  predictor_.getState(x);
  predictor_.getState(last_time_state);
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
    target_cache_.push_back(TargetStamp{ .stamp = targets_stamp.stamp, .target = *match_target_it });
    double z[3];
    z[0] = match_target_it->transform.getOrigin().x();
    z[1] = match_target_it->transform.getOrigin().y();
    z[2] = match_target_it->transform.getOrigin().z();
    predictor_.update(z);
    double current_state[6];
    predictor_.getState(current_state);
    tracker_distance_.angle = atan2(current_state[2], current_state[0]) - atan2(x[2], x[0]);
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

void Tracker::matchNewArmor(std::vector<Tracker>& exist_trackers)
{
  double x_new[6];
  this->getTargetState(x_new);
  for (auto& exist_tracker : exist_trackers)
  {
    new_armor_matcher_.setTargetPosition(tf2::Vector3(x_new[0], x_new[2], x_new[4]));
    {
      double x[6];
      exist_tracker.getTargetState(x);
      if (new_armor_matcher_.input(tf2::Vector3(x[0], x[2], x[4])))
      {
        this->state_ = Tracker::EXIST;
        break;
      }
    }
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
  trackers_.push_back(Tracker(id_, max_match_distance_, max_lost_time_, max_storage_time_, num_data_,
                              max_new_armor_time_, max_new_armor_match_distance_, target_stamp, v0));
}
void Trackers::updateTrackersState()
{
  ros::Time current_time = ros::Time::now();
  if (current_time - last_satisfied_time_ > max_judge_period_)
  {
    state_ = Trackers::PRECISE_AUTO_AIM;
    reconfirmation_ = true;
    is_satisfied_ = false;
    imprecise_exist_trackers_.clear();
    points_buffer_->clear();
    spinning_ = false;
    points_buffer_ = std::make_shared<std::vector<std::vector<double>>>(
        std::vector<std::vector<double>>(points_num_, std::vector<double>(3, 0.)));
    if ((current_time - last_satisfied_time_).toSec() > 1.)  /// todo: TEST: longer time to clear average_filter
    {
      this->average_filter_->clear();
      idx_ = 0;
    }
  }
}
bool Trackers::attackModeDiscriminator(Tracker* selected_tracker)
{
  bool have_new_armor = false;
  for (auto& tracker : imprecise_exist_trackers_)
  {
    if (tracker.state_ == Tracker::NEW_ARMOR)
      have_new_armor = true;
  }
  if (have_new_armor)
  {
    if (imprecise_exist_trackers_.size() != 1)
    {
      spinning_ = (ros::Time::now() - last_spinning_time_).toSec() < max_spinning_time_;
      ROS_INFO("%lf", (ros::Time::now() - last_spinning_time_).toSec());
      last_spinning_time_ = ros::Time::now();
    }
  }
  else
    spinning_ = false;
  current_angle_ = selected_tracker->tracker_distance_.angle;
  if (spinning_)
  {
    last_satisfied_time_ = ros::Time::now();
    if (!reconfirmation_)
    {
      if (std::signbit(current_angle_) == std::signbit(last_angle_))
      {
        state_ = IMPRECISE_AUTO_AIM;
        is_satisfied_ = true;
      }
      else
      {
        is_satisfied_ = false;
        state_ = PRECISE_AUTO_AIM;
      }
    }
    else
    {
      state_ = PRECISE_AUTO_AIM;
      reconfirmation_ = false;
      is_satisfied_ = false;
    }
  }
  else
  {
    is_satisfied_ = false;
    reconfirmation_ = false;
  }
  last_angle_ = current_angle_;
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

bool Trackers::computeAttackPosition(Tracker* selected_tracker)
{
  bool is_satisfied = true;
  double state[6];
  selected_tracker->getTargetState(state);
  double xyz[3]{ state[0], state[2], state[4] };
  this->average_filter_->input(xyz);
  idx_++;
  if (idx_ < points_num_)
    is_satisfied = false;
  return is_satisfied;
}

void Trackers::getAttackState(double* attack_state)
{
  attack_state[0] = this->average_filter_->x();
  attack_state[2] = this->average_filter_->y();
  attack_state[4] = this->average_filter_->z();
  attack_state[1] = 0.;
  attack_state[3] = 0.;
  attack_state[5] = 0.;
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
