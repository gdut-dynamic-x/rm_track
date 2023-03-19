//
// Created by qiayuan on 2021/9/25.
//

#include "rm_track/rm_track.h"
#include <rm_msgs/TrackData.h>

namespace rm_track
{
RmTrack::RmTrack(ros::NodeHandle& nh)
{
  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(10));
  double max_match_distance;
  nh.param("max_match_distance", max_match_distance, 0.2);
  double max_new_armor_distance;
  nh.param("max_new_armor_distance", max_new_armor_distance, 0.01);
  XmlRpc::XmlRpcValue filters;
  if (nh.getParam("filters", filters))
    for (int i = 0; i < filters.size(); ++i)
    {
      if (filters[i]["type"] == "height_filter")
        logic_filters_.push_back(new HeightFilter(filters[i], tf_buffer_));
      else if (filters[i]["type"] == "distance_filter")
        logic_filters_.push_back(new DistanceFilter(filters[i], tf_buffer_));
      else if (filters[i]["type"] == "confidence_filter")
        logic_filters_.push_back(new ConfidenceFilter(filters[i]));
      else if (filters[i]["type"] == "pitch_filter")
        logic_filters_.push_back(new PitchFilter(filters[i]));
      else
        ROS_ERROR("Filter '%s' does not exist", filters[i].toXml().c_str());
    }
  else
    ROS_ERROR("No filters are defined (namespace %s)", nh.getNamespace().c_str());

  XmlRpc::XmlRpcValue selectors;
  if (nh.getParam("selectors", selectors))
  {
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "new_armor")
      {
        logic_selectors_.push_back(new NewArmorSelector(max_new_armor_distance));
        break;
      }
    }
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "last_armor")
      {
        logic_selectors_.push_back(new LastArmorSelector(max_match_distance));
        break;
      }
    }
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "closet_to_light_center")
      {
        logic_selectors_.push_back(new ClosestToLightCenterSelector());
        break;
      }
    }
  }
  else
    ROS_ERROR("No selectors are defined (namespace %s)", nh.getNamespace().c_str());

  LinearKf predictor;
  ros::NodeHandle linear_kf_nh = ros::NodeHandle(nh, "linear_kf");
  predictor.initStaticConfig(linear_kf_nh);

  apriltag_receiver_ =
      std::make_shared<AprilTagReceiver>(nh, id2trackers_, mutex_, max_match_distance, tf_buffer_, "/tag_detections");
  rm_detection_receiver_ =
      std::make_shared<RmDetectionReceiver>(nh, id2trackers_, mutex_, max_match_distance, tf_buffer_, "/detection");
  track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);
}

void RmTrack::updateTrackerState()
{
  for (auto& trackers : id2trackers_)
  {
    trackers.second->updateTrackersState();
    for (auto it = trackers.second->trackers_.begin(); it != trackers.second->trackers_.end();)
    {
      it->updateTrackerState();
      if (it->target_cache_.empty())
        it = trackers.second->trackers_.erase(it);
      else
        it++;
    }
  }
}

bool RmTrack::selectAttackMode(Tracker* tracker)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(tracker->target_id_)->second;
  return trackers->attackModeDiscriminator();
}

void RmTrack::getCircleCenter(Tracker* selected_tracker, std::vector<double>& circle_center)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(selected_tracker->target_id_)->second;
  trackers->computeCircleCenter(selected_tracker);
  trackers->getCircleCenter(circle_center);
}

bool RmTrack::getAttackPosition(Tracker* selected_tracker, double* attack_point)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(selected_tracker->target_id_)->second;
  bool is_satisfied = trackers->computeAttackPosition();
  trackers->getAttackPosition(attack_point);
  ROS_INFO("is_satisfied = %d", is_satisfied);
  return is_satisfied;
}

void RmTrack::getAverageHeight(Tracker* selected_tracker, double* x, double* height)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(selected_tracker->target_id_)->second;
  if (x[4] != 0)
    trackers->height_.push_front(x[4]);
  double height_num = static_cast<double>(trackers->height_.size());
  if (height_num > 10)  // TODO: height_num parameter is waiting for measurement
    trackers->height_.pop_back();
  for (auto z : trackers->height_)
  {
    *height += z / height_num;
  }
}

void RmTrack::run()
{
  std::lock_guard<std::mutex> guard(mutex_);
  updateTrackerState();
  Tracker* selected_tracker = nullptr;
  for (auto& filter : logic_filters_)
    filter->input(id2trackers_);
  for (int i = 0; i < logic_selectors_.size(); ++i)
  {
    if (logic_selectors_[i]->input(id2trackers_))
    {
      selected_tracker = logic_selectors_[i]->output();
      break;
    }
  }
  double x[6]{ 0, 0, 0, 0, 0, 0 };
  ros::Time now = ros::Time::now();
  double target_accel_length = 0;
  int target_id;
  if (selected_tracker)
  {
    target_id = selected_tracker->target_id_;
    if (selectAttackMode(selected_tracker))
    {
      ROS_ERROR("SPINGING");
      //      double height;
      //      getAverageHeight(selected_tracker, x, &height);
      rm_msgs::TrackData track_data;
      track_data.header.frame_id = "odom";
      track_data.header.stamp = now;
      track_data.id = target_id;
      double attack_points[3];
      if (getAttackPosition(selected_tracker, attack_points))
      {
        track_data.target_pos.x = attack_points[0];
        track_data.target_pos.y = attack_points[1];
        track_data.target_pos.z = attack_points[2];
      }
      else
      {
        selected_tracker->getTargetState(x);
        track_data.target_pos.x = x[0];
        track_data.target_pos.y = x[2];
        track_data.target_pos.z = x[4];
      }
      /// diable average height
      // average height method
      //      selected_tracker->getTargetState(x);
      //      track_data.target_pos.x = x[0];
      //      track_data.target_pos.y = x[2];
      //      track_data.target_pos.z = x[4];
      //      if (height != 0)
      //        track_data.target_pos.z = height;
      /// disabel moving average points
      // moving average points method
      //      track_data.target_pos.z = [4];
      /// enable common method
      track_data.target_pos.x = x[0];
      track_data.target_pos.y = x[2];
      track_data.target_pos.z = x[4];

      track_data.target_vel.x = 0.;
      track_data.target_vel.y = 0.;
      track_data.target_vel.z = 0.;
      target_accel_length = selected_tracker->targetAccelLength();
      track_data.accel = target_accel_length;
      track_pub_.publish(track_data);
      return;
    }
    /// Disable the v*t
    selected_tracker->getTargetState(x);
    //    selected_tracker->predict(x, now);
    target_accel_length = selected_tracker->targetAccelLength();
  }
  else
    target_id = 0;
  rm_msgs::TrackData track_data;
  track_data.header.frame_id = "odom";
  track_data.header.stamp = now;
  track_data.id = target_id;
  track_data.target_pos.x = x[0];
  track_data.target_pos.y = x[2];
  track_data.target_pos.z = x[4];
  track_data.target_vel.x = x[1];
  track_data.target_vel.y = x[3];
  track_data.target_vel.z = x[5];
  track_data.accel = target_accel_length;

  track_pub_.publish(track_data);
}
}  // namespace rm_track
