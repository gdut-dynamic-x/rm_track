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
      else
        ROS_ERROR("Filter '%s' does not exist", filters[i].toXml().c_str());
    }
  else
    ROS_ERROR("No filters are defined (namespace %s)", nh.getNamespace().c_str());

  XmlRpc::XmlRpcValue selectors;
  if (nh.getParam("selectors", selectors))
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "last_armor")
        logic_selectors_.push_back(new LastArmorSelector(max_match_distance));
      else if (selectors[i] == "same_id_armor")
        logic_selectors_.push_back(new SameIDArmorSelector());
      else if (selectors[i] == "random_armor")
        logic_selectors_.push_back(new RandomArmorSelector());
      else
        ROS_ERROR("Selector '%s' does not exist", selectors[i].toXml().c_str());
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

  marker_targets_pos_.header.frame_id = "odom";
  marker_targets_pos_.ns = "position";
  marker_targets_pos_.action = visualization_msgs::Marker::ADD;
  marker_targets_pos_.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_targets_pos_.scale.x = 0.1;
  marker_targets_pos_.scale.y = 0.1;
  marker_targets_pos_.scale.z = 0.1;
  marker_targets_pos_.lifetime = ros::Duration(1);
  marker_targets_pos_.color.r = 1.0;
  marker_targets_pos_.color.g = 0.0;
  marker_targets_pos_.color.b = 0.0;
  marker_targets_pos_.color.a = 1.0;

  marker_targets_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::Marker>(nh, "targets_pos", 10));
}

void RmTrack::updateTrackerState()
{
  for (auto& trackers : id2trackers_)
  {
    for (auto it = trackers.second->trackers_.begin(); it != trackers.second->trackers_.end();)
    {
      it->updateTrackerState();
      if (it->target_cache_.empty())
        it = trackers.second->trackers_.erase(it);
      else
        it++;
    }
  }
  if (marker_targets_pub_->trylock())
  {
    marker_targets_pub_->msg_ = marker_targets_pos_;
    marker_targets_pub_->unlockAndPublish();
  }
}

void RmTrack::run()
{
  std::lock_guard<std::mutex> guard(mutex_);
  updateTrackerState();
  Tracker* selected_tracker = nullptr;
  for (auto& filter : logic_filters_)
    filter->input(id2trackers_);
  for (auto& selector : logic_selectors_)
  {
    if (selector->input(id2trackers_))
    {
      selected_tracker = selector->output();
      break;
    }
  }
  double x[6]{ 0, 0, 0, 0, 0, 0 };
  ros::Time now = ros::Time::now();
  int target_id;
  if (selected_tracker)
  {
    selected_tracker->predict(x, now);
    target_id = selected_tracker->target_id_;
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

  track_pub_.publish(track_data);
}
}  // namespace rm_track
