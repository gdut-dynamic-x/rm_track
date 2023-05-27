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
      else if (filters[i]["type"] == "id_filter")
        logic_filters_.push_back(new IdFilter(filters[i]));
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
      else if (selectors[i] == "closest_to_image_center")
        logic_selectors_.push_back(new ClosestToImageCenterSelector());
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
  marker_targets_pub_ = nh.advertise<visualization_msgs::MarkerArray>("targets", 10);
}

void RmTrack::updateTrackerState()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker delete_all;
  delete_all.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);
  int marker_id = 0;
  for (auto& trackers : id2trackers_)
  {
    for (auto it = trackers.second->trackers_.begin(); it != trackers.second->trackers_.end(); marker_id++)
    {
      it->updateMarker(marker_array, marker_id);
      it->updateTrackerState();
      if (it->target_cache_.empty())
        it = trackers.second->trackers_.erase(it);
      else
        it++;
    }
  }
  marker_targets_pub_.publish(marker_array);
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
  double target_accel_length = 0;
  ros::Time now = ros::Time::now();
  int target_id;
  if (selected_tracker)
  {
    selected_tracker->predict(x, now);
    target_accel_length = selected_tracker->targetAccelLength();
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
  track_data.accel = target_accel_length;

  if ((ros::Time::now() - rm_detection_receiver_->lastReceiveTime()).toSec() < 0.1)
    track_pub_.publish(track_data);
}
}  // namespace rm_track
