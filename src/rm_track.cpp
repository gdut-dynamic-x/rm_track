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
  {
    for (int i = 0; i < selectors.size(); ++i)
    {
      if (selectors[i] == "new_armor")
      {
        logic_selectors_.push_back(new NewArmorSelector());
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
      if (selectors[i] == "closest_to_image_center")
      {
        logic_selectors_.push_back(new ClosestToImageCenterSelector());
        break;
      }
    }
  }
  else
    ROS_ERROR("No selectors are defined (namespace %s)", nh.getNamespace().c_str());

  LinearKf predictor;
  ros::NodeHandle linear_kf_nh = ros::NodeHandle(nh, "linear_kf");
  predictor.initStaticConfig(linear_kf_nh);

  apriltag_receiver_ = std::make_shared<AprilTagReceiver>(nh, id2trackers_, mutex_, max_match_distance,
                                                          "tag_track_mode_reconf", tf_buffer_, "/tag_detections");
  rm_detection_receiver_ = std::make_shared<RmDetectionReceiver>(nh, id2trackers_, mutex_, max_match_distance,
                                                                 "track_mode_reconf", tf_buffer_, "/detection");
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
    trackers.second->updateTrackersState();
    std::vector<Tracker*> new_trackers;
    std::vector<Tracker> exist_trackers;
    for (auto it = trackers.second->trackers_.begin(); it != trackers.second->trackers_.end(); marker_id++)
    {
      it->updateMarker(marker_array, marker_id);
      it->updateTrackerState();
      if (!it->target_cache_.empty() && it->state_ == Tracker::NEW_ARMOR)
        new_trackers.push_back(it.base());
      if (!it->target_cache_.empty() && it->state_ == Tracker::EXIST)
        exist_trackers.push_back(*it);
      if (it->target_cache_.empty())
        it = trackers.second->trackers_.erase(it);
      else
        it++;
    }
    for (auto new_tracker : new_trackers)
      new_tracker->matchNewArmor(exist_trackers);
    trackers.second->imprecise_exist_trackers_ = trackers.second->getExistTracker();
  }
  marker_targets_pub_.publish(marker_array);
}

bool RmTrack::selectAttackMode(Tracker* selected_tracker)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(selected_tracker->target_id_)->second;
  if (selected_tracker->target_id_ != last_id_)
  {
    last_id_ = selected_tracker->target_id_;
    return false;
  }
  last_id_ = selected_tracker->target_id_;
  return trackers->attackModeDiscriminator(selected_tracker);
}

void RmTrack::getAttackState(Tracker* selected_tracker, double* attack_state, double& accel)
{
  std::shared_ptr<Trackers> trackers = id2trackers_.find(selected_tracker->target_id_)->second;
  if (trackers->computeAttackPosition(selected_tracker))
  {
    trackers->getAttackState(attack_state);
    accel = 0.;
  }
  else
  {
    double common_track_state[6];
    ros::Time now = ros::Time::now();
    selected_tracker->getTargetState(common_track_state);
    selected_tracker->predict(common_track_state, now);
    attack_state[0] = common_track_state[0];
    attack_state[2] = common_track_state[2];
    attack_state[4] = common_track_state[4];
    attack_state[1] = common_track_state[1];
    attack_state[3] = common_track_state[3];
    attack_state[5] = common_track_state[5];
    selected_tracker->predict(common_track_state, now);
    accel = selected_tracker->targetAccelLength();
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
  double target_accel_length = 0;
  ros::Time now = ros::Time::now();
  int target_id;
  if (selected_tracker)
  {
    target_id = selected_tracker->target_id_;
    if (selectAttackMode(selected_tracker))
    {
      rm_msgs::TrackData track_data;
      track_data.header.frame_id = "odom";
      track_data.header.stamp = now;
      track_data.id = target_id;
      getAttackState(selected_tracker, x, target_accel_length);
      track_data.target_pos.x = x[0];
      track_data.target_pos.y = x[2];
      track_data.target_pos.z = x[4];
      track_data.target_vel.x = x[1];
      track_data.target_vel.y = x[3];
      track_data.target_vel.z = x[5];
      track_data.accel = target_accel_length;
      track_pub_.publish(track_data);
      return;
    }
    selected_tracker->getTargetState(x);
    selected_tracker->predict(x, now);
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
